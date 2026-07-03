use embassy_stm32::{
    bind_interrupts, dma, exti,
    gpio::Output,
    usart::{InterruptHandler, Uart},
};
use embassy_stm32::mode::Async;
use embassy_time::{with_timeout, Duration, Timer};
use num_enum::TryFromPrimitive;

/// Standard start code for all R503 packets (High byte: 0xEF, Low byte: 0x01)
pub const START_CODE: u16 = 0xEF01;
const POWER_UP_BYTE: u8 = 0x55;
const ENABLE_TIMEOUT: Duration = Duration::from_millis(2000);
const PACKET_TIMEOUT: Duration = Duration::from_millis(1000);

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, TryFromPrimitive)]
pub enum Instruction {
    /// Collect finger image and store in ImageBuffer.
    /// Problem: Returns NoFinger if sensor is empty.
    GenImg = 0x01,
    /// Generate character file from image and store in CharBuffer1/2.
    /// Problem: Returns ImageMessy if the scan quality is poor.
    Img2Tz = 0x02,
    /// Precisely compare templates in CharBuffer1 and CharBuffer2.
    /// Problem: Returns NoMatch if the two scans differ.
    Match = 0x03,
    /// Search the library for a template matching CharBuffer1/2.
    /// Problem: Returns NotFound if no match exists in Flash.
    Search = 0x04,
    /// Combine multiple character files into a single template.
    /// Problem: Returns EnrollMismatch if scans were inconsistent.
    RegModel = 0x05,
    /// Store a template from CharBuffer into a specific Flash location.
    /// Problem: Returns BadLocation if ID is out of bounds.
    Store = 0x06,
    /// Verify the 4-byte system password.
    /// Problem: Required before most other commands can run.
    VfyPwd = 0x13,
    /// Configure the LED indicator (color, speed, mode).
    /// Problem: Returns InvalidReg if parameters are out of range.
    AuraConfig = 0x35,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, TryFromPrimitive, defmt::Format)]

pub enum PacketType {
    /// Command packet sent by the MCU.
    Command = 0x01,
    /// Data packet containing image or template bytes.
    Data = 0x02,
    /// Response packet sent by the sensor after a command.
    Acknowledgement = 0x07,
    /// The final packet in a multi-packet data stream.
    EndData = 0x08,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, TryFromPrimitive, defmt::Format)]
pub enum ConfirmationCode {
    /// Command executed successfully.
    Ok = 0x00,
    /// Packet was corrupted during transmission (checksum mismatch).
    PacketReceiveErr = 0x01,
    /// No finger detected on the sensor surface.
    NoFinger = 0x02,
    /// Failed to capture image (dirty sensor or dry skin).
    ImageFail = 0x03,
    /// Image is too "noisy" or blurry to process.
    ImageMessy = 0x06,
    /// Image lacks enough distinct ridge features.
    FeatureFail = 0x07,
    /// The two fingerprints compared do not match.
    NoMatch = 0x08,
    /// Scanned finger was not found in the database.
    NotFound = 0x09,
    /// Multiple enrollment scans were too different to merge.
    EnrollMismatch = 0x0A,
    /// The requested Flash Page ID is outside the valid range.
    BadLocation = 0x0B,
    /// Error accessing the fingerprint library (Flash read error).
    DbRangeFail = 0x0C,
    /// Data transfer error during feature upload.
    UploadFeatureFail = 0x0D,
    /// Sensor is busy or failed to respond to the data flow.
    PacketResponseFail = 0x0E,
    /// Error uploading the raw image to the MCU.
    UploadFail = 0x0F,
    /// Failed to delete the requested fingerprint template.
    DeleteFail = 0x10,
    /// Failed to clear the entire fingerprint database.
    DbClearFail = 0x11,
    /// Incorrect 4-byte password provided.
    PasswordFail = 0x13,
    /// No valid image found in the primary buffer.
    InvalidImage = 0x15,
    /// Internal Flash writing error (hardware wear/failure).
    FlashErr = 0x18,
    /// Invalid register number or system parameter.
    InvalidReg = 0x1A,
    /// Correct Address Code received.
    AddrCode = 0x20,
    /// Password verification is required before this operation.
    PassVerifyRequired = 0x21,
    /// Handshake success / Sensor is ready.
    ModuleOk = 0x55,
    /// Undefined response code from hardware.
    Unknown = 0xFF,
}

#[derive(Debug)]
pub enum FingerError {
    /// 1. Hardware/Communication errors (Timeout, Framing, Overrun)
    /// This allows us to use '?' on self.uart.read().await
    Uart(embassy_stm32::usart::Error),

    /// 2. Protocol errors (Invalid Header, Bad Checksum)
    /// This happens if the data is corrupted during transmission
    Protocol(&'static str),

    /// 3. Sensor Logic errors (The ConfirmationCode sent by the R503)
    /// This wraps the specific response codes we defined earlier
    Sensor(ConfirmationCode),

    /// 4. Unexpected Packet Type
    /// For example, receiving a Data packet when expecting an Acknowledgment
    UnexpectedPacket(PacketType),

    NoFinger,
    NoMatch,
    Timeout,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, TryFromPrimitive)]
pub enum LedMode {
    Breathing = 0x01,
    Flashing = 0x02,
    AlwaysOn = 0x03,
    AlwaysOff = 0x04,
    GradualOn = 0x05,
    GradualOff = 0x06,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, TryFromPrimitive)]
pub enum LedColor {
    Red = 0x01,
    Blue = 0x02,
    Purple = 0x03,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LedEffect {
    pub mode: LedMode,
    pub speed: u8,
    pub color: LedColor,
    pub cycles: u8,
}

pub struct Response<const N: usize> {
    pub address: [u8; 4],
    pub packet_type: PacketType,
    pub length: u16,
    pub data: [u8; N],
    pub checksum: u16,
}

/// Implementation to allow embassy's UART error to be automatically
/// converted into FingerError when using the '?' operator.
impl From<embassy_stm32::usart::Error> for FingerError {
    fn from(err: embassy_stm32::usart::Error) -> Self {
        FingerError::Uart(err)
    }
}

pub struct FingerprintSensor<'a> {
    enable_pin: Output<'a>,
    uart: Uart<'a, Async>,
    address: [u8; 4],
    password: [u8; 4],
}

bind_interrupts!(pub struct Irqs {
    USART1 => InterruptHandler<embassy_stm32::peripherals::USART1>;
    DMA1_CHANNEL1 => dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH1>;
    DMA1_CHANNEL2_3 => dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH2>;
    EXTI2_3 => exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI2_3>;
});

impl<'a> FingerprintSensor<'a> {
    pub fn new(
        enable_pin: Output<'a>,
        address: [u8; 4],
        password: [u8; 4],
        uart: Uart<'a, Async>,
    ) -> Self {
        Self {
            enable_pin,
            uart,
            address,
            password,
        }
    }

    pub async fn enable(&mut self) -> Result<(), FingerError> {
        self.enable_pin.set_low();
        let mut buf = [0u8; 1];
        {
            let uart = &mut self.uart;
            with_timeout(ENABLE_TIMEOUT, async {
                loop {
                    // The power transition leaves residue on the RX line: a
                    // break/framing error latched while the sensor TX was at 0V,
                    // plus possible noise bytes. Reading is the only way to clear
                    // the flags, so discard both errors and non-0x55 bytes until
                    // the genuine power-up handshake arrives; the outer timeout
                    // bounds a truly dead sensor.
                    if let Ok(()) = uart.read(&mut buf).await {
                        if buf[0] == POWER_UP_BYTE {
                            break;
                        }
                    }
                }
            })
            .await
            .map_err(|_| FingerError::Timeout)?;
        }
        self.verify_password().await?;
        Ok(())
    }
    pub fn disable(&mut self) {
        self.enable_pin.set_high();
    }

    async fn receive_packet<const N: usize>(&mut self) -> Result<Response<N>, FingerError> {
        let mut header = [0u8; 9];
        let uart = &mut self.uart;

        with_timeout(PACKET_TIMEOUT, uart.read(&mut header))
            .await
            .map_err(|_| FingerError::Timeout)?
            .map_err(FingerError::Uart)?;

        if u16::from_be_bytes([header[0], header[1]]) != START_CODE {
            return Err(FingerError::Protocol("Invalid Start Code"));
        }

        let address = [header[2], header[3], header[4], header[5]];
        let packet_type = PacketType::try_from(header[6])
            .map_err(|_| FingerError::Protocol("Invalid Packet Type"))?;

        let length_field = u16::from_be_bytes([header[7], header[8]]) as usize;
        let mut body = [0u8; 128];
        if length_field < 2 || length_field > body.len() {
            return Err(FingerError::Protocol("Invalid Length"));
        }
        with_timeout(PACKET_TIMEOUT, uart.read(&mut body[..length_field]))
            .await
            .map_err(|_| FingerError::Timeout)?
            .map_err(FingerError::Uart)?;

        let received_checksum = u16::from_be_bytes([body[length_field - 2], body[length_field - 1]]);
        let (_, expected_checksum) = compute_checksum(packet_type, &body[..length_field - 2]);
        if received_checksum != expected_checksum {
            return Err(FingerError::Protocol("Checksum mismatch"));
        }

        let mut data = [0u8; N];
        let data_to_copy = (length_field - 2).min(N);
        data[..data_to_copy].copy_from_slice(&body[..data_to_copy]);

        Ok(Response {
            address,
            packet_type,
            length: length_field as u16,
            data,
            checksum: received_checksum,
        })
    }

    fn ack_packet<const N: usize>(
        response: &Response<N>,
    ) -> Result<ConfirmationCode, FingerError> {
        if response.packet_type != PacketType::Acknowledgement {
            return Err(FingerError::UnexpectedPacket(response.packet_type));
        }
        let code: ConfirmationCode = response.data[0]
            .try_into()
            .map_err(|_| FingerError::Protocol("Unknown Confirmation Code"))?;

        Ok(code)
    }

    /// Send a command payload and return the RESP data bytes of the
    /// acknowledgement (data[0] is the confirmation code, already checked).
    async fn transact<const REQ: usize, const RESP: usize>(
        &mut self,
        payload: &[u8; REQ],
    ) -> Result<[u8; RESP], FingerError>
    where
        [(); REQ + 11]:,
    {
        let cmd = packet(PacketType::Command, &self.address, payload);
        self.uart.write(&cmd).await?;
        let response = self.receive_packet::<RESP>().await?;
        let ack_code = Self::ack_packet(&response)?;
        if ack_code == ConfirmationCode::Ok {
            Ok(response.data)
        } else {
            Err(FingerError::Sensor(ack_code))
        }
    }

    async fn send_command<const N: usize>(&mut self, payload: &[u8; N]) -> Result<(), FingerError>
    where
        [(); N + 11]:,
    {
        self.transact::<_, 1>(payload).await.map(|_| ())
    }

    pub async fn led_await(&mut self, effect: &LedEffect) -> Result<(), FingerError> {
        let result = self.led(effect).await;
        if effect.cycles > 0 && result.is_ok() {
            let wait_ms = (effect.speed as u32 * effect.cycles as u32 * 32) + 100;
            Timer::after_millis(wait_ms as u64).await;
        }
        result
    }

    pub async fn led(&mut self, effect: &LedEffect) -> Result<(), FingerError> {
        self.send_command(&[
            Instruction::AuraConfig as u8,
            effect.mode as u8,
            effect.speed,
            effect.color as u8,
            effect.cycles,
        ]).await
    }

    pub async fn verify_password(&mut self) -> Result<(), FingerError> {
        let mut payload = [0u8; 5];
        payload[0] = Instruction::VfyPwd as u8;
        payload[1..5].copy_from_slice(&self.password);
        self.send_command(&payload).await
    }

    pub async fn generate_image(&mut self) -> Result<(), FingerError> {
        self.send_command(&[Instruction::GenImg as u8]).await
    }

    pub async fn image_to_template(&mut self, buffer_id: u8) -> Result<(), FingerError> {
        self.send_command(&[Instruction::Img2Tz as u8, buffer_id]).await
    }

    pub async fn store_template(&mut self, buffer_id: u8, slot: u16) -> Result<(), FingerError> {
        let [sh, sl] = slot.to_be_bytes();
        self.send_command(&[Instruction::Store as u8, buffer_id, sh, sl]).await
    }

    pub async fn create_model(&mut self) -> Result<(), FingerError> {
        self.send_command(&[Instruction::RegModel as u8]).await
    }

    pub fn create_verify_packet(address: &[u8; 4], password: &[u8; 4]) -> [u8; 16] {
        let mut payload = [0u8; 5];
        payload[0] = Instruction::VfyPwd as u8;
        payload[1..5].copy_from_slice(password);
        packet(PacketType::Command, address, &payload)
    }

    pub async fn search_database(
        &mut self,
        buffer_id: u8,
        start_page: u16,
        count: u16,
    ) -> Result<(u16, u16), FingerError> {
        let [ph, pl] = start_page.to_be_bytes();
        let [ch, cl] = count.to_be_bytes();
        let payload = [Instruction::Search as u8, buffer_id, ph, pl, ch, cl];

        match self.transact::<_, 5>(&payload).await {
            Ok(data) => {
                let page_id = u16::from_be_bytes([data[1], data[2]]);
                let score = u16::from_be_bytes([data[3], data[4]]);
                Ok((page_id, score))
            }
            Err(FingerError::Sensor(ConfirmationCode::NotFound)) => Err(FingerError::NoMatch),
            Err(e) => Err(e),
        }
    }
}

fn packet<const N: usize>(
    packet_type: PacketType,
    address: &[u8; 4],
    data: &[u8; N],
) -> [u8; N + 11] {
    let mut pkt = [0u8; N + 11];

    let (len, sum) = compute_checksum(packet_type, data);

    pkt[0..2].copy_from_slice(&START_CODE.to_be_bytes());

    // 4 bytes of Address
    pkt[2..6].copy_from_slice(address);

    pkt[6] = packet_type as u8;
    pkt[7..9].copy_from_slice(&len.to_be_bytes());
    pkt[9..9 + N].copy_from_slice(data);
    pkt[9 + N..9 + N + 2].copy_from_slice(&sum.to_be_bytes());

    pkt
}

fn compute_checksum(packet_type: PacketType, data: &[u8]) -> (u16, u16) {
    let len = (data.len() + 2) as u16;
    let [len_h, len_l] = len.to_be_bytes();
    let mut sum = packet_type as u16;
    sum = sum.wrapping_add(len_h as u16);
    sum = sum.wrapping_add(len_l as u16);

    for &b in data.iter() {
        sum = sum.wrapping_add(b as u16);
    }
    (len, sum)
}
