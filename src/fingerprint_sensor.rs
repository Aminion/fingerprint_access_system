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
/// Template slots in the sensor's Flash library (PageID 0..LIBRARY_CAPACITY).
pub const LIBRARY_CAPACITY: u16 = 200;

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
    /// Set a system parameter (e.g. baud rate index, security level).
    SetSysParam = 0x0E,
    /// Verify the 4-byte system password.
    /// Problem: Required before most other commands can run.
    VfyPwd = 0x13,
    /// Read the 256-slot occupancy bitmap for one index page (32 bytes).
    ReadIndexTable = 0x1F,
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
    /// Hardware/communication error (timeout, framing, overrun).
    Uart(embassy_stm32::usart::Error),

    /// Protocol error (invalid header or checksum mismatch).
    Protocol(&'static str),

    /// Sensor returned a non-Ok confirmation code.
    Sensor(ConfirmationCode),

    /// Received an unexpected packet type (e.g. Data when expecting Ack).
    UnexpectedPacket(PacketType),

    NoFinger,
    NoMatch,
    Timeout,
    /// Every library slot already holds a template.
    LibraryFull,
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

impl LedEffect {
    /// How long the animation runs, so a caller can wait it out without
    /// holding a sensor-command guard across the idle time.
    /// Zero for effects that hold a steady level (cycles == 0).
    pub const fn duration(&self) -> Duration {
        if self.cycles == 0 {
            Duration::from_ticks(0)
        } else {
            Duration::from_millis((self.speed as u64 * self.cycles as u64 * 32) + 100)
        }
    }
}

/// A sensor instruction: what to send and how to read the acknowledgement.
pub trait Command {
    const INSTRUCTION: Instruction;
    /// Minimum ack data bytes `decode` reads, after the confirmation code.
    const RESP_LEN: usize = 0;
    type Output;

    /// Parameter bytes following the instruction code.
    fn params(&self) -> impl AsRef<[u8]>;

    /// Decode the ack data after the confirmation code; holds at least `RESP_LEN` bytes.
    fn decode(data: &[u8]) -> Self::Output;

    fn map_err(code: ConfirmationCode) -> FingerError {
        FingerError::Sensor(code)
    }
}

/// Collect a finger image into ImageBuffer.
pub struct GenImg;

impl Command for GenImg {
    const INSTRUCTION: Instruction = Instruction::GenImg;
    type Output = ();
    fn params(&self) -> impl AsRef<[u8]> { [] }
    fn decode(_: &[u8]) {}
}

/// Generate a character file from the image into the given CharBuffer.
pub struct Img2Tz(pub u8);

impl Command for Img2Tz {
    const INSTRUCTION: Instruction = Instruction::Img2Tz;
    type Output = ();
    fn params(&self) -> impl AsRef<[u8]> { [self.0] }
    fn decode(_: &[u8]) {}
}

/// Merge the CharBuffers into a single template.
pub struct RegModel;

impl Command for RegModel {
    const INSTRUCTION: Instruction = Instruction::RegModel;
    type Output = ();
    fn params(&self) -> impl AsRef<[u8]> { [] }
    fn decode(_: &[u8]) {}
}

/// Store the template in `buffer` into library `slot`.
pub struct Store {
    pub buffer: u8,
    pub slot: u16,
}

impl Command for Store {
    const INSTRUCTION: Instruction = Instruction::Store;
    type Output = ();
    fn params(&self) -> impl AsRef<[u8]> {
        let [sh, sl] = self.slot.to_be_bytes();
        [self.buffer, sh, sl]
    }
    fn decode(_: &[u8]) {}
}

/// Search `count` library slots from `start` for the template in `buffer`.
/// A miss is reported as `FingerError::NoMatch`.
pub struct Search {
    pub buffer: u8,
    pub start: u16,
    pub count: u16,
}

pub struct SearchMatch {
    pub slot: u16,
    pub score: u16,
}

impl Command for Search {
    const INSTRUCTION: Instruction = Instruction::Search;
    const RESP_LEN: usize = 4;
    type Output = SearchMatch;
    fn params(&self) -> impl AsRef<[u8]> {
        let [ph, pl] = self.start.to_be_bytes();
        let [ch, cl] = self.count.to_be_bytes();
        [self.buffer, ph, pl, ch, cl]
    }
    fn decode(data: &[u8]) -> SearchMatch {
        SearchMatch {
            slot: u16::from_be_bytes([data[0], data[1]]),
            score: u16::from_be_bytes([data[2], data[3]]),
        }
    }
    fn map_err(code: ConfirmationCode) -> FingerError {
        match code {
            ConfirmationCode::NotFound => FingerError::NoMatch,
            code => FingerError::Sensor(code),
        }
    }
}

/// Occupancy bitmap for slots page*256 .. page*256+255:
/// slot n is bit (n % 8) of byte (n / 8), set when the slot holds a template.
pub struct ReadIndexTable(pub u8);

impl Command for ReadIndexTable {
    const INSTRUCTION: Instruction = Instruction::ReadIndexTable;
    const RESP_LEN: usize = 32;
    type Output = [u8; 32];
    fn params(&self) -> impl AsRef<[u8]> { [self.0] }
    fn decode(data: &[u8]) -> [u8; 32] {
        let mut table = [0u8; 32];
        table.copy_from_slice(&data[..32]);
        table
    }
}

/// Verify the 4-byte system password.
pub struct VfyPwd(pub [u8; 4]);

impl Command for VfyPwd {
    const INSTRUCTION: Instruction = Instruction::VfyPwd;
    type Output = ();
    fn params(&self) -> impl AsRef<[u8]> { self.0 }
    fn decode(_: &[u8]) {}
}

/// Running an effect configures the LED ring; it returns once the sensor
/// acknowledges, not when the animation ends (see `led_await`).
impl Command for LedEffect {
    const INSTRUCTION: Instruction = Instruction::AuraConfig;
    type Output = ();
    fn params(&self) -> impl AsRef<[u8]> {
        [self.mode as u8, self.speed, self.color as u8, self.cycles]
    }
    fn decode(_: &[u8]) {}
}

/// Largest instruction + params payload we ever send.
const MAX_PAYLOAD: usize = 16;
/// Start code, address, type and length ahead of the payload; checksum after it.
const MAX_PACKET: usize = MAX_PAYLOAD + 11;
/// Largest length field we accept: payload plus the 2-byte checksum.
const MAX_BODY: usize = 128;

struct Frame {
    packet_type: PacketType,
    body: [u8; MAX_BODY],
    len: usize,
}

impl Frame {
    /// Payload without the trailing checksum.
    fn data(&self) -> &[u8] {
        &self.body[..self.len - 2]
    }
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
        self.run(VfyPwd(self.password)).await
    }
    pub fn disable(&mut self) {
        self.enable_pin.set_high();
    }

    /// Send `cmd` and decode its acknowledgement.
    pub async fn run<C: Command>(&mut self, cmd: C) -> Result<C::Output, FingerError> {
        let params = cmd.params();
        let params = params.as_ref();
        let mut payload = [0u8; MAX_PAYLOAD];
        payload[0] = C::INSTRUCTION as u8;
        payload[1..=params.len()].copy_from_slice(params);

        let (pkt, len) = packet(PacketType::Command, &self.address, &payload[..=params.len()]);
        self.uart.write(&pkt[..len]).await?;

        let frame = self.receive_packet().await?;
        if frame.packet_type != PacketType::Acknowledgement {
            return Err(FingerError::UnexpectedPacket(frame.packet_type));
        }
        let (&code, data) = frame
            .data()
            .split_first()
            .ok_or(FingerError::Protocol("Empty Acknowledgement"))?;
        let code: ConfirmationCode = code
            .try_into()
            .map_err(|_| FingerError::Protocol("Unknown Confirmation Code"))?;
        if code != ConfirmationCode::Ok {
            return Err(C::map_err(code));
        }
        if data.len() < C::RESP_LEN {
            return Err(FingerError::Protocol("Short Acknowledgement"));
        }
        Ok(C::decode(data))
    }

    async fn receive_packet(&mut self) -> Result<Frame, FingerError> {
        let mut header = [0u8; 9];
        let uart = &mut self.uart;

        with_timeout(PACKET_TIMEOUT, uart.read(&mut header))
            .await
            .map_err(|_| FingerError::Timeout)?
            .map_err(FingerError::Uart)?;

        if u16::from_be_bytes([header[0], header[1]]) != START_CODE {
            return Err(FingerError::Protocol("Invalid Start Code"));
        }

        let packet_type = PacketType::try_from(header[6])
            .map_err(|_| FingerError::Protocol("Invalid Packet Type"))?;

        let len = u16::from_be_bytes([header[7], header[8]]) as usize;
        let mut body = [0u8; MAX_BODY];
        if len < 2 || len > body.len() {
            return Err(FingerError::Protocol("Invalid Length"));
        }
        with_timeout(PACKET_TIMEOUT, uart.read(&mut body[..len]))
            .await
            .map_err(|_| FingerError::Timeout)?
            .map_err(FingerError::Uart)?;

        let received_checksum = u16::from_be_bytes([body[len - 2], body[len - 1]]);
        let (_, expected_checksum) = compute_checksum(packet_type, &body[..len - 2]);
        if received_checksum != expected_checksum {
            return Err(FingerError::Protocol("Checksum mismatch"));
        }

        Ok(Frame { packet_type, body, len })
    }

    /// Note: this keeps the caller inside its command window for the whole
    /// animation. Where the WAKE pin must stay observable meanwhile, run the
    /// effect under the guard and wait `effect.duration()` outside it.
    pub async fn led_await(&mut self, effect: LedEffect) -> Result<(), FingerError> {
        self.run(effect).await?;
        Timer::after(effect.duration()).await;
        Ok(())
    }

    /// Lowest empty library slot.
    pub async fn find_free_slot(&mut self) -> Result<u16, FingerError> {
        let table = self.run(ReadIndexTable(0)).await?;
        (0..LIBRARY_CAPACITY)
            .find(|&slot| table[slot as usize / 8] & (1 << (slot % 8)) == 0)
            .ok_or(FingerError::LibraryFull)
    }
}

/// Frame `data` into a packet; returns the buffer and the used length.
fn packet(
    packet_type: PacketType,
    address: &[u8; 4],
    data: &[u8],
) -> ([u8; MAX_PACKET], usize) {
    let mut pkt = [0u8; MAX_PACKET];
    let n = data.len();

    let (len, sum) = compute_checksum(packet_type, data);

    pkt[0..2].copy_from_slice(&START_CODE.to_be_bytes());

    // 4 bytes of Address
    pkt[2..6].copy_from_slice(address);

    pkt[6] = packet_type as u8;
    pkt[7..9].copy_from_slice(&len.to_be_bytes());
    pkt[9..9 + n].copy_from_slice(data);
    pkt[9 + n..9 + n + 2].copy_from_slice(&sum.to_be_bytes());

    (pkt, n + 11)
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
