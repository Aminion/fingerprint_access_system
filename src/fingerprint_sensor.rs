use core::{fmt, iter::Sum, ops::Add};
use embassy_stm32::{
    pac::{bdma::Ch, i2c::vals::Headr},
    usart::{BasicInstance, RxDma, TxDma, Uart, UartRx, UartTx},
};
use num_enum::TryFromPrimitive;

/// Standard start code for all R503 packets (High byte: 0xEF, Low byte: 0x01)
pub const START_CODE: u16 = 0xEF01;
pub const START_CODE_H: u8 = (START_CODE >> 8) as u8;
pub const START_CODE_L: u8 = (START_CODE & 0xFF) as u8;

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
}

#[repr(u8)]
pub enum LedMode {
    Breathing = 0x01,
    Flashing = 0x02,
    AlwaysOn = 0x03,
    AlwaysOff = 0x04,
    GradualOn = 0x05,
    GradualOff = 0x06,
}

#[repr(u8)]
pub enum LedColor {
    Red = 0x01,
    Blue = 0x02,
    Purple = 0x03,
}

/// Implementation to allow embassy's UART error to be automatically
/// converted into FingerError when using the '?' operator.
impl From<embassy_stm32::usart::Error> for FingerError {
    fn from(err: embassy_stm32::usart::Error) -> Self {
        FingerError::Uart(err)
    }
}

// Standard formatting (used by {} and panic)
impl core::fmt::Display for FingerError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Uart(e) => write!(f, "UART Hardware Error: {:?}", e),
            Self::Protocol(m) => write!(f, "Protocol Error: {}", m),
            Self::Sensor(c) => write!(f, "Sensor Error: {:?}", c),
            Self::UnexpectedPacket(p) => write!(f, "Unexpected Packet Type: {:?}", p),
            Self::NoFinger => write!(f, "No Finger Detected"),
            Self::NoMatch => write!(f, "No Match Found"),
        }
    }
}

/// Optional: If you use defmt for logging, this helps print the errors
#[cfg(feature = "defmt")]
impl defmt::Format for FingerError {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            FingerError::Uart(_) => defmt::write!(fmt, "FingerError::Uart"),
            FingerError::Protocol(msg) => defmt::write!(fmt, "FingerError::Protocol({})", msg),
            FingerError::Sensor(code) => defmt::write!(fmt, "FingerError::Sensor({:?})", code),
            FingerError::UnexpectedPacket(p) => {
                defmt::write!(fmt, "FingerError::UnexpectedPacket({:?})", p)
            }
            FingerError::NoFinger => defmt::write!(fmt, "FingerError::NoFinger"),
            FingerError::NoMatch => defmt::write!(fmt, "FingerError::NoMatch"),
        }
    }
}

pub struct FingerprintSensor<'a, T: BasicInstance, TXDMA, RXDMA> {
    uart: Uart<'a, T, TXDMA, RXDMA>,
    address: [u8; 4],
    password: [u8; 4],
}

impl<'a, T, TXDMA, RXDMA> FingerprintSensor<'a, T, TXDMA, RXDMA>
where
    T: BasicInstance,
    TXDMA: TxDma<T>, // This satisfies the bound for .write()
    RXDMA: RxDma<T>, // This satisfies the bound for .read()
{
    pub fn new(uart: Uart<'a, T, TXDMA, RXDMA>, address: [u8; 4], password: [u8; 4]) -> Self {
        Self {
            uart,
            address,
            password,
        }
    }

    pub async fn led_breathing_infinite(&mut self, color: LedColor) -> Result<(), FingerError> {
        self.control_led(LedMode::Breathing, 0xFF, color, 0x00).await
    }

    pub async fn led_off(&mut self) -> Result<(), FingerError> {
        self.control_led(LedMode::AlwaysOff, 0, LedColor::Red, 0).await
    }

    pub async fn control_led(
        &mut self,
        mode: LedMode,
        speed: u8,
        color: LedColor,
        cycles: u8,
    ) -> Result<(), FingerError> {
        // Data = Instruction (0x35) + Mode + Speed + Color + Cycles
        let payload = [0x35, mode as u8, speed, color as u8, cycles];

        let cmd = packet(PacketType::Command, &self.address, &payload);
        self.uart.write(&cmd).await?;

        let mut rx_buf = [0u8; 12];
        self.uart.read(&mut rx_buf).await?;

        let (_, _, res_payload, _) = try_parse_packet(&rx_buf)?;

        // Check confirmation code at res_payload[0]
        if res_payload[0] == 0x00 {
            Ok(())
        } else {
            Err(FingerError::Sensor(res_payload[0].try_into().unwrap()))
        }
    }
    pub async fn verify_password(&mut self) -> Result<(), FingerError> {
        let cmd = Self::create_verify_packet(&self.address, &self.password);

        self.uart.write(&cmd).await?;

        // 3. Read the 12-byte response packet
        let mut rx_buf = [0u8; 12];
        self.uart.read(&mut rx_buf).await?;

        let (packet_type, _, payload, _) = try_parse_packet(&rx_buf)?;

        if packet_type != PacketType::Acknowledgement {
            return Err(FingerError::UnexpectedPacket(packet_type));
        }

        // 6. Check the Confirmation Code (Byte index 9)
        let code: ConfirmationCode = payload[0]
            .try_into()
            .map_err(|_| FingerError::Protocol("Unknown Confirmation Code"))?;
        match code {
            ConfirmationCode::Ok => {
                defmt::info!("Password verified. Sensor unlocked.");
                Ok(())
            }
            _ => {
                defmt::error!("Password verification failed: {:?}", code);
                Err(FingerError::Sensor(code))
            }
        }
    }

    pub async fn generate_image(&mut self) -> Result<(), FingerError> {
        // GenImg has no data, so we pass an empty array
        let cmd = packet(
            PacketType::Command,
            &self.address,
            &[Instruction::GenImg as u8],
        );

        self.uart.write(&cmd).await?;

        let mut rx_buf = [0u8; 12];
        self.uart.read(&mut rx_buf).await?; // Or your while loop

        let (_, _, payload, _) = try_parse_packet(&rx_buf)?;

        let code: ConfirmationCode = payload[0]
            .try_into()
            .map_err(|_| FingerError::Protocol("Unknown Confirmation Code"))?;

        match code {
            ConfirmationCode::Ok => Ok(()),
            ConfirmationCode::NoFinger => Err(FingerError::NoFinger),
            _ => Err(FingerError::Sensor(code)),
        }
    }

    pub async fn image_to_template(&mut self, buffer_id: u8) -> Result<(), FingerError> {
        // Data = Instruction (1 byte) + BufferID (1 byte)
        let payload = [Instruction::Img2Tz as u8, buffer_id];
        let cmd = packet(PacketType::Command, &self.address, &payload);

        self.uart.write(&cmd).await?;

        let mut rx_buf = [0u8; 12];
        self.uart.read(&mut rx_buf).await?;

        let (_, _, res_payload, _) = try_parse_packet(&rx_buf)?;

        let code: ConfirmationCode = res_payload[0]
            .try_into()
            .map_err(|_| FingerError::Protocol("Unknown Confirmation Code"))?;

        match code {
            ConfirmationCode::Ok => Ok(()),
            _ => Err(FingerError::Sensor(code)),
        }
    }

    pub async fn store_template(&mut self, buffer_id: u8, slot: u16) -> Result<(), FingerError> {
        // Data = Instruction (1 byte) + BufferID (1 byte) + PageID (2 bytes)
        let slot_bytes = slot.to_be_bytes();
        let payload = [
            Instruction::Store as u8,
            buffer_id,
            slot_bytes[0],
            slot_bytes[1],
        ];

        let cmd = packet(PacketType::Command, &self.address, &payload);
        self.uart.write(&cmd).await?;

        let mut rx_buf = [0u8; 12];
        self.uart.read(&mut rx_buf).await?;

        let (_, _, res_payload, _) = try_parse_packet(&rx_buf)?;

        let code: ConfirmationCode = res_payload[0]
            .try_into()
            .map_err(|_| FingerError::Protocol("Unknown Confirmation Code"))?;

        match code {
            ConfirmationCode::Ok => {
                defmt::info!("Template stored successfully in slot {}", slot);
                Ok(())
            }
            _ => Err(FingerError::Sensor(code)),
        }
    }

    pub async fn create_model(&mut self) -> Result<(), FingerError> {
        // Instruction 0x05: Combines Buffer 1 and Buffer 2
        // result is stored back in BOTH Buffer 1 and 2 as a "Model"
        let payload = [0x05];
        let cmd = packet(PacketType::Command, &self.address, &payload);

        self.uart.write(&cmd).await?;

        let mut rx_buf = [0u8; 12];
        self.uart.read(&mut rx_buf).await?;

        let (_, _, res_payload, _) = try_parse_packet(&rx_buf)?;
        let code: ConfirmationCode = res_payload[0]
            .try_into()
            .map_err(|_| FingerError::Protocol("Unknown Code"))?;

        match code {
            ConfirmationCode::Ok => Ok(()),
            _ => Err(FingerError::Sensor(code)),
        }
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
        // Parameters:
        // 1. Instruction: 0x04
        // 2. Buffer ID: Which RAM buffer to compare (1 or 2)
        // 3. Start Page: Where to begin searching (2 bytes)
        // 4. Page Num: How many slots to search (2 bytes)

        let start_bytes = start_page.to_be_bytes();
        let count_bytes = count.to_be_bytes();

        let payload = [
            0x04,
            buffer_id,
            start_bytes[0],
            start_bytes[1],
            count_bytes[0],
            count_bytes[1],
        ];

        let cmd = packet(PacketType::Command, &self.address, &payload);
        self.uart.write(&cmd).await?;

        // Search response is 16 bytes:
        // [Header(2), Addr(4), PID(1), Len(2), Code(1), PageID(2), Score(2), Checksum(2)]
        let mut rx_buf = [0u8; 16];
        self.uart.read(&mut rx_buf).await?;

        let (_, _, res_payload, _) = try_parse_packet(&rx_buf)?;

        let code: ConfirmationCode = res_payload[0]
            .try_into()
            .map_err(|_| FingerError::Protocol("Unknown Confirmation Code"))?;

        match code {
            ConfirmationCode::Ok => {
                // Page ID is at payload[1..3]
                let page_id = u16::from_be_bytes([res_payload[1], res_payload[2]]);
                // Match Score is at payload[3..5]
                let score = u16::from_be_bytes([res_payload[3], res_payload[4]]);

                defmt::info!("Match found! Slot: {}, Score: {}", page_id, score);
                Ok((page_id, score))
            }
            ConfirmationCode::NoFinger => {
                defmt::warn!("No match found in the specified database range.");
                Err(FingerError::NoMatch)
            }
            _ => Err(FingerError::Sensor(code)),
        }
    }
}

pub fn try_parse_packet(data: &[u8]) -> Result<(PacketType, u16, &[u8], u16), FingerError> {
    // 1. Check Header (Indices 0, 1)
    let header = data
        .get(0..2)
        .map(|b| u16::from_be_bytes([b[0], b[1]]))
        .ok_or(FingerError::Protocol("Missing Header"))?;

    if header != START_CODE {
        return Err(FingerError::Protocol("Invalid Start Code"));
    }

    // 2. Parse Packet Type (Index 6)
    let packet_type = data
        .get(6)
        .copied()
        .ok_or(FingerError::Protocol("Missing Packet Type"))
        .and_then(|b| {
            PacketType::try_from(b).map_err(|_| FingerError::Protocol("Invalid Packet Type"))
        })?;

    // 3. Parse Length (Indices 7, 8) -> Should be 7..9
    let len = data
        .get(7..9)
        .map(|b| u16::from_be_bytes([b[0], b[1]]))
        .ok_or(FingerError::Protocol("Missing Length"))?;

    // Math Check: Total packet size is 9 + len.
    // Example: If len is 7, total packet is 16 bytes.
    let total_len = 9 + len as usize;

    // 4. Extract Payload (Starts at index 9, ends 2 bytes before the total end)
    let payload_end = total_len - 2;
    let payload = data
        .get(9..payload_end)
        .ok_or(FingerError::Protocol("Missing Payload"))?;

    // 5. Extract Checksum (The last two bytes of the packet)
    let checksum = data
        .get(payload_end..total_len)
        .map(|b| u16::from_be_bytes([b[0], b[1]]))
        .ok_or(FingerError::Protocol("Missing Checksum"))?;

    Ok((packet_type, len, payload, checksum))
}

pub fn packet<const N: usize>(
    packet_type: PacketType,
    address: &[u8; 4],
    data: &[u8; N],
) -> [u8; N + 11] {
    let mut pkt = [0u8; N + 11];
    let (len, sum) = checksum(packet_type, data);

    pkt[0] = START_CODE_H;
    pkt[1] = START_CODE_L;

    // 4 bytes of Address
    pkt[2..6].copy_from_slice(address);

    pkt[6] = packet_type as u8;

    let (len_h, len_l) = split_to_bytes(len);
    pkt[7] = len_h;
    pkt[8] = len_l;

    pkt[9..9 + N].copy_from_slice(data);

    let (sum_h, sum_l) = split_to_bytes(sum);
    pkt[9 + N] = sum_h;
    pkt[9 + N + 1] = sum_l;

    pkt
}

pub fn split_to_bytes(value: u16) -> (u8, u8) {
    let high_byte = (value >> 8) as u8;
    let low_byte = (value & 0xFF) as u8;
    (high_byte, low_byte)
}

pub fn checksum(packet_type: PacketType, data: &[u8]) -> (u16, u16) {
    let len = (data.len() + 2) as u16;
    let (len_high_byte, len_low_byte) = split_to_bytes(len);
    let mut sum: u16 = 0;
    sum = sum + packet_type as u16;
    sum = sum + len_high_byte as u16;
    sum = sum + len_low_byte as u16;

    for &b in data.iter() {
        sum = sum.wrapping_add(b as u16);
    }
    (len, sum)
}
