use core::{fmt, ops::Add};
use embassy_stm32::usart::{BasicInstance, RxDma, TxDma, Uart, UartRx, UartTx};
use num_enum::TryFromPrimitive;

/// Standard start code for all R503 packets (High byte: 0xEF, Low byte: 0x01)
pub const START_CODE: u16 = 0xEF01;

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
        }
    }
}

pub struct FingerprintSensor<'a, T: BasicInstance, TXDMA, RXDMA> {
    uart: Uart<'a, T, TXDMA, RXDMA>,
    password: [u8; 4],
}

impl<'a, T, TXDMA, RXDMA> FingerprintSensor<'a, T, TXDMA, RXDMA>
where
    T: BasicInstance,
    TXDMA: TxDma<T>, // This satisfies the bound for .write()
    RXDMA: RxDma<T>, // This satisfies the bound for .read()
{
    pub fn new(uart: Uart<'a, T, TXDMA, RXDMA>, password: [u8; 4]) -> Self {
        Self { uart, password }
    }
    pub async fn verify_password(&mut self) -> Result<(), FingerError> {
        // 1. Prepare the command (Instruction 0x13 + 4-byte password)
        // We assume the default password here; you could also pass it as an argument.
        let password: [u8; 4] = [0x00, 0x00, 0x00, 0x00];
        let cmd = create_verify_packet(password);
        let serialized_cmd = cmd;

        // 2. Send the command via UART
        // The '?' operator will catch any embassy_stm32::usart::Error
        // and convert it into FingerError::Uart automatically.
        self.uart.write(&serialized_cmd).await?;

        // 3. Read the 12-byte response packet
        let mut rx_buf = [0u8; 12];
        self.uart.read(&mut rx_buf).await?;

        // 4. Validate the Packet Header
        let header = u16::from_be_bytes([rx_buf[0], rx_buf[1]]);
        if header != START_CODE {
            return Err(FingerError::Protocol("Invalid Start Code"));
        }

        // 5. Verify the Packet Type
        // Using TryFrom (from num_enum) to ensure the sensor sent an Acknowledge (0x07)
        let pid = PacketType::try_from(rx_buf[6])
            .map_err(|_| FingerError::Protocol("Invalid Packet Type"))?;

        if pid != PacketType::Acknowledgement {
            return Err(FingerError::UnexpectedPacket(pid));
        }

        // 6. Check the Confirmation Code (Byte index 9)
        let code: ConfirmationCode = rx_buf[9]
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
}

/// Creates the complete 12-byte verification packet for a given password.
/// Header(2) + Addr(4) + PID(1) + Len(2) + Ins(1) + Pwd(4) + Checksum(2) = 16 bytes total.
pub fn create_verify_packet(password: [u8; 4]) -> [u8; 16] {
    let mut pkt = [0u8; 16];

    // 1. Start Code (0xEF01)
    pkt[0] = 0xEF;
    pkt[1] = 0x01;

    // 2. Default Address (0xFFFFFFFF)
    pkt[2..6].copy_from_slice(&[0xFF, 0xFF, 0xFF, 0xFF]);

    // 3. Packet Identifier (0x01 = Command)
    pkt[6] = 0x01;

    // 4. Length (Instruction (1) + Password (4) + Checksum (2) = 7)
    // Sent as Big-Endian u16
    pkt[7] = 0x00;
    pkt[8] = 0x07;

    // 5. Instruction Code (0x13 = VfyPwd)
    pkt[9] = 0x13;

    // 6. Password Data
    pkt[10..14].copy_from_slice(&password);

    let sum = checksum(PacketType::Command, Instruction::VfyPwd, &password);
    let (high_byte, low_byte) = split_to_bytes(sum);
    pkt[14] = high_byte;
    pkt[15] = low_byte;

    pkt
}

pub fn split_to_bytes(value: u16) -> (u8, u8) {
    let high_byte = (value >> 8) as u8;
    let low_byte = (value & 0xFF) as u8;
    (high_byte, low_byte)
}

pub fn checksum(packet_type: PacketType, instruction: Instruction, data: &[u8]) -> u16 {
    let len = (1 + data.len() + 2) as u16;
    let (len_high_byte, len_low_byte) = split_to_bytes(len);
    let mut sum: u16 = 0;
    sum = sum + packet_type as u16;
    sum = sum + instruction as u16;
    sum = sum + len_high_byte as u16;
    sum = sum + len_low_byte as u16;

    for &b in data.iter() {
        sum = sum.wrapping_add(b as u16);
    }
    sum
}
