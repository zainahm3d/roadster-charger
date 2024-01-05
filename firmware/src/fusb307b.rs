#[repr(u8)]
pub enum Register {
    CCSTAT = 0x1D,
    RESET = 0xA2,
    SINK_TRANSMIT = 0x40,
    TRANSMIT = 0x50,
    RXDETECT = 0x2F,
    RXBYTECNT = 0x30,
    TCPC_CTRL = 0x19,
    FAULTSTAT = 0x1F,
    ROLECTRL = 0x1A,
}
