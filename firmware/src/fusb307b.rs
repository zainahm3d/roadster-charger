use bitfield::bitfield;

pub const ADDRESS: u8 = 0x50;

#[repr(u8)]
#[allow(dead_code)]
pub enum Register {
    Ccstat = 0x1D,
    Reset = 0xA2,
    SinkTransmit = 0x40,
    Transmit = 0x50,
    RxDetect = 0x2F,
    RxByteCnt = 0x30,
    TcpcCtrl = 0x19,
    FaultStat = 0x1F,
    RoleCtrl = 0x1A,
    StdOutCfg = 0x18,
    Command = 0x23,
}
