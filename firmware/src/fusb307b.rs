use bitfield::bitfield;

const ADDRESS: u8 = 0x50;

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
    STD_OUT_CFG = 0x18,
    COMMAND = 0x23,
}

// write read/write/modify register macros

bitfield! {
    pub struct RoleCtrl(u8);
    reserved, _: 7;
    drp, _: 6;
    rp_val, _: 4, 5;
    cc2_term, _: 3, 2;
    cc1_term, _: 0, 1;
}

// use bitvec