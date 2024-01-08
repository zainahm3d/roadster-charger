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

bitfield! {
    struct FaultCtrl(u8);
    impl Debug;
    pub disch_timer_dis, set_disch_timer_dis : 3;
    pub vconn_ocp_dis, set_vconn_ocp_dis: 0;
}

bitfield! {
    struct PwrCtrl(u8);
    impl Debug;
    pub vbus_mon, set_vbus_mon : 6;
    pub dis_valrm, set_dis_valrm: 5;
    pub auto_disch, set_auto_disch: 4;
    pub en_bleed_disch, set_en_bleed_disch: 3;
    pub force_disch, set_force_disch: 2;
    pub vconn_pwr, set_vconn_pwr: 1;
    pub en_vconn, set_en_vconn: 0;
}

bitfield! {
    struct CCStat(u8);
    impl Debug;
    pub look4con, _: 5;
    pub con_res, _: 4;
    pub cc2_stat, _: 3, 2;
    pub cc1_stat, _: 1, 0;
}

bitfield! {
    struct PwrStat(u8);
    impl Debug;
    pub debug_acc, _: 7;
    pub tcpc_init, _: 6;
    pub source_hv, _: 5;
    pub source_vbus, _: 4;
    pub vbus_val_en, _: 3;
    pub vconn_val, _: 1;
    pub snkvbus, _: 0;
}

bitfield! {
    struct FaultStat(u8);
    impl Debug;
    pub all_regs_reset, _: 7;
    pub auto_disch_fail, _: 5;
    pub force_disch_fail, _: 4;
    pub vconn_ocp, _: 1;
    pub i2c_err, _: 0;
}

bitfield! {
    struct RxDetect(u8);
    impl Debug;
    pub en_cable_rst, set_en_cable_rst: 6;
    pub en_hard_rst, set_en_hard_reset: 5;
    pub en_sop2_dbg, set_en_sop2_dbg: 4;
    pub en_sop1_dbg, set_en_sop1_dbg: 3;
    pub en_sop2, set_en_sop2: 2;
    pub en_sop1, set_en_sop1: 1;
    pub en_sop, set_en_sop: 0;
}

bitfield! {
    struct RxStat(u8);
    impl Debug;
    pub recieved_sop_message, _: 2, 0;
}

bitfield! {
    struct Transmit(u8);
    impl Debug;
    pub retry_counter, _: 5, 4;
    pub transmit_sop_message, set_transmit_sop_message: 2, 0;
}

bitfield! {
    struct SinkTransmit(u8);
    impl Debug;
    pub dis_sink_tx, set_dis_sink_tx: 6;
    pub retry_counter, _: 5,4;
    pub transmit_sop_message, set_transmit_sop_message: 2, 0;
}
