// Interface over i2c to the FUSB307B USB-PD type C port controller

use bitfield::bitfield;
use esp32c3_hal::i2c::I2C;
use esp32c3_hal::peripherals::I2C0;
use esp32c3_hal::prelude::*;
use esp_println::println;

use crate::usb_pd;

pub fn init(i2c: &mut I2C<'_, I2C0>) {
    // Reset the chip
    let mut reset = Reset(0x00);
    reset.set_sw_rst(true);
    i2c.write(ADDRESS, &[Register::Reset as u8, reset.0])
        .unwrap();

    // Clear the "chip has reset" fault
    let mut fault_stat = FaultStat(0x00);
    fault_stat.set_all_regs_reset(true);
    write_reg(i2c, Register::FaultStat, &fault_stat.0);

    // Tell this thing it's not a dual role port
    let mut role_ctrl = RoleCtrl(0x00);
    role_ctrl.set_cc1_term(0b10); // Present Rd
    role_ctrl.set_cc2_term(0b10); // Present Rd
    write_reg(i2c, Register::RoleCtrl, &role_ctrl.0);

    // Read orientation of connected USB-C cable
    let cc_stat = CCStat(read_reg(i2c, Register::CCStat));
    let mut role_ctrl = RoleCtrl(0x00);
    let mut tcpc_ctrl = TcpcCtrl(0x00);

    if cc_stat.cc1_stat() != 0 {
        tcpc_ctrl.set_orient(false);
        role_ctrl.set_cc1_term(0b10); // present Rd
        role_ctrl.set_cc2_term(0b11); // open
        println!("CC1 connected");
    } else if cc_stat.cc2_stat() != 0 {
        tcpc_ctrl.set_orient(true);
        role_ctrl.set_cc1_term(0b11); // open
        role_ctrl.set_cc2_term(0b10); // present Rd
        println!("CC2 connected");
    } else {
        role_ctrl.set_cc1_term(0b10); // Rd
        role_ctrl.set_cc2_term(0b10); // Rd
        println!("Neither CC line is connected");
    }

    write_reg(i2c, Register::TcpcCtrl, &tcpc_ctrl.0);
    write_reg(i2c, Register::RoleCtrl, &role_ctrl.0);

    // Enable cc line transmissions as a sink
    let mut sink_transmit = SinkTransmit(0x00);
    sink_transmit.set_dis_sink_tx(false);
    sink_transmit.set_retry_count(0b11); // 3 retries (max)
    write_reg(i2c, Register::SinkTransmit, &sink_transmit.0);

    // Enable SOP RX detection and auto goodcrc responses (if we get sourcecaps now we must respond
    // within 100ms or power will be cut by the source!)
    let mut rx_detect = RxDetect(0x00);
    rx_detect.set_en_sop(true);
    write_reg(i2c, Register::RxDetect, &rx_detect.0);
}

fn write_reg(i2c: &mut I2C<'_, I2C0>, register: Register, byte: &u8) {
    i2c.write(ADDRESS, &[register as u8, *byte]).unwrap();
}

fn read_reg(i2c: &mut I2C<'_, I2C0>, register: Register) -> u8 {
    let mut buffer: [u8; 1] = [0x00];
    i2c.write_read(ADDRESS, &[register as u8], &mut buffer)
        .unwrap();
    buffer[0]
}

pub const ADDRESS: u8 = 0x50;

#[repr(u8)]
#[allow(dead_code)]
pub enum Register {
    AlertL = 0x10,
    AlertH = 0x11,
    CCStat = 0x1D,
    Command = 0x23,
    FaultStat = 0x1F,
    Reset = 0xA2,
    RoleCtrl = 0x1A,
    RxByteCnt = 0x30,
    RxDetect = 0x2F,
    SinkTransmit = 0x40,
    StdOutCfg = 0x18,
    TcpcCtrl = 0x19,
    Transmit = 0x50,
}

bitfield! {
    struct AlertL(u8);
    impl Debug;
    vbus_alarm_high, set_vbus_alarm_high: 7;
    tx_success, set_tx_success: 6;
    tx_discarded, set_tx_discarded: 5;
    tx_failed, set_tx_failed: 4;
    recieved_hard_reset, set_recieved_hard_reset: 3;
    recieve_status, set_recieve_status: 2;
    port_power, set_port_power: 1;
    cc_stat_changed, set_cc_stat_changed: 0;
}

bitfield! {
    struct AlertH(u8);
    impl Debug;
    vendor_defined_alert, set_vendor_defined_alert: 7;
    vbus_sink_disconnect, set_vbus_sink_disconnect: 3;
    rx_buffer_overflow, set_rx_buffer_overflow: 2;
    fault_alarm, set_fault_alarm: 1;
    voltage_alarm_low, set_voltage_alarm_low: 0;
}

bitfield! {
    struct FaultCtrl(u8);
    impl Debug;
    disch_timer_dis, set_disch_timer_dis : 3;
    vconn_ocp_dis, set_vconn_ocp_dis: 0;
}

bitfield! {
    struct PwrCtrl(u8);
    impl Debug;
    vbus_mon, set_vbus_mon : 6;
    dis_valrm, set_dis_valrm: 5;
    auto_disch, set_auto_disch: 4;
    en_bleed_disch, set_en_bleed_disch: 3;
    force_disch, set_force_disch: 2;
    vconn_pwr, set_vconn_pwr: 1;
    en_vconn, set_en_vconn: 0;
}

bitfield! {
    struct CCStat(u8);
    impl Debug;
    look4con, _: 5;
    con_res, _: 4;
    cc2_stat, _: 3, 2;
    cc1_stat, _: 1, 0;
}

bitfield! {
    struct PwrStat(u8);
    impl Debug;
    debug_acc, _: 7;
    tcpc_init, _: 6;
    source_hv, _: 5;
    source_vbus, _: 4;
    vbus_val_en, _: 3;
    vconn_val, _: 1;
    snkvbus, _: 0;
}

bitfield! {
    struct FaultStat(u8);
    impl Debug;
    all_regs_reset, set_all_regs_reset: 7;
    auto_disch_fail, set_auto_disch_fail: 5;
    force_disch_fail, set_force_disch_fail: 4;
    vconn_ocp, set_vconn_ocp: 1;
    i2c_err, set_i2c_err_: 0;
}

bitfield! {
    struct RxDetect(u8);
    impl Debug;
    en_cable_rst, set_en_cable_rst: 6;
    en_hard_rst, set_en_hard_reset: 5;
    en_sop2_dbg, set_en_sop2_dbg: 4;
    en_sop1_dbg, set_en_sop1_dbg: 3;
    en_sop2, set_en_sop2: 2;
    en_sop1, set_en_sop1: 1;
    en_sop, set_en_sop: 0;
}

bitfield! {
    struct RxStat(u8);
    impl Debug;
    recieved_sop_message, _: 2, 0;
}

bitfield! {
    struct Transmit(u8);
    impl Debug;
    retry_count, _: 5, 4;
    transmit_sop_message, set_transmit_sop_message: 2, 0;
}

bitfield! {
    struct SinkTransmit(u8);
    impl Debug;
    dis_sink_tx, set_dis_sink_tx: 6;
    retry_count, set_retry_count: 5,4;
    transmit_sop_message, set_transmit_sop_message: 2, 0;
}

bitfield! {
    struct Reset(u8);
    impl Debug;
    pd_rst, set_pd_rst: 1;
    sw_rst, set_sw_rst: 0;
}

bitfield! {
    struct RoleCtrl(u8);
    impl Debug;
    drp, set_drp: 6;
    rp_val, set_rp_val: 5, 4;
    cc2_term, set_cc2_term: 3, 2;
    cc1_term, set_cc1_term: 1, 0;
}

bitfield! {
    struct TcpcCtrl(u8);
    impl Debug;
    en_watchdog, set_en_watchdog: 5;
    debug_acc_ctrl, set_debug_acc_ctrl: 4;
    bist_tmode, set_bist_tmode: 1;
    orient, set_orient: 0;
}