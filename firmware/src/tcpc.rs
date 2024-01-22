// Interface over i2c to the FUSB307B USB-PD type C port controller

use core::panic;

use bitfield::bitfield;
use esp32c3_hal::i2c::I2C;
use esp32c3_hal::peripherals::I2C0;
use esp32c3_hal::prelude::*;
use esp32c3_hal::Delay;
use esp_println::println;
use heapless;
use zerocopy::AsBytes;

use crate::usb_pd;

const ADDRESS: u8 = 0x50;
const BUF_SIZE: usize = 28; // for both tx and rx buffers

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum PDState {
    // Port partner (SOURCE) is expected to blast source capabilities on cable plug
    WaitingForSourceCaps,
    // Hopefully the source accepts our request
    WaitingForPsAccept,
    // Source should notify us when the power rail has stabilized at target voltage
    WaitingForPsRdy,
    // We're done with the important bits.
    NegotiationComplete,
}

// TODO: find a way to clean up the unsafe - this never runs from interrupt context
pub fn run_state_machine(
    i2c: &mut I2C<'_, I2C0>,
    rx_header: &usb_pd::MessageHeader,
    rx_buffer: &heapless::Vec<u8, BUF_SIZE>,
) -> PDState {
    static mut STATE: PDState = PDState::WaitingForSourceCaps;
    let msg_type = rx_header.message_type();
    let is_ctrl_msg = rx_header.num_data_objects() == 0;
    unsafe {
        match STATE {
            PDState::WaitingForSourceCaps => {
                if !is_ctrl_msg && msg_type == usb_pd::DataMessage::SourceCaps as u16 {
                    let pdos = parse_pdos(rx_header, rx_buffer);
                    let index = select_pdo_index(&pdos).unwrap() as u32;
                    request_pdo_index(i2c, index, &pdos);
                    STATE = PDState::WaitingForPsAccept
                }
            }

            PDState::WaitingForPsAccept => {
                if is_ctrl_msg && msg_type == usb_pd::ControlMessage::Accept as u16 {
                    STATE = PDState::WaitingForPsRdy;
                }
            }

            PDState::WaitingForPsRdy => {
                if is_ctrl_msg && msg_type == usb_pd::ControlMessage::PsRdy as u16 {
                    STATE = PDState::NegotiationComplete;
                }
            }

            PDState::NegotiationComplete => {
                // TODO
            }
        }
        STATE
    }
}

pub fn init(i2c: &mut I2C<'_, I2C0>, delay: &mut Delay) {
    // Reset the chip
    let mut reset = Reset(0x00);
    reset.set_sw_rst(true);
    write_reg(i2c, Register::Reset, &reset.0);

    delay.delay_ms(20u32);

    // Clear the "chip has reset" fault
    let mut fault_stat = FaultStat(0x00);
    fault_stat.set_all_regs_reset(true);
    write_reg(i2c, Register::FaultStat, &fault_stat.0);

    // Enable vbus voltage monitoring and auto discharge of vbus
    let mut pwr_ctrl = PwrCtrl(0x60); // default
    pwr_ctrl.set_dis_vbus_mon(false);
    pwr_ctrl.set_auto_disch(true);
    write_reg(i2c, Register::PwrCtrl, &pwr_ctrl.0);

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

pub fn establish_pd_contract(i2c: &mut I2C<'_, I2C0>) {
    loop {
        // Wait for a message to come in
        let mut alertl = AlertL(0x00);
        while !(alertl.recieve_status()) {
            alertl.0 = read_reg(i2c, Register::AlertL);
        }

        // if we get here a message has been received
        let rx_header = get_rx_header(i2c);
        let rx_buffer = get_rx_buffer(i2c);

        // Clear alert
        let mut alertl = AlertL(0x00);
        alertl.set_recieve_status(true);
        write_reg(i2c, Register::AlertL, &alertl.0);

        let state = run_state_machine(i2c, &rx_header, &rx_buffer);

        if state == PDState::NegotiationComplete {
            return;
        }
    }
}

fn _get_vbus_mv(i2c: &mut I2C<'_, I2C0>) -> u32 {
    let mut vbus: u16 = 0;
    vbus.as_bytes_mut()[0] = read_reg(i2c, Register::VbusVoltageL);
    vbus.as_bytes_mut()[1] = read_reg(i2c, Register::VbusVoltageH);
    vbus as u32 * 25 // LSB = 25mV
}

fn parse_pdos(
    rx_header: &usb_pd::MessageHeader,
    rx_buffer: &heapless::Vec<u8, BUF_SIZE>,
) -> heapless::Vec<usb_pd::FixedSupplyPDO, 7> {
    // 28 byte buffer / 4 bytes per PDO = 7 PDOs max
    let mut all_pdos = heapless::Vec::<usb_pd::FixedSupplyPDO, 7>::new();
    for i in 0..rx_header.num_data_objects() {
        let offset: usize = i as usize * 4usize;

        let mut new_pdo = usb_pd::FixedSupplyPDO(0x00);
        new_pdo.0 = u32::from_le_bytes(rx_buffer[offset..offset + 4].try_into().unwrap());
        all_pdos.push(new_pdo).unwrap();
    }

    all_pdos
}

// Pick which PDO we want. No PPS for now, only fixed PDOs.
// Add one before requesting
fn select_pdo_index(pdos: &heapless::Vec<usb_pd::FixedSupplyPDO, 7>) -> Option<u8> {
    let mut max_voltage_mv = 0;
    let mut max_power_mw = 0;
    let mut max_power_index: i32 = -1;

    for (i, pdo) in pdos.iter().enumerate() {
        // boost converter current throughput is lower below 13V
        if pdo.is_fixed_pdo() && pdo.voltage_mv() > 13_000 {
            if pdo.voltage_mv() > 22_000 {
                // hardware can only handle 22V on the input
                continue;
            }

            // Prefer a higher voltage PDO even if it's at the same power level
            let power_mw = pdo.voltage_mv() * pdo.current_ma() / 1000;
            if power_mw > max_power_mw
                || (pdo.voltage_mv() > max_voltage_mv && power_mw >= max_power_mw)
            {
                max_voltage_mv = pdo.voltage_mv();
                max_power_mw = power_mw;
                max_power_index = i as i32;
            }
        }
    }

    if max_power_index == -1 {
        None
    } else {
        Some(max_power_index as u8)
    }
}

pub fn request_pdo_index(
    i2c: &mut I2C<'_, I2C0>,
    pdo_index: u32,
    all_pdos: &heapless::Vec<usb_pd::FixedSupplyPDO, 7>,
) {
    let mut rdo = usb_pd::FixedVariableRDO(0x00);
    let pdo_current = all_pdos[pdo_index as usize].current_10ma_units();
    rdo.set_object_position(pdo_index + 1);
    rdo.set_minimum_operating_current_10ma_units(pdo_current);
    rdo.set_operating_current_10ma_units(pdo_current);
    rdo.set_no_usb_suspend(true);

    let mut rdo_header = usb_pd::MessageHeader(0x00);
    rdo_header.set_port_power_role(false); // Sink
    rdo_header.set_num_data_objects(1);
    rdo_header.set_message_type(usb_pd::ControlMessage::Request as u16);
    rdo_header.set_message_id(0b001); // ?
    rdo_header.set_pd_spec_revision(0b01); // PD 2.0
    transmit_message(i2c, &mut rdo_header, rdo.0.as_bytes());
}

// Take a mutable pointer to the message header so we can fill in the message ID
// Block write the entire 28 byte TX buffer in one i2c transaction
fn transmit_message(i2c: &mut I2C<'_, I2C0>, tx_header: &mut usb_pd::MessageHeader, data: &[u8]) {
    // 3 bit, needs to roll over @ 0b111
    static mut MESSAGE_ID: u16 = 0;

    if data.len() > BUF_SIZE {
        panic!("TX data count greater than buffer size!");
    }

    // Safety: this function is never called from interrupt context
    unsafe {
        tx_header.set_message_id(MESSAGE_ID);
        MESSAGE_ID += 1;
        if MESSAGE_ID > 0b111 {
            MESSAGE_ID = 0;
        }
    }

    // Copy tx header to tcpc
    let header = tx_header.0.as_bytes();
    write_reg(i2c, Register::TxHeadH, &header[1]);
    write_reg(i2c, Register::TxHeadL, &header[0]);

    // Write tx byte count, add 2 bytes for header
    let tx_byte_count: u8 = 2 + data.len() as u8;
    write_reg(i2c, Register::TxByteCnt, &tx_byte_count);

    let mut tx_buf: [u8; BUF_SIZE + 1] = [0; BUF_SIZE + 1];
    tx_buf[0] = Register::TxDataMin as u8;
    for (i, byte) in data.iter().enumerate() {
        tx_buf[i + 1] = *byte;
    }
    i2c.write(ADDRESS, &tx_buf).unwrap();

    // Kick off transmission to SOP
    let mut transmit = Transmit(0x00);
    transmit.set_retry_count(0b11); // 3 retries
    transmit.set_transmit_sop_message(0b000); // SOP message
    write_reg(i2c, Register::Transmit, &transmit.0); // go!
}

fn get_rx_header(i2c: &mut I2C<'_, I2C0>) -> usb_pd::MessageHeader {
    let mut header = usb_pd::MessageHeader(0x00);

    header.0.as_bytes_mut()[0] = read_reg(i2c, Register::RxHeadL);
    header.0.as_bytes_mut()[1] = read_reg(i2c, Register::RxHeadH);

    header
}

// Block read entire RX buffer
fn get_rx_buffer(i2c: &mut I2C<'_, I2C0>) -> heapless::Vec<u8, BUF_SIZE> {
    // rxbytecnt includes the two byte header and sop byte, ignore them.
    let num_bytes: usize = read_reg(i2c, Register::RxByteCnt) as usize - 3;

    if num_bytes > BUF_SIZE {
        panic!("RX data count greater than fifo size!");
    }

    let mut rx_buf: [u8; BUF_SIZE] = [0; BUF_SIZE];
    i2c.write_read(ADDRESS, &[Register::RxDataMin as u8], &mut rx_buf)
        .unwrap();

    heapless::Vec::<u8, BUF_SIZE>::from_slice(&rx_buf[0..num_bytes]).unwrap()
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
    RxStat = 0x31,
    RxDetect = 0x2F,
    SinkTransmit = 0x40,
    StdOutCfg = 0x18,
    TcpcCtrl = 0x19,
    Transmit = 0x50,
    RxHeadL = 0x32,
    RxHeadH = 0x33,
    RxDataMin = 0x34,
    RxDataMax = 0x4F,
    TxByteCnt = 0x51,
    TxHeadL = 0x52,
    TxHeadH = 0x53,
    TxDataMin = 0x54,
    TxDataMax = 0x6F,
    PwrCtrl = 0x1C,
    VbusVoltageL = 0x70,
    VbusVoltageH = 0x71,
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
    dis_vbus_mon, set_dis_vbus_mon : 6;
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
    retry_count, set_retry_count: 5, 4;
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
