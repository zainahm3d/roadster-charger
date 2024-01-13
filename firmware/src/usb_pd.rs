// Minimum field definitions to sink current from a USB PD source

use bitfield::bitfield;

// #[repr(u8)]
// pub enum ControlMessage {
//     GoodCrc = 0b0_0001,
//     GetSourceCap = 0b0_0111,
//     Accept = 0b0_0011,
//     Reject = 0b0_0100,
// }

bitfield! {
    pub struct MessageHeader(u16);
    impl Debug;
    pub extended, set_extended: 15;
    pub num_data_objects, set_num_data_objects: 14, 12;
    pub message_id, set_message_id: 11, 9;
    pub port_power_role, set_port_power_role: 8;
    pub pd_spec_revision, set_pd_spec_revision: 7, 6;
    pub port_data_role, set_port_data_role: 5;
    pub message_type, set_message_type: 4, 0;
}

bitfield! {
    pub struct FixedSupplyPDO(u32);
    impl Debug;
    pub fixed_supply, _: 31, 30;
    pub dual_role_power, _: 29;
    pub usb_suspend_supported, _: 28;
    pub unconstrained_power, _: 27;
    pub usb_communications_capable, _: 26;
    pub dual_role_data, _: 25;
    pub unchunked_extended_messages_supported, _: 24;
    pub epr_mode_capable, _: 23;
    pub peak_current, _: 21, 20;
    pub voltage_50mv_units, _: 19, 10;
    pub current_10ma_units, _: 9, 0;
}

impl FixedSupplyPDO {
    pub fn voltage_mv(&self) -> u32 {
        self.voltage_50mv_units() * 50
    }

    pub fn current_ma(&self) -> u32 {
        self.current_10ma_units() * 10
    }
}

bitfield! {
    pub struct FixedVariableRDO(u32);
    impl Debug;
    pub _, set_object_position: 31, 28;
    pub _, set_giveback_flag: 27;
    pub _, set_capability_mismatch: 26;
    pub _, set_usb_communications_capable: 25;
    pub _, set_no_usb_suspend: 24;
    pub _, set_unchunked_extended_messages_supported: 23;
    pub _, set_epr_mode_capable: 22;
    pub _, set_operating_current_10ma_units: 19, 10;
    pub _, set_minimum_operating_current_10ma_units: 9, 0;
}
