// Minimum field definitions to sink current from a USB PD source

use bitfield::bitfield;

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
    pub struct FixedVariableRequest(u32);
    impl Debug;
    pub _, set_object_position: 31, 28;
    pub _, set_giveback_flag: 27;
    pub _, set_capability_mismatch: 26;
    pub _, set_usb_communications_capable: 25;
    pub _, set_no_usb_suspend: 24;
    pub _, set_unchunked_extended_messages_supported: 23;
    pub _, set_epr_mode_capabge: 22;
    pub _, set_operating_current_10ma_units: 19, 10;
    pub _, set_minimum_operating_current_10ma_units: 9, 0;
}
