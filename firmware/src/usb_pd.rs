// Minimum field definitions to sink current from a USB PD source

use bitfield::bitfield;
#[allow(dead_code)]
#[repr(u8)]
pub enum ControlMessage {
    Request = 0b0_0010,
    Accept = 0b0_0011,
    Reject = 0b0_0100,
    PsRdy = 0b0_0110,
    GetSourceCap = 0b0_0111,
    VendorDefined = 0b0_1111,
}

#[allow(dead_code)]
pub enum DataMessage {
    SourceCaps = 0b0_0001,
    Request = 0b0_0010,
    VendorDefined = 0b0_1111,
}

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

// TODO: Make this generic for any type of PDO, and build
// another bitfield for fixed supply PDOs
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

    pub spr_pps, _: 29, 28; // only when this is an apdo
}

#[allow(dead_code)]
impl FixedSupplyPDO {
    pub fn voltage_mv(&self) -> u32 {
        self.voltage_50mv_units() * 50
    }

    pub fn current_ma(&self) -> u32 {
        self.current_10ma_units() * 10
    }

    pub fn is_fixed_pdo(&self) -> bool {
        self.fixed_supply() == 0b00
    }

    pub fn is_augmented_pdo(&self) -> bool {
        self.fixed_supply() == 0b11
    }

    pub fn is_spr_pps_apdo(&self) -> bool {
        self.is_augmented_pdo() && self.spr_pps() == 0b00
    }
}

bitfield! {
    // Standard power range programmable power supply
    // Augmented power data object (0b11)
    pub struct SprPpsApdo(u32);
    impl Debug;
    pub apdo, _: 31, 30;
    pub spr_pps, _: 29, 28;
    pub pps_power_limited, _: 27;
    pub max_voltage_100mv_units, _: 24, 17;
    pub min_voltage_100mv_units, _: 15, 8;
    pub max_current_50ma_units, _: 6, 0;
}

#[allow(dead_code)]
impl SprPpsApdo {
    pub fn max_voltage_mv(&self) -> u32 {
        self.max_voltage_100mv_units() * 100
    }

    pub fn min_voltage_mv(&self) -> u32 {
        self.min_voltage_100mv_units() * 100
    }

    pub fn max_current_ma(&self) -> u32 {
        self.max_current_50ma_units() * 50
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
    pub _, set_maximum_operating_current_10ma_units: 9, 0;
}
