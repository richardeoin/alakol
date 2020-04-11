//! Chip and board specific configuration settings go here.
//extern crate stm32h7;
//use stm32h7::stm32h7x3;
// use ::bootload;

// /// TCP port to listen on
// pub const TCP_PORT: u16 = 7777;

/// PHY address
pub const ETH_PHY_ADDR: u8 = 0;

pub struct UserConfig {
    magic: u32,
    pub mac_address: [u8; 6],
    pub ip_address: [u8; 4],
    pub ip_gateway: [u8; 4],
    pub ip_prefix: u8,
    _padding: [u8; 1],
    checksum: u32,
}
pub static DEFAULT_CONFIG: UserConfig = UserConfig {
    // Locally administered MAC
    magic: 0,
    mac_address: [0x02, 0x00, 0x01, 0x02, 0x03, 0x04],
    ip_address: [10, 1, 1, 10],
    ip_gateway: [10, 1, 1, 1],
    ip_prefix: 24,
    _padding: [0u8; 1],
    checksum: 0,
};
