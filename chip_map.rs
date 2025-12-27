// chip_map.rs

#[derive(Copy, Clone)]
pub struct Field {
    pub reg: u8,
    pub offset: u8,
    pub bits: u8,
    pub writable: bool,
}

// Trait for field map providers
pub trait FieldMapProvider {
    fn get_read_field(name: &str) -> Option<Field>;
    fn get_write_field(name: &str) -> Option<Field>;
}

// Default case where no field map is provided
pub struct NoFieldMap;

impl FieldMapProvider for NoFieldMap {
    fn get_read_field(_name: &str) -> Option<Field> {
        None
    }
    fn get_write_field(_name: &str) -> Option<Field> {
        None
    }
}