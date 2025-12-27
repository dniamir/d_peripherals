// chip_map.rs

pub struct Field {
    pub reg: u8,
    pub offset: u8,
    pub bits: u8,
    pub writable: bool,
}

// Trait for field map providers
pub trait FieldMapProvider {
    fn get_field(name: &str) -> Option<&'static Field>;
}

// Default case where no field map is provided
pub struct NoFieldMap;

impl FieldMapProvider for NoFieldMap {
    fn get_field(_name: &str) -> Option<&'static Field> {
        None
    }
}