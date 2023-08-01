#[repr(C)]
pub struct MotionControlCommand {
    pub velocity: [u32; 2],
}

impl MotionControlCommand {
    pub const fn new() -> Self {
        Self { 
            velocity: [0u32; 2],
        }
    }
}