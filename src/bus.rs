use crate::cpu::Mem;

const RAM_BASE: u16 = 0x0000;
const RAM_MIRRORS_END: u16 = 0x2000;
const PPU_BASE: u16 = 0x2000;
const PPU_MIRRORS_END: u16 = 0x4000;
pub struct Bus {
    cpu_vram: [u8; 2048]
}

impl Bus {
    pub fn new() -> Self{
        Bus {
            cpu_vram: [0; 2048]
        }
    }
}

impl Mem for Bus {
    fn mem_read(&self, addr: u16) -> u8 {
        match addr {
            RAM_BASE..RAM_MIRRORS_END => {
                let mirror_down_addr = addr & 0b0000_0111_1111_1111;
                self.cpu_vram[mirror_down_addr as usize]
            }
            PPU_BASE..PPU_MIRRORS_END => {
                todo!("PPU")
            }

            _ => {
                println!("Erorr: check addr");
                0
            }
        }
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        match addr {
            RAM_BASE..RAM_MIRRORS_END => {
                let mirror_down_addr = addr & 0b0000_0111_1111_1111;
                self.cpu_vram[mirror_down_addr as usize] = data;
            }

            PPU_BASE..PPU_MIRRORS_END => {
                todo!("PPU")
            }

            _ => {
                println!("Erorr: check addr");
            }
        }
    }
}