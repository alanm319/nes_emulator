use std::collections::HashMap;
use bitflags::bitflags;

use crate::opcodes;

const STACK_BASE: u16 = 0x0100;
const STACK_RESET: u8 = 0xFD;

#[derive(Debug)]
pub enum AddressingMode {
    Immediate,
    ZeroPage,
    ZeroPageY,
    ZeroPageX,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    IndirectX,
    IndirectY,
    Indirect,
    NoneAddressing,
}


bitflags! {
    #[derive(Clone)]
    pub struct StatusFlags: u8 {
        const CARRY = 0b00000001;
        const ZERO  = 0b00000010;
        const INTERRUPT_DISABLE = 0b00000100;
        const DECIMAL = 0b00001000;
        const BREAK = 0b00010000;
        const UNUSED = 0b00100000;
        const OVERFLOW = 0b01000000;
        const NEGATIVE = 0b10000000;
    }
}

pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub register_y: u8,
    pub status: StatusFlags,
    pub program_counter: u16,
    pub stack_pointer: u8,
    memory: [u8; 0xFFFF]
}

impl CPU {
    pub fn new() -> CPU {
        CPU {
            register_a : 0,
            register_x: 0,
            register_y: 0,
            status: StatusFlags::from_bits_retain(0b00100100),
            program_counter: 0,
            stack_pointer: STACK_RESET,
            memory: [0; 0xFFFF]
        }
    }

    fn stack_push(&mut self, value: u8) {
        self.mem_write(STACK_BASE + self.stack_pointer as u16, value);
        self.stack_pointer = self.stack_pointer.wrapping_sub(1);
    }

    fn stack_pull(&mut self) -> u8 {
        self.stack_pointer = self.stack_pointer.wrapping_sub(1);
        self.mem_read(STACK_BASE as u16 + self.stack_pointer as u16)
    }

    fn stack_push_u16(&mut self, value: u16) {
        let low = (value & 0xFF) as u8;
        let high = (value >> 8) as u8;
        self.stack_push(high);
        self.stack_push(low);
    }

    fn stack_pull_u16(&mut self) -> u16 {
        let low = self.stack_pull() as u16;
        let high = self.stack_pull() as u16;
        (high << 8) | low
    }

    fn mem_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }

    fn mem_read_u16(&self, pos: u16) -> u16 {
        let low= self.mem_read(pos) as u16;
        let high= self.mem_read(pos+1) as u16;
        (high << 8) | low 
    }

    fn mem_write_u16(&mut self, pos: u16, data: u16) {
        let low = (data & 0xFF) as u8;
        let high = (data >> 8) as u8;
        self.mem_write(pos, low);
        self.mem_write(pos+1, high);        
    }

    // used by ADC and SBC
    // updates all flags accordingly
    fn add_to_register_a(&mut self, value: u8) {
        let carry_in: u16 = if self.status.contains(StatusFlags::CARRY) {1} else {0};
        let sum = self.register_a as u16 + value as u16 + carry_in;
        let result = sum as u8;

        if sum > 0xFF {
            self.status.insert(StatusFlags::CARRY);
        } else {
            self.status.remove(StatusFlags::CARRY);
        }
        
        if !(self.register_a ^ value) & (self.register_a ^ result) & 0x80 != 0 {
            self.status.insert(StatusFlags::OVERFLOW);
        } else {
            self.status.remove(StatusFlags::OVERFLOW);
        }
        
        self.register_a = result;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn adc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        
        self.add_to_register_a(value);
    }
    
    fn and(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_a = self.register_a & value;
        self.update_zero_and_negative_flags(self.register_a);
    }
    
    fn asl_accumulator(&mut self) {
        if self.register_a >> 7 == 1 {
            self.status.insert(StatusFlags::CARRY);
        } else {
            self.status.remove(StatusFlags::CARRY);
        }

        self.register_a = self.register_a << 1;
    }
    
    fn asl(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut value = self.mem_read(addr);
        if value >> 7 == 1 {
            self.status.insert(StatusFlags::CARRY);
        } else {
            self.status.remove(StatusFlags::CARRY);
        }

        value = value << 1;
        self.mem_write(addr, value);
        self.update_zero_and_negative_flags(value);
    }

    fn branch(&mut self, condition: bool) {
        if condition {
            let offset = self.mem_read(self.program_counter) as i8; // signed byte relative offset
            let abs_addr = self.program_counter + 1 + offset as u16;

            self.program_counter = abs_addr;
        }
    }

    fn bcc(&mut self) {
        self.branch(!self.status.contains(StatusFlags::CARRY));
    }

    fn bcs(&mut self) {
        self.branch(self.status.contains(StatusFlags::CARRY));
    }

    fn beq(&mut self) {
        self.branch(self.status.contains(StatusFlags::ZERO));
    }

    fn bit(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        if self.register_a & value == 0 {
            self.status.insert(StatusFlags::ZERO);
        } else {
            self.status.remove(StatusFlags::ZERO);
        }

        self.status.set(StatusFlags::OVERFLOW, value & 0b0100_0000 != 0);
        self.status.set(StatusFlags::NEGATIVE, value & 0b1000_0000 != 0);
    }

    fn bmi(&mut self) {
        self.branch(self.status.contains(StatusFlags::NEGATIVE));
    }

    fn bne(&mut self) {
        self.branch(!self.status.contains(StatusFlags::ZERO));
    }

    fn bpl(&mut self) {
        self.branch(!self.status.contains(StatusFlags::NEGATIVE));
    }

    fn bvc(&mut self) {
        self.branch(!self.status.contains(StatusFlags::OVERFLOW));
    }

    fn bvs(&mut self) {
        self.branch(self.status.contains(StatusFlags::OVERFLOW));
    }

    fn clc(&mut self) {
        self.status.remove(StatusFlags::CARRY);
    }

    fn cld(&mut self) {
        self.status.remove(StatusFlags::DECIMAL);
    }

    fn cli(&mut self) {
        self.status.remove(StatusFlags::INTERRUPT_DISABLE);
    }

    fn clv(&mut self) {
        self.status.remove(StatusFlags::OVERFLOW);
    }

    fn cmp(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let result = self.register_a.wrapping_sub(value);

        if self.register_a >= value {
            self.status.insert(StatusFlags::CARRY);
        } else {
            self.status.remove(StatusFlags::CARRY);
        }

        self.update_zero_and_negative_flags(result);
    }

    fn cpx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let result = self.register_x.wrapping_sub(value);

        if self.register_x >= value {
            self.status.insert(StatusFlags::CARRY)
        } else {
            self.status.remove(StatusFlags::CARRY);
        }

        self.update_zero_and_negative_flags(result);
    }

    fn cpy(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let result = self.register_y.wrapping_sub(value);

        if self.register_y >= value {
            self.status.insert(StatusFlags::CARRY)
        } else {
            self.status.remove(StatusFlags::CARRY);
        }

        self.update_zero_and_negative_flags(result);
    }

    fn dec(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        let result = value.wrapping_sub(1);
        self.update_zero_and_negative_flags(result);
    }

    fn dex(&mut self) {
        self.register_x = self.register_x.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn dey(&mut self) {
        self.register_y = self.register_y.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn eor(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_a = self.register_a ^ value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn inc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let result = value.wrapping_add(1);

        self.mem_write(addr, result);
        self.update_zero_and_negative_flags(result);
    }

    fn inx(&mut self) {
        self.register_x = self.register_x.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn iny(&mut self) {
        self.register_y = self.register_y.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn jmp(&mut self, mode: &AddressingMode) {
        match mode {
            AddressingMode::Absolute => {
                let addr = self.mem_read_u16(self.program_counter);
                self.program_counter = addr;
            }

            AddressingMode::Indirect => {
                let first_addr = self.mem_read_u16(self.program_counter);

                /*  This accounts for the 6502 indirect addressing bug:
                    https://www.nesdev.org/obelisk-6502-guide/reference.html#JMP
                 */ 
                let second_addr = if first_addr & 0x00ff == 0x00ff {
                    let low = self.mem_read(first_addr);
                    let high = self.mem_read(first_addr & 0xff00);
                    (high as u16) << 8 | (low as u16)
                } else {
                    self.mem_read_u16(first_addr)
                };

                self.program_counter = second_addr;
            }

            _ => {
                panic!("mode {:?} is not supported for jmp", mode);
            }
        }
    }

    fn jsr(&mut self) {
        self.stack_push_u16(self.program_counter + 2 - 1);
        let addr = self.mem_read_u16(self.program_counter);
        self.program_counter = addr;
    }

    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_a = value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn ldx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_x = value;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn ldy(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_y = value;
        self.update_zero_and_negative_flags(self.register_y);
    }
    
    fn lsr_accumulator(&mut self) {
        if self.register_a & 1 == 1 {
            self.status.insert(StatusFlags::CARRY);
        } else {
            self.status.remove(StatusFlags::CARRY);
        }

        self.register_a = self.register_a >> 1;
        self.update_zero_and_negative_flags(self.register_a);
    }
    
    fn lsr(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut value = self.mem_read(addr);

        if value & 1 == 1 {
            self.status.insert(StatusFlags::CARRY);
        } else {
            self.status.remove(StatusFlags::CARRY);
        }

        value = value >> 1;
        self.mem_write(addr, value);
        self.update_zero_and_negative_flags(value);
    }

    fn ora(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_a = self.register_a | value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn pha(&mut self) {
       self.stack_push(self.register_a); 
    }

    fn php(&mut self) {
        let mut flags = self.status.clone();
        flags.insert(StatusFlags::BREAK);
        flags.insert(StatusFlags::UNUSED);
        self.stack_push(flags.bits());
    }

    fn pla(&mut self) {
        self.register_a = self.stack_pull();
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn plp(&mut self) {
        self.status = StatusFlags::from_bits_retain(self.stack_pull());
        self.status.remove(StatusFlags::BREAK);
        self.status.remove(StatusFlags::UNUSED);
    }

    fn rol_accumulator(&mut self) {
        let carry_in = if self.status.contains(StatusFlags::CARRY) {1} else {0};
        if self.register_a >> 7 == 1 {
            self.status.insert(StatusFlags::CARRY);
        } else {
            self.status.remove(StatusFlags::CARRY);
        }

        self.register_a = self.register_a << 1 | carry_in;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn rol(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut value = self.mem_read(addr);
        let carry_in = if self.status.contains(StatusFlags::CARRY) {1} else {0};

        if value >> 7 == 1 {
            self.status.insert(StatusFlags::CARRY);
        } else {
            self.status.remove(StatusFlags::CARRY);
        }

        value = value << 1 | carry_in;
        self.mem_write(addr, value);
        self.update_zero_and_negative_flags(value);
    }

    fn ror_accumulator(&mut self) {
        let carry_in = if self.status.contains(StatusFlags::CARRY) {1} else {0};
        if self.register_a & 1 == 1 {
            self.status.insert(StatusFlags::CARRY);
        } else {
            self.status.remove(StatusFlags::CARRY);
        }

        self.register_a = self.register_a >> 1 | (carry_in << 7);
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn ror(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut value = self.mem_read(addr);
        let carry_in = if self.status.contains(StatusFlags::CARRY) {1} else {0};

        if value & 1 == 1 {
            self.status.insert(StatusFlags::CARRY);
        } else {
            self.status.remove(StatusFlags::CARRY);
        }

        value = value >> 1 | (carry_in << 7);
        self.mem_write(addr, value);
        self.update_zero_and_negative_flags(value);
    }

    fn rti(&mut self) {
        self.status = StatusFlags::from_bits_retain(self.stack_pull());
        self.status.remove(StatusFlags::BREAK);
        self.status.insert(StatusFlags::UNUSED);
        self.program_counter = self.stack_pull_u16();
    }

    fn rts(&mut self) {
        self.program_counter = self.stack_pull_u16() + 1;
    }

    fn sbc(&mut self, mode: &AddressingMode) {
       let addr = self.get_operand_address(mode);
       let value = self.mem_read(addr);

       self.add_to_register_a(!value);
    }

    fn sec(&mut self) {
        self.status.insert(StatusFlags::CARRY);
    }

    fn sed(&mut self) {
        self.status.insert(StatusFlags::DECIMAL);
    }

    fn sei(&mut self) {
        self.status.insert(StatusFlags::INTERRUPT_DISABLE);
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_a);
    }

    fn stx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_x);
    }

    fn sty(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_y);
    }

    fn tax(&mut self) {
        self.register_x = self.register_a;
        self.update_zero_and_negative_flags(self.register_x);
    }
    
    fn tay(&mut self) {
        self.register_y = self.register_a;
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn tsx(&mut self) {
        self.register_x = self.stack_pointer;
    }

    fn txa(&mut self) {
        self.register_a = self.register_x;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn txs(&mut self) {
        self.stack_pointer = self.register_x;
    }

    fn tya(&mut self) {
        self.register_a = self.register_y;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn update_zero_and_negative_flags(&mut self, result: u8) {

        if result == 0 {
            self.status.insert(StatusFlags::ZERO);
        } else {
            self.status.remove(StatusFlags::ZERO);
        }

        if result & 0b1000_0000 != 0 {
            self.status.insert(StatusFlags::NEGATIVE);
        } else {
            self.status.remove(StatusFlags::NEGATIVE);
        }
    }

    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.program_counter,

            AddressingMode::ZeroPage => self.mem_read(self.program_counter) as u16,

            AddressingMode::Absolute => self.mem_read_u16(self.program_counter),

            AddressingMode::ZeroPageX => {
                let pos = self.mem_read(self.program_counter);
                let addr = pos.wrapping_add(self.register_x) as u16;
                addr
            }

            AddressingMode::ZeroPageY => {
                let pos = self.mem_read(self.program_counter);
                let addr = pos.wrapping_add(self.register_y) as u16;
                addr
            }

            AddressingMode::AbsoluteX => {
                let base = self.mem_read_u16(self.program_counter);
                let addr = base.wrapping_add(self.register_x as u16);
                addr
            }

            AddressingMode::AbsoluteY => {
                let base = self.mem_read_u16(self.program_counter);
                let addr = base.wrapping_add(self.register_y as u16);
                addr
            }

            AddressingMode::IndirectX => {
                let base = self.mem_read(self.program_counter);
                let ptr: u8 = base.wrapping_add(self.register_x);
                let low = self.mem_read(ptr as u16);
                let high = self.mem_read(ptr.wrapping_add(1) as u16);
                (high as u16) << 8 | (low as u16)
            }

            AddressingMode::IndirectY => {
                let base = self.mem_read(self.program_counter);
                let ptr: u8 = base.wrapping_add(self.register_y);
                let low = self.mem_read(ptr as u16);
                let high = self.mem_read(ptr.wrapping_add(1) as u16);
                (high as u16) << 8 | (low as u16)
            }

            AddressingMode::Indirect | AddressingMode::NoneAddressing => {
                panic!("mode {:?} is not supported", mode);
            }
        }
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run()
    }

    pub fn reset(&mut self) {
        self.register_a = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.status = StatusFlags::empty();
        self.stack_pointer = STACK_RESET;
        self.program_counter = self.mem_read_u16(0xFFFC);
    }

    pub fn load(&mut self, program: Vec<u8>) {
        self.memory[0x8000 .. (0x8000 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_u16(0xFFFC, 0x8000);
    }

    pub fn run(&mut self) {
        //Hashmap where keys are actual opcodes and values are opcodes objects
        let ref opcodes_map: HashMap<u8, &'static opcodes::OpCode> = *opcodes::OPCODES_MAP;

        loop {
            let code = self.mem_read(self.program_counter);
            self.program_counter += 1;
            let opcode = opcodes_map.get(&code).expect(&format!("OpCode {:x} is not recognized", code));
            let prev_pc = self.program_counter;
            match code {

                0x69 | 0x65 | 0x75 | 0x6D | 0x7D | 0x79 | 0x61 | 0x71 => {
                    self.adc(&opcode.mode);
                }

                0x29 | 0x25 | 0x35 | 0x2D | 0x3D | 0x39 | 0x21 | 0x31 => {
                    self.and(&opcode.mode);
                }

                0x0A | 0x06 | 0x16 | 0x0E | 0x1E => {
                    self.asl(&opcode.mode);
                }

                0x90 => self.bcc(),

                0xB0 => self.bcs(),

                0xF0 => self.beq(),

                0x24 | 0x2C => self.bit(&opcode.mode),

                0x30 => self.bmi(),

                0xD0 => self.bne(),

                0x10 => self.bpl(),

                0x00 => return, // BRK

                0x50 => self.bvc(),

                0x70 => self.bvs(),

                0x18 => self.clc(),

                0xD8 => self.cld(),

                0x58 => self.cli(),

                0xB8 => self.clv(),

                0xC9 | 0xC5 | 0xD5 | 0xCD | 0xDD | 0xD9 | 0xC1 | 0xD1 => {
                    self.cmp(&opcode.mode);
                }

                0xE0 | 0xE4 | 0xEC => {
                    self.cpx(&opcode.mode);
                }

                0xC0 | 0xC4 | 0xCC => {
                    self.cpy(&opcode.mode);
                }

                0xC6 | 0xD6 | 0xCE | 0xDE => {
                    self.dec(&opcode.mode);
                }

                0xCA => self.dex(),

                0x88 => self.dey(),

                0x49 | 0x45 | 0x55 | 0x4D | 0x5D | 0x59 | 0x41 | 0x51 => {
                    self.eor(&opcode.mode);
                }

                0xE6 | 0xF6 | 0xEE | 0xFE => {
                    self.inc(&opcode.mode);
                }

                0xE8 => self.inx(),

                0xC8 => self.iny(),

                0x4C | 0x6C => self.jmp(&opcode.mode),

                0x20 => self.jsr(),
                
                0xA9 | 0xa5 | 0xad | 0xbd | 0xb9 | 0xa1 | 0xb1 => {
                    self.lda(&opcode.mode);
                }

                0xA2 | 0xA6 | 0xB6 | 0xAE | 0xBE => {
                    self.ldx(&opcode.mode);
                }

                0xA0 | 0xA4 | 0xB4 | 0xAC | 0xBC => {
                    self.ldy(&opcode.mode);
                }

                0x4A | 0x46 | 0x56 | 0x4E | 0x5E => {
                    self.lsr(&opcode.mode);
                }

                0xEA => {}

                0x09 | 0x05 | 0x15 | 0x0D | 0x1D | 0x19 | 0x01 | 0x11 => {
                    self.ora(&opcode.mode);
                }

                0x48 => self.pha(),

                0x08 => self.php(),

                0x68 => self.pla(),

                0x28 => self.plp(),

                0x2A | 0x26 | 0x36 | 0x2E | 0x3E => {
                    self.rol(&opcode.mode);
                }

                0x6A | 0x66 | 0x76 | 0x6E | 0x7E => {
                    self.ror(&opcode.mode);
                }

                0x40 => self.rti(),

                0x60 => self.rts(),

                0xE9 | 0xE5 | 0xF5 | 0xED | 0xFD | 0xF9 | 0xE1 | 0xF1 => {
                    self.sbc(&opcode.mode);
                }

                0x38 => self.sec(),

                0xF8 => self.sed(),

                0x78 => self.sei(),

                0x85 | 0x95 | 0x8D | 0x9D | 0x99 | 0x81 | 0x91 => {
                    self.sta(&opcode.mode);
                }

                0x86 | 0x96 | 0x8E => {
                    self.stx(&opcode.mode);
                }

                0x84 | 0x94 | 0x86 => {
                    self.sty(&opcode.mode);
                }

                0xAA => self.tax(),

                0xA8 => self.tay(),

                0xBA => self.tsx(),

                0x8A => self.txa(),

                0x9A => self.txs(),

                0x98 => self.tya(),

                unknown_code => panic!("{:?} is an unknown opcode", unknown_code)
            }
            
            if prev_pc == self.program_counter {
                self.program_counter += (opcode.len - 1) as u16;
            } 
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
   fn test_0xa9_lda_immediate_load_data() {
       let mut cpu = CPU::new();
       cpu.load_and_run(vec![0xa9, 0x05, 0x00]);
       assert_eq!(cpu.register_a, 0x05);
       assert!(cpu.status.contains(StatusFlags::ZERO));
       assert!(cpu.status.contains(StatusFlags::NEGATIVE));
   }

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.status.contains(StatusFlags::ZERO));
    }

    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x0a, 0xaa, 0x00]);
  
        assert_eq!(cpu.register_x, 10);
    }

    #[test]
   fn test_5_ops_working_together() {
       let mut cpu = CPU::new();
       cpu.load_and_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);
 
       assert_eq!(cpu.register_x, 0xc1);
   }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0xff, 0xaa, 0xe8, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 1);
    }

    #[test]
    fn test_lda_from_memory() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x10, 0x55);
        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);
        assert_eq!(cpu.register_a, 0x55);
    }

}