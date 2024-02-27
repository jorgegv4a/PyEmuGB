from pathlib import Path
from typing import Dict


from gb_memory import AddressSpace, RegisterBank, SingleRegister, FLAG_Z_MASK, FLAG_N_MASK, FLAG_H_MASK, FLAG_C_MASK
from gb_ops import opcodes
from gb_constants import CARTRIDGE_ROM_ONLY


class CPU:
    def __init__(self, game_rom: bytes):
        self.registers = RegisterBank()
        self.memory = AddressSpace()
        self.memory.load_rom(game_rom, CARTRIDGE_ROM_ONLY)
        self.IME = 0 # Interrupt Master Enable
        self.IF = 0 # Interrupt Flag
        self.IE = SingleRegister() # Interrupt Enable

    def fetch(self):
        opcode = self.memory[self.registers.PC]
        self.registers.increment_pc()
        return opcode

    def decode(self, opcode: int):
        if opcode != 0xCB:
            opcode_dict = opcodes["unprefixed"][f"0x{opcode:02x}"]
        else:
            opcode_lower = self.fetch()
            opcode = (opcode << 8) | opcode_lower
            opcode_dict = opcodes["cbprefixed"][f"0x{opcode_lower:02x}"]
        return opcode_dict, opcode

    def execute(self, opcode: int, opcode_dict: Dict):
        extra_bytes = []

        code_length = opcode_dict["length"] - 1
        if ((opcode >> 8) & 0xFF) == 0xCB:
            code_length -= 1
        for i in range(code_length):
            extra_bytes.append(self.fetch()) # almost always LS byte first

        if opcode == 0x00: # NOP
            print("> NOP")

        elif opcode == 0xC3: # JP a16
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            print(F"> JP 0x{immediate:04X}")
            self.registers.write_PC(immediate)

        elif opcode == 0xAF: # XOR A
            print("> XOR A")
            self.registers.A ^= self.registers.A

            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()

        elif opcode == 0x21: # LD HL, d16
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            print(F"> LD HL, 0x{immediate:04X}")
            self.registers.HL = immediate

        elif opcode == 0x0E: # LD C, d8
            immediate = extra_bytes[0]
            print(F"> LD C, 0x{immediate:02X}")
            self.registers.C = immediate

        elif opcode == 0x06: # LD B, d8
            immediate = extra_bytes[0]
            print(F"> LD B, 0x{immediate:02X}")
            self.registers.B = immediate

        elif opcode == 0x32: # LD (HL-), A
            print(F"> LD (HL-), A")
            print(F"> HL: 0x{self.registers.HL:04X}, A: 0x{self.registers.A:02X}")
            self.memory[self.registers.HL] = self.registers.A
            self.registers.HL -= 1

        elif opcode == 0x05: # DEC B
            print(F"> DEC B")
            upper_nibble_pre = (self.registers.B >> 4) & 0xF
            self.registers.B -= 1
            upper_nibble_post = (self.registers.B >> 4) & 0xF
            self.registers.set_N()
            if self.registers.B == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            if upper_nibble_pre != upper_nibble_post:
                self.registers.set_H()
            else:
                self.registers.clear_H()

        elif opcode == 0x20: # JR NZ, r8
            print(F"> JR NZ, r8")
            # signed int
            immediate = (extra_bytes[0] & 0x7F) - 128
            if not self.registers.read_Z():
                self.registers.write_PC(self.registers.PC + immediate)

        elif opcode == 0x0D: # DEC C
            print(F"> DEC C")
            upper_nibble_pre = (self.registers.C >> 4) & 0xF
            self.registers.C -= 1
            upper_nibble_post = (self.registers.C >> 4) & 0xF
            self.registers.set_N()
            if self.registers.C == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            if upper_nibble_pre != upper_nibble_post:
                self.registers.set_H()
            else:
                self.registers.clear_H()
        elif opcode == 0x3E: # LD A, d8
            immediate = extra_bytes[0]
            print(F"> LD A, 0x{immediate:02X}")
            self.registers.A = immediate

        elif opcode == 0xF3: # DI
            print(F"> DI")
            self.IME = 0

        elif opcode == 0xE0: # LDH (d8), A
            immediate = extra_bytes[0]
            print(F"> LDH (0x{immediate:02X}), A")
            self.memory[0xFF00 | immediate] = self.registers.A

        elif opcode == 0xF0: # LDH A, (d8)
            immediate = extra_bytes[0]
            print(F"> LDH (0x{immediate:02X}), A")
            self.registers.A = self.memory[0xFF00 | immediate]

        elif opcode == 0xFE: # CP d8
            immediate = extra_bytes[0]
            print(F"> CP (0x{immediate:02X}), A")
            value_pre = self.registers.A
            upper_nibble_pre = (self.registers.A >> 4) & 0xF
            self.registers.A -= immediate
            upper_nibble_post = (self.registers.A >> 4) & 0xF
            if upper_nibble_pre != upper_nibble_post:
                self.registers.set_H()
            else:
                self.registers.clear_H()
            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            if value_pre < self.registers.A:
                self.registers.set_C()
            else:
                self.registers.clear_C()
            self.registers.set_N()

        elif opcode == 0x36: # LD (HL), d8
            immediate = extra_bytes[0]
            print(F"> LD (HL), d8")
            print(F"> HL: 0x{self.registers.HL:04X}, d8: 0x{immediate:02X}")
            self.memory[self.registers.HL] = immediate

        elif opcode == 0xEA: # LD (a16), A
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            print(F"> LD (a16), A")
            print(F"> a16: 0x{immediate:04X}, A: 0x{self.registers.A:02X}")
            self.memory[immediate] = self.registers.A

        elif opcode == 0x31: # LD SP, d16
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            print(F"> LD SP, d16")
            print(F"> SP: 0x{self.registers.SP:04X}, d16: 0x{immediate:04X}")
            self.registers.SP = immediate

        elif opcode == 0x2A: # LD A, (HL+)
            print(F"> LD A, (HL+)")
            print(F"> A: 0x{self.registers.A:02X}, HL: 0x{self.registers.HL:04X}")
            self.registers.A = self.memory[self.registers.HL]
            self.registers.HL += 1

        elif opcode == 0xE2: # LD (C), A
            print(F"> LD (C), A")
            print(F"> C (0x{self.registers.C:02X}), A: {self.registers.A}")
            self.memory[0xFF00 | self.registers.C] = self.registers.A

        elif opcode == 0x0C: # INC C
            print(F"> INC C")
            upper_nibble_pre = (self.registers.C >> 4) & 0xF
            self.registers.C += 1
            upper_nibble_post = (self.registers.C >> 4) & 0xF
            self.registers.clear_N()
            if self.registers.C == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            if upper_nibble_pre != upper_nibble_post:
                self.registers.set_H()
            else:
                self.registers.clear_H()

        elif opcode == 0xCD: # CALL a16
            print(F"> CALL a16")
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            self.registers.SP -= 2
            self.memory[self.registers.SP + 1] = (self.registers.PC >> 8) & 0xFF
            self.memory[self.registers.SP] = self.registers.PC & 0xFF
            self.registers.write_PC(immediate)

        elif opcode == 0x01: # LD BC, d16
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            print(F"> LD BC, d16")
            print(F"> BC: 0x{self.registers.BC:04X}, d16: 0x{immediate:04X}")
            self.registers.BC = immediate

        elif opcode == 0x0B: # DEC BC
            print(F"> DEC BC")
            self.registers.BC -= 1

        elif opcode == 0x78: # LD A, B
            print(F"> LD A, B")
            self.registers.A = self.registers.B

        elif opcode == 0xB1: # OR C
            print(F"> OR C")
            self.registers.A |= self.registers.C
            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            self.registers.clear_C()
            self.registers.clear_H()
            self.registers.clear_N()

        elif opcode == 0xC9: # RET
            print(f"> RET")
            address = self.memory[self.registers.SP] | (self.memory[self.registers.SP + 1] << 8)
            self.registers.SP += 2
            self.registers.write_PC(address)

        elif opcode == 0xFB: # EI
            print(f"> EI")
            self.IME = 1 # TODO: should be done after the next cycle, not immediately

        elif opcode == 0x2F: # CPL
            print(f"> CPL")
            self.registers.A ^= 0xFF
            self.registers.set_H()
            self.registers.set_N()

        elif opcode == 0xE6: # AND d8
            immediate = extra_bytes[0]
            print(F"> AND d8")
            self.registers.A &= immediate
            self.registers.set_H()
            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            self.registers.clear_C()
            self.registers.clear_N()

        elif opcode == 0xCB37: # SWAP A
            print(F"> SWAP A")
            self.registers.A = ((self.registers.A & 0xF) << 4) | ((self.registers.A >> 4) & 0xF)
            self.registers.set_H()
            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            self.registers.clear_C()
            self.registers.clear_N()

        elif opcode == 0x47: # LD B, A
            print(F"> LD B, A")
            self.registers.B = self.registers.A

        elif opcode == 0xB0: # OR B
            print(F"> OR B")
            self.registers.A |= self.registers.B
            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            self.registers.clear_C()
            self.registers.clear_H()
            self.registers.clear_N()

        elif opcode == 0x4F: # LD C, A
            print(F"> LD C, A")
            self.registers.C = self.registers.A

        elif opcode == 0xA9:  # XOR C
            print("> XOR C")
            self.registers.A ^= self.registers.C

            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()

        elif opcode == 0xA1: # AND C
            print(F"> AND C")
            self.registers.A &= self.registers.C
            self.registers.set_H()
            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            self.registers.set_H()
            self.registers.clear_C()
            self.registers.clear_N()

        elif opcode == 0x79: # LD A, C
            print(F"> LD A, C")
            self.registers.A = self.registers.C

        elif opcode == 0xEF: # RST $28
            print(F"> RST $28")
            self.registers.SP -= 2
            self.memory[self.registers.SP + 1] = (self.registers.PC >> 8) & 0xFF
            self.memory[self.registers.SP] = self.registers.PC & 0xFF
            self.registers.write_PC(0x0028)

        elif opcode == 0x87: # ADD A, A
            print(F"> ADD A, A")
            value_pre = self.registers.A
            upper_nibble_pre = (self.registers.A >> 4) & 0xF
            self.registers.A += self.registers.A
            upper_nibble_post = (self.registers.A >> 4) & 0xF
            if upper_nibble_pre != upper_nibble_post:
                self.registers.set_H()
            else:
                self.registers.clear_H()
            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            if value_pre > self.registers.A:
                self.registers.set_C()
            else:
                self.registers.clear_C()
            self.registers.clear_N()

        elif opcode == 0xE1: # POP HL
            print(F"> POP HL")
            self.registers.HL = (self.memory[self.registers.SP + 1] << 8) | self.memory[self.registers.SP]
            self.registers.SP += 2

        elif opcode == 0x5F: # LD E, A
            print(F"> LD E, A")
            self.registers.E = self.registers.A

        elif opcode == 0x16: # LD D, d8
            immediate = extra_bytes[0]
            print(F"> LD D, d8")
            print(F"> D: 0x{self.registers.D:02X}, d8: 0x{immediate:02X}")
            self.registers.D = immediate

        elif opcode == 0x19: # ADD HL, DE
            print(F"> ADD HL, DE")
            value_pre = self.registers.HL
            # 16 bit addition uses the 8 bit ALU, LSB first then MSB, so the resulting flags apply to the high byte
            upper_nibble_pre = (self.registers.H >> 4) & 0xF
            self.registers.HL += self.registers.DE
            upper_nibble_post = (self.registers.H >> 4) & 0xF
            if upper_nibble_pre != upper_nibble_post:
                self.registers.set_H()
            else:
                self.registers.clear_H()
            if value_pre > self.registers.HL:
                self.registers.set_C()
            else:
                self.registers.clear_C()
            self.registers.clear_N()

        elif opcode == 0x5E: # LD E, (HL)
            print(F"> LD E, (HL)")
            print(F"> E: 0x{self.registers.E:02X}, HL: 0x{self.registers.HL:04X}")
            self.registers.E = self.memory[self.registers.HL]

        elif opcode == 0x23: # INC HL
            print(F"> INC HL")
            self.registers.HL += 1

        elif opcode == 0x56: # LD D, (HL)
            print(F"> LD D, (HL)")
            print(F"> D: 0x{self.registers.D:02X}, HL: 0x{self.registers.HL:04X}")
            self.registers.D = self.memory[self.registers.HL]

        elif opcode == 0xD5: # PUSH DE
            print(F"> PUSH DE")
            self.registers.SP -= 2
            self.memory[self.registers.SP + 1] = self.registers.D
            self.memory[self.registers.SP] = self.registers.E

        elif opcode == 0xE9:  # JP (HL)
            print(F"> JP (HL)")
            print(F"> HL: 0x{self.registers.HL:04X}")
            self.registers.write_PC(self.memory[self.registers.HL])

        elif opcode == 0xF5: # PUSH AF
            print(F"> PUSH AF")
            self.registers.SP -= 2
            self.memory[self.registers.SP + 1] = self.registers.A
            self.memory[self.registers.SP] = self.registers.F

        elif opcode == 0xC5: # PUSH BC
            print(F"> PUSH BC")
            self.registers.SP -= 2
            self.memory[self.registers.SP + 1] = self.registers.B
            self.memory[self.registers.SP] = self.registers.C

        elif opcode == 0xE5: # PUSH HL
            print(F"> PUSH HL")
            self.registers.SP -= 2
            self.memory[self.registers.SP + 1] = self.registers.H
            self.memory[self.registers.SP] = self.registers.L

        elif opcode == 0xFA: # LD A, (a16)
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            print(F"> LD A, (a16)")
            self.registers.A = self.memory[immediate]

        elif opcode == 0x28: # JR Z, r8
            print(F"> JR Z, r8")
            # signed int
            immediate = (extra_bytes[0] & 0x7F) - 128
            if self.registers.read_Z():
                self.registers.write_PC(self.registers.PC + immediate)

        # The instructions CALL, PUSH, and RST all put
        # information onto the stack. The instructions POP, RET,
        # and RETI all take information off of the stack.
        # (Interrupts put a return address on the stack and
        # remove it at their completion as well.)
        else:
            raise NotImplementedError(F"Opcode not implemented: 0x{opcode:02X} ({opcode_dict['mnemonic']})")

    def run(self):
        # self.boot()
        while True:
            print(F"PC: 0x{self.registers.PC:04X}")
            opcode = self.fetch()
            opcode_dict, opcode = self.decode(opcode)
            instr_str = f"{opcode:02X} ({opcode_dict['cycles']}) {opcode_dict['mnemonic']}"
            if "operand1" in opcode_dict:
                instr_str += f" {opcode_dict['operand1']}"
            if "operand2" in opcode_dict:
                instr_str += f", {opcode_dict['operand2']}"
            print(f"\t{instr_str}")
            self.execute(opcode, opcode_dict)


def main():
    file_path = Path("/home/mojonero/Downloads/Tetris (JUE) (V1.1)/Tetris (JUE) (V1.1) [!].gb")
    with open(file_path, "rb") as file:
        ROM = file.read()

    cpu = CPU(ROM)
    cpu.run()
    print()


if __name__ == '__main__':
    main()
