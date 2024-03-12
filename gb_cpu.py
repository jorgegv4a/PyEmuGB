from typing import Dict

from gb_memory import AddressSpace, RegisterBank
from gb_ops import opcodes
from gb_constants import CARTRIDGE_ROM_ONLY
from gb_graphics import LCDController, get_tiles

SR_map = {
    0: 'B',
    1: 'C',
    2: 'D',
    3: 'E',
    4: 'H',
    5: 'L',
    7: 'A',
}


class CPU:
    def __init__(self, game_rom: bytes):
        self.registers = RegisterBank()
        self.memory = AddressSpace()
        self.memory.load_rom(game_rom, CARTRIDGE_ROM_ONLY)
        self.ppu = LCDController(self.memory)
        self.IME = 0 # Interrupt Master Enable
        self.clock = 0 # real clock, Machine cycles take 4 cycles, PPU dots take 1 cycle

        get_tiles(self.memory)

    def tick(self, n):
        """
        Clock is increased by 4 after
            - Instruction Fetch
            - Data Byte Fetch
            - Instruction CB Fetch

        :param n:
        :return:
        """
        self.clock += n
        for i in range(n):
            self.ppu.tick()

    def fetch(self):
        opcode = self.memory[self.registers.PC]
        self.registers.increment_pc()
        self.tick(4)
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
        start_clock_t = self.clock - 4 # discount 1 M-cycle to fetch first instr byte
        code_length = opcode_dict["length"] - 1
        if ((opcode >> 8) & 0xFF) == 0xCB:
            code_length -= 1

        extra_bytes = []
        for i in range(code_length):
            extra_bytes.append(self.fetch()) # almost always LS byte first

        # how many cycles we haven't ticked over yet
        remaining_cycles = opcode_dict["cycles"][0] - (self.clock - start_clock_t)

        if opcode == 0x00: # NOP
            print("> NOP")

        elif opcode == 0x10: # STOP
            print("> STOP")
            # TODO: stop clock, disable LCD, wait until joypad interrupt
        # -- LOADS
        # ---- LOAD FROM SINGLE REGISTER TO SINGLE REGISTER
        if (0x40 <= opcode < 0x80) and opcode != 0x76:
            self._handle_no_param_loads(opcode)

        # ---- LOAD FROM DOUBLE IMMEDIATE TO DOUBLE REGISTER
        elif opcode == 0x01: # LD BC, d16
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            print(F"> LD BC, d16")
            self.registers.BC = immediate

        elif opcode == 0x21: # LD HL, d16
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            print(F"> LD HL, 0x{immediate:04X}")
            self.registers.HL = immediate

        elif opcode == 0x31: # LD SP, d16
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            print(F"> LD SP, d16")
            self.registers.SP = immediate

        # ---- LOAD FROM SINGLE IMMEDIATE TO SINGLE REGISTER
        elif opcode == 0x3E:  # LD A, d8
            immediate = extra_bytes[0]
            print(F"> LD A, 0x{immediate:02X}")
            self.registers.A = immediate

        elif opcode == 0x06: # LD B, d8
            immediate = extra_bytes[0]
            print(F"> LD B, 0x{immediate:02X}")
            self.registers.B = immediate

        elif opcode == 0x0E: # LD C, d8
            immediate = extra_bytes[0]
            print(F"> LD C, 0x{immediate:02X}")
            self.registers.C = immediate

        elif opcode == 0x16:  # LD D, d8
            immediate = extra_bytes[0]
            print(F"> LD D, d8")
            self.registers.D = immediate

        # ---- LOAD FROM SINGLE REGISTER TO INDIRECT ADDRESS
        elif opcode == 0x32: # LD (HL-), A
            print(F"> LD (HL-), A")
            self.memory[self.registers.HL] = self.registers.A
            self.registers.HL -= 1

        # ---- LOAD FROM SINGLE IMMEDIATE TO INDIRECT ADDRESS
        elif opcode == 0x36: # LD (HL), d8
            immediate = extra_bytes[0]
            print(F"> LD (HL), d8")
            self.memory[self.registers.HL] = immediate

        # ---- LOAD FROM SINGLE REGISTER TO DOUBLE IMMEDIATE INDIRECT ADDRESS
        elif opcode == 0xEA: # LD (a16), A
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            print(F"> LD (a16), A")
            self.memory[immediate] = self.registers.A

        # ---- LOAD FROM DOUBLE REGISTER TO DOUBLE IMMEDIATE INDIRECT ADDRESS
        elif opcode == 0x08: # LD (a16), SP
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            print(F"> LD (a16), SP")
            self.memory[immediate] = self.registers.SP & 0xFF
            self.memory[immediate + 1] = (self.registers.SP >> 8) & 0xFF

        # ---- LOAD FROM SINGLE REGISTER TO REGISTER INDIRECT ADDRESS
        elif opcode == 0xE2: # LD (C), A
            print(F"> LD (C), A")
            print(F"> C (0x{self.registers.C:02X}), A: {self.registers.A}")
            self.memory[0xFF00 | self.registers.C] = self.registers.A

        # ---- LOAD FROM DOUBLE REGISTER INDIRECT ADDRESS TO SINGLE REGISTER
        elif opcode == 0x0A: # LD A, (BC)
            print(F"> LD A, (BC)")
            self.registers.A = self.memory[self.registers.BC]

        elif opcode == 0x1A: # LD A, (DE)
            print(F"> LD A, (DE)")
            self.registers.A = self.memory[self.registers.DE]

        elif opcode == 0x2A: # LD A, (HL+)
            print(F"> LD A, (HL+)")
            self.registers.A = self.memory[self.registers.HL]
            self.registers.HL += 1

        # ---- LOAD FROM DOUBLE IMMEDIATE INDIRECT ADDRESS TO SINGLE REGISTER
        elif opcode == 0xFA: # LD A, (a16)
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            print(F"> LD A, (a16)")
            self.registers.A = self.memory[immediate]

        # ---- LOAD FROM SINGLE REGISTER TO INDIRECT HIGH ADDRESS
        elif opcode == 0xE0: # LDH (d8), A
            immediate = 0xFF00 | extra_bytes[0]
            print(F"> LDH (0x{immediate:04X}), A")
            self.memory[immediate] = self.registers.A

        # ---- LOAD FROM INDIRECT HIGH ADDRESS TO SINGLE REGISTER
        elif opcode == 0xF0: # LDH A, (d8)
            immediate = 0xFF00 | extra_bytes[0]
            print(F"> LDH (0x{immediate:04X}), A")
            self.registers.A = self.memory[immediate]

        # -- ARITHMETIC AND LOGIC
        # ---- A AND SINGLE REGISTER
        elif opcode == 0xA7:  # AND A
            print(F"> AND A")
            self.registers.A &= self.registers.A
            self.registers.set_H()
            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            self.registers.set_H()
            self.registers.clear_C()
            self.registers.clear_N()

        elif opcode == 0xA1:  # AND C
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

        elif opcode == 0xA5:  # AND L
            print(F"> AND L")
            self.registers.A &= self.registers.L
            self.registers.set_H()
            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            self.registers.set_H()
            self.registers.clear_C()
            self.registers.clear_N()

        # ---- A AND SINGLE IMMEDIATE
        elif opcode == 0xE6:  # AND d8
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

        # ---- A OR SINGLE IMMEDIATE
        elif opcode == 0xB0:  # OR B
            print(F"> OR B")
            self.registers.A |= self.registers.B
            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            self.registers.clear_C()
            self.registers.clear_H()
            self.registers.clear_N()

        elif opcode == 0xB1:  # OR C
            print(F"> OR C")
            self.registers.A |= self.registers.C
            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            self.registers.clear_C()
            self.registers.clear_H()
            self.registers.clear_N()

        # ---- A XOR SINGLE REGISTER
        elif opcode == 0xAF:  # XOR A
            print("> XOR A")
            self.registers.A ^= self.registers.A

            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            self.registers.clear_N()
            self.registers.clear_H()
            self.registers.clear_C()

        elif opcode == 0xA9:  # XOR C
            print("> XOR C")
            self.registers.A ^= self.registers.C

            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            self.registers.clear_N()
            self.registers.clear_H()
            self.registers.clear_C()

        # ---- ADD SINGLE REGISTER TO SINGLE REGISTER
        elif opcode == 0x87:  # ADD A, A
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

        elif opcode == 0x80:  # ADD A, B
            print(F"> ADD A, B")
            value_pre = self.registers.A
            upper_nibble_pre = (self.registers.A >> 4) & 0xF
            self.registers.A += self.registers.B
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

        # ---- ADD DOUBLE REGISTER TO DOUBLE REGISTER
        elif opcode == 0x19:  # ADD HL, DE
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

        # SUBSTRACT WITH CARRY FROM SINGLE REGISTER
        elif opcode == 0x9D:  # SBC A, L
            print(F"> SBC A, L")
            value_pre = self.registers.A
            upper_nibble_pre = (self.registers.A >> 4) & 0xF
            self.registers.A -= self.registers.L + self.registers.read_C()
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

        # ---- INCREMENT SINGLE REGISTER
        elif opcode == 0x04:  # INC B
            print(F"> INC B")
            upper_nibble_pre = (self.registers.B >> 4) & 0xF
            self.registers.B += 1
            upper_nibble_post = (self.registers.B >> 4) & 0xF
            self.registers.clear_N()
            if self.registers.B == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            if upper_nibble_pre != upper_nibble_post:
                self.registers.set_H()
            else:
                self.registers.clear_H()

        elif opcode == 0x0C:  # INC C
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

        # ---- INCREMENT DOUBLE REGISTER
        elif opcode == 0x23:  # INC HL
            print(F"> INC HL")
            self.registers.HL += 1

        # ---- DECREMENT SINGLE REGISTER
        elif opcode == 0x3D: # DEC A
            print(F"> DEC A")
            upper_nibble_pre = (self.registers.A >> 4) & 0xF
            self.registers.A -= 1
            upper_nibble_post = (self.registers.A >> 4) & 0xF
            self.registers.set_N()
            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            if upper_nibble_pre != upper_nibble_post:
                self.registers.set_H()
            else:
                self.registers.clear_H()

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

        # ---- DECREMENT DOUBLE REGISTER
        elif opcode == 0x0B: # DEC BC
            print(F"> DEC BC")
            self.registers.BC -= 1

        # ---- COMPLEMENT A
        elif opcode == 0x2F: # CPL
            print(f"> CPL")
            self.registers.A ^= 0xFF
            self.registers.set_H()
            self.registers.set_N()

        # ---- COMPARE SINGLE IMMEDIATE
        elif opcode == 0xFE: # CP d8
            immediate = extra_bytes[0]
            print(F"> CP (0x{immediate:02X}), A")
            value_pre = self.registers.A
            upper_nibble_pre = (value_pre >> 4) & 0xF
            value = (self.registers.A + ((immediate ^ 0xFF) + 1) & 0xFF) & 0xFF
            upper_nibble_post = (value >> 4) & 0xF
            if upper_nibble_pre != upper_nibble_post:
                self.registers.set_H()
            else:
                self.registers.clear_H()
            if value == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            if value_pre < value:
                self.registers.set_C()
            else:
                self.registers.clear_C()
            self.registers.set_N()

        # -- ROTATES AND SHIFTS
        # ---- SWAP SINGLE REGISTER
        elif opcode == 0xCB37:  # SWAP A
            print(F"> SWAP A")
            self.registers.A = ((self.registers.A & 0xF) << 4) | ((self.registers.A >> 4) & 0xF)
            self.registers.set_H()
            if self.registers.A == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            self.registers.clear_C()
            self.registers.clear_N()

        # ---- ROTATE RIGHT CIRCULAR A
        elif opcode == 0x0F:  # RRCA
            print(F"> RRCA")
            bottom_bit = self.registers.A & 1
            self.registers.A = (self.registers.A >> 1) | (bottom_bit << 7)
            if bottom_bit:
                self.registers.set_C()
            else:
                self.registers.clear_C()
            self.registers.clear_Z()
            self.registers.clear_N()
            self.registers.clear_H()

        # ---- ROTATE LEFT CIRCULAR A
        elif opcode == 0x07:  # RLCA
            print(F"> RLCA")
            top_bit = (self.registers.A >> 7) & 1
            self.registers.A = ((self.registers.A & 0x7F) << 1) | top_bit
            if top_bit:
                self.registers.set_C()
            else:
                self.registers.clear_C()
            self.registers.clear_Z()
            self.registers.clear_N()
            self.registers.clear_H()

        # ---- ROTATE RIGHT CIRCULAR A
        elif opcode == 0x1F:  # RRA
            print(F"> RRA")
            bottom_bit = self.registers.A & 1
            self.registers.A = (self.registers.A >> 1) | (self.registers.read_C() << 7)
            if bottom_bit:
                self.registers.set_C()
            else:
                self.registers.clear_C()
            self.registers.clear_Z()
            self.registers.clear_N()
            self.registers.clear_H()

        # -- CONTROL FLOW
        # ---- ABSOLUTE JUMPS
        elif opcode == 0xC3:  # JP a16
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            print(F"> JP 0x{immediate:04X}")
            self.registers.write_PC(immediate)

        elif opcode == 0xE9:  # JP (HL)
            print(F"> JP (HL)")
            self.registers.write_PC(self.memory[self.registers.HL])

        # ---- RELATIVE JUMP
        elif opcode == 0x18:  # JR e
            print(F"> JR e")
            # signed int
            immediate = (extra_bytes[0] & 0x7F) - 128
            self.registers.write_PC(self.registers.PC + immediate)

        elif opcode == 0x20:  # JR NZ, r8
            print(F"> JR NZ, r8")
            # signed int
            immediate = (extra_bytes[0] & 0x7F) - 128
            if not self.registers.read_Z():
                self.registers.write_PC(self.registers.PC + immediate)
            else:
                remaining_cycles = opcode_dict["cycles"][1] - (self.clock - start_clock_t)

        elif opcode == 0x28:  # JR Z, r8
            print(F"> JR Z, r8")
            # signed int
            immediate = (extra_bytes[0] & 0x7F) - 128
            if self.registers.read_Z():
                self.registers.write_PC(self.registers.PC + immediate)
            else:
                remaining_cycles = opcode_dict["cycles"][1] - (self.clock - start_clock_t)

        # ---- RESET VECTOR $28
        elif opcode == 0xEF:  # RST $28
            print(F"> RST $28")
            self.registers.SP -= 2
            self.memory[self.registers.SP + 1] = (self.registers.PC >> 8) & 0xFF
            self.memory[self.registers.SP] = self.registers.PC & 0xFF
            self.registers.write_PC(0x0028)

        # ---- CALL DOUBLE INMMEDIATE
        elif opcode == 0xCD:  # CALL a16
            print(F"> CALL a16")
            immediate = (extra_bytes[1] << 8) | extra_bytes[0]
            self.registers.SP -= 2
            self.memory[self.registers.SP + 1] = (self.registers.PC >> 8) & 0xFF
            self.memory[self.registers.SP] = self.registers.PC & 0xFF
            self.registers.write_PC(immediate)

        # ---- RETURN
        elif opcode == 0xC9:  # RET
            print(f"> RET")
            address = self.memory[self.registers.SP] | (self.memory[self.registers.SP + 1] << 8)
            self.registers.SP += 2
            self.registers.write_PC(address)

        # -- INTERRUPT CONTROL
        # ---- DISABLE INTERRUPTS
        elif opcode == 0xF3: # DI
            print(F"> DI")
            self.IME = 0

        # ---- ENABLE INTERRUPTS
        elif opcode == 0xFB: # EI
            print(f"> EI")
            self.IME = 1 # TODO: should be done after the next cycle, not immediately

        # ---- STACK PUSH/POP
        elif opcode == 0xE1: # POP HL
            print(F"> POP HL")
            self.registers.HL = (self.memory[self.registers.SP + 1] << 8) | self.memory[self.registers.SP]
            self.registers.SP += 2

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

        elif opcode == 0xD5: # PUSH DE
            print(F"> PUSH DE")
            self.registers.SP -= 2
            self.memory[self.registers.SP + 1] = self.registers.D
            self.memory[self.registers.SP] = self.registers.E

        elif opcode == 0xE5: # PUSH HL
            print(F"> PUSH HL")
            self.registers.SP -= 2
            self.memory[self.registers.SP + 1] = self.registers.H
            self.memory[self.registers.SP] = self.registers.L

        # The instructions CALL, PUSH, and RST all put
        # information onto the stack. The instructions POP, RET,
        # and RETI all take information off of the stack.
        # (Interrupts put a return address on the stack and
        # remove it at their completion as well.)
        else:
            raise NotImplementedError(F"Opcode not implemented: 0x{opcode:02X} ({opcode_dict['mnemonic']})")

        # Calls or Jumps take extra cycles
        self.tick(remaining_cycles)

    def boot(self):
        # TODO: implement power up sequence checks + graphics
        self.registers.AF = 0x01B0
        self.registers.BC = 0x0013
        self.registers.DE = 0x00D8
        self.registers.HL = 0x014D
        self.registers.SP = 0xFFFE
        self.registers.write_PC(0x0100)

        self.memory[0xFF05] = 0x00  # TIMA
        self.memory[0xFF06] = 0x00  # TMA
        self.memory[0xFF07] = 0x00  # TAC
        self.memory[0xFF10] = 0x80  # NR10
        self.memory[0xFF11] = 0xBF  # NR11
        self.memory[0xFF12] = 0xF3  # NR12
        self.memory[0xFF14] = 0xBF  # NR14
        self.memory[0xFF16] = 0x3F  # NR21
        self.memory[0xFF17] = 0x00  # NR22
        self.memory[0xFF19] = 0xBF  # NR24
        self.memory[0xFF1A] = 0x7F  # NR30
        self.memory[0xFF1B] = 0xFF  # NR31
        self.memory[0xFF1C] = 0x9F  # NR32
        self.memory[0xFF1E] = 0xBF  # NR33
        self.memory[0xFF20] = 0xFF  # NR41
        self.memory[0xFF21] = 0x00  # NR42
        self.memory[0xFF22] = 0x00  # NR43
        self.memory[0xFF23] = 0xBF  # NR30
        self.memory[0xFF24] = 0x77  # NR50
        self.memory[0xFF25] = 0xF3  # NR51
        self.memory[0xFF26] = 0xF1  # NR52, GB, 0xF0-SGB
        self.memory[0xFF40] = 0x91  # LCDC
        self.memory[0xFF42] = 0x00  # SCY
        self.memory[0xFF43] = 0x00  # SCX
        self.memory[0xFF45] = 0x00  # LYC
        self.memory[0xFF47] = 0xFC  # BGP
        self.memory[0xFF48] = 0xFF  # OBP0
        self.memory[0xFF49] = 0xFF  # OBP1
        self.memory[0xFF4A] = 0x00  # WY
        self.memory[0xFF4B] = 0x00  # WX
        self.memory[0xFFFF] = 0x00  # IE

    def run(self):
        self.boot()
        while True:
            # print(F"PC: 0x{self.registers.PC:04X}")
            print(F"{self}")
            opcode = self.fetch()
            opcode_dict, opcode = self.decode(opcode)
            instr_str = f"{opcode:02X} ({opcode_dict['cycles']}) {opcode_dict['mnemonic']}"
            if "operand1" in opcode_dict:
                instr_str += f" {opcode_dict['operand1']}"
            if "operand2" in opcode_dict:
                instr_str += f", {opcode_dict['operand2']}"
            print(f"\t{instr_str}")
            self.execute(opcode, opcode_dict)

    def _load_from_HL(self, src_reg: str):
        print(f"> LD (HL), {src_reg}")
        value = getattr(self.registers, src_reg)
        self.memory[self.registers.HL] = value

    def _load_to_HL(self, dst_reg: str):
        print(f"> LD {dst_reg}, (HL)")
        # self.registers.D = self.memory[self.registers.HL]
        value = self.memory[self.registers.HL]
        setattr(self.registers, dst_reg, value)

    def _load_r8_to_r8(self, dst_reg: str, src_reg: str):
        print(f"> LD {dst_reg}, {src_reg}")
        value = getattr(self.registers, src_reg)
        setattr(self.registers, dst_reg, value)

    def _handle_no_param_loads(self, opcode: int):
        """
        Handles instructions in the big single register load group (0x40 - 0x7F)
        :param opcode:
        :return:
        """
        src_reg_i = opcode & 0x7
        src_reg = SR_map.get(src_reg_i, None)

        dst_reg_i = (opcode >> 3) & 0x7
        dst_reg = SR_map.get(dst_reg_i, None)
        if (opcode >> 6) & 0x03 == 1:
            if src_reg is None:
                self._load_to_HL(dst_reg)
            elif dst_reg is None:
                self._load_from_HL(src_reg)
            else:
                self._load_r8_to_r8(dst_reg, src_reg)
        else:
            raise ValueError(f"Unexpected opcode {opcode}, expected generic load instruction!")

    def _add_r8(self, src_reg: str):
        print(f"> ADD {src_reg}")
        value_pre = self.registers.A
        operand_value = getattr(self.registers, src_reg)
        upper_nibble_pre = (self.registers.A >> 4) & 0xF
        self.registers.A += operand_value
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

    def _add_HL(self):
        print(f"> ADD (HL)")
        value_pre = self.registers.A
        operand_value = self.memory[self.registers.HL]
        upper_nibble_pre = (self.registers.A >> 4) & 0xF
        self.registers.A += operand_value
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

    def _subtract_with_carry_r8(self, src_reg: str):
        print(F"> SBC {src_reg}")
        value_pre = self.registers.A
        operand_value = getattr(self.registers, src_reg)
        upper_nibble_pre = (self.registers.A >> 4) & 0xF
        self.registers.A -= operand_value + self.registers.read_C()
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

    def _subtract_with_carry_HL(self):
        print(F"> SBC (HL)")
        value_pre = self.registers.A
        operand_value = self.memory[self.registers.HL]
        upper_nibble_pre = (self.registers.A >> 4) & 0xF
        self.registers.A -= operand_value + self.registers.read_C()
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

    def _handle_no_param_alu(self, opcode: int):
        """
        Handles instructions in the big single register arithmetic/logic group (0x80 - 0xBF)
        :param opcode:
        :return:
        """
        src_reg_i = opcode & 0x7
        src_reg = SR_map.get(src_reg_i, None)

        if (opcode >> 3) == 0x10:
            if src_reg is None:
                self._add_HL()
            else:
                self._add_r8(src_reg)
        elif (opcode >> 3) == 0x11:
            text += txt(f"% y ADC {src_reg}")
        elif (opcode >> 3) == 0x12:
            text += txt(f"% y SUB {src_reg}")
        elif (opcode >> 3) == 0x13:
            if src_reg is None:
                self._subtract_with_carry_HL()
            else:
                self._subtract_with_carry_r8(src_reg)
        elif (opcode >> 3) == 0x14:
            text += txt(f"% y AND {src_reg}")
        elif (opcode >> 3) == 0x15:
            text += txt(f"% y XOR {src_reg}")
        elif (opcode >> 3) == 0x16:
            text += txt(f"% y OR {src_reg}")
        elif (opcode >> 3) == 0x17:
            text += txt(f"% y CP {src_reg}")
        else:
            raise ValueError(f"Unexpected opcode {opcode}, expected generic ALU instruction!")

    def __str__(self):
        return f'{self.registers} | IME: {self.IME} | T: {self.clock}'

    def __repr__(self):
        return self.__str__()