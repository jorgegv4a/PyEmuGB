from typing import Dict, List

from gb_memory import AddressSpace, RegisterBank
from gb_ops import opcodes
from gb_constants import CARTRIDGE_ROM_ONLY
from gb_graphics import LCDController, get_tiles

r8_map = {
    0: 'B',
    1: 'C',
    2: 'D',
    3: 'E',
    4: 'H',
    5: 'L',
    7: 'A',
}

r16_map = {
    0: 'BC',
    1: 'DE',
    2: 'HL',
    3: 'AF',
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
        elif (0x40 <= opcode < 0x80) and opcode != 0x76:
            self._handle_no_param_loads(opcode)

        elif 0x80 <= opcode < 0xC0:
            self._handle_no_param_alu(opcode)

        elif opcode & 0xC6 == 0x04:
            self._handle_inc_dec_r8(opcode)

        elif opcode & 0xC7 == 0xC6:
            self._handle_d8_alu(opcode, extra_bytes)

        elif opcode & 0xC7 == 0x06:
            self._handle_d8_loads(opcode, extra_bytes)

        elif opcode & 0xC7 == 0xC7:
            self._handle_reset_vector(opcode)

        elif opcode & 0xCF == 0xC1:
            self._handle_r16_pop(opcode)

        elif opcode & 0xCF == 0xC5:
            self._handle_r16_push(opcode)

        elif opcode & 0xC7 == 0x03:
            self._handle_inc_dec_r16(opcode)

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

        # ---- LOAD FROM SINGLE REGISTER TO INDIRECT ADDRESS
        elif opcode == 0x32: # LD (HL-), A
            print(F"> LD (HL-), A")
            self.memory[self.registers.HL] = self.registers.A
            self.registers.HL -= 1

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

        # ---- INCREMENT DOUBLE REGISTER
        elif opcode == 0x23:  # INC HL
            print(F"> INC HL")
            self.registers.HL += 1

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

    def _load_to_r8(self, dst_reg: str, value: int):
        setattr(self.registers, dst_reg, value)

    def _read_r8(self, src_reg: str) -> int:
        return getattr(self.registers, src_reg)

    def _load_to_HL(self, src_reg: str):
        print(f"> LD (HL), {src_reg}")
        value = self._read_r8(src_reg)
        self.memory[self.registers.HL] = value

    def _load_from_HL(self, dst_reg: str):
        print(f"> LD {dst_reg}, (HL)")
        value = self.memory[self.registers.HL]
        self._load_to_r8(dst_reg, value)

    def _load_r8_to_r8(self, dst_reg: str, src_reg: str):
        print(f"> LD {dst_reg}, {src_reg}")
        value = self._read_r8(src_reg)
        self._load_to_r8(dst_reg, value)

    def _handle_no_param_loads(self, opcode: int):
        """
        Handles instructions in the big single register load group (0x40 - 0x7F)
        :param opcode:
        :return:
        """
        src_reg_i = opcode & 0x7
        src_reg = r8_map.get(src_reg_i, None)

        dst_reg_i = (opcode >> 3) & 0x7
        dst_reg = r8_map.get(dst_reg_i, None)
        if (opcode >> 6) & 0x03 == 1:
            if src_reg is None:
                self._load_to_HL(dst_reg)
            elif dst_reg is None:
                self._load_from_HL(src_reg)
            else:
                self._load_r8_to_r8(dst_reg, src_reg)
        else:
            raise ValueError(f"Unexpected opcode {opcode}, expected generic load instruction!")

    def _add_uint8(self, operand_byte: int):
        value_pre = self.registers.A
        upper_nibble_pre = (self.registers.A >> 4) & 0xF
        self.registers.A += operand_byte
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

    def _add_with_carry_uint8(self, operand_byte: int):
        value_pre = self.registers.A
        upper_nibble_pre = (self.registers.A >> 4) & 0xF
        self.registers.A += operand_byte + self.registers.read_C()
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

    def _subtract_uint8(self, operand_byte: int):
        value_pre = self.registers.A
        upper_nibble_pre = (self.registers.A >> 4) & 0xF
        self.registers.A -= operand_byte
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

    def _subtract_with_carry_uint8(self, operand_byte: int):
        value_pre = self.registers.A
        upper_nibble_pre = (self.registers.A >> 4) & 0xF
        self.registers.A -= operand_byte + self.registers.read_C()
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

    def _and_uint8(self, operand_byte: int):
        self.registers.A &= operand_byte
        self.registers.set_H()
        if self.registers.A == 0:
            self.registers.set_Z()
        else:
            self.registers.clear_Z()
        self.registers.set_H()
        self.registers.clear_C()
        self.registers.clear_N()

    def _xor_uint8(self, operand_byte: int):
        self.registers.A ^= operand_byte
        if self.registers.A == 0:
            self.registers.set_Z()
        else:
            self.registers.clear_Z()
        self.registers.clear_N()
        self.registers.clear_H()
        self.registers.clear_C()

    def _or_uint8(self, operand_byte: int):
        self.registers.A |= operand_byte
        if self.registers.A == 0:
            self.registers.set_Z()
        else:
            self.registers.clear_Z()
        self.registers.clear_C()
        self.registers.clear_H()
        self.registers.clear_N()

    def _compare_uint8(self, operand_byte: int):
        value_pre = self.registers.A
        upper_nibble_pre = (value_pre >> 4) & 0xF
        value = (self.registers.A + ((operand_byte ^ 0xFF) + 1) & 0xFF) & 0xFF
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

    def _handle_no_param_alu(self, opcode: int):
        """
        Handles instructions in the big single register arithmetic/logic group (0x80 - 0xBF)
        :param opcode:
        :return:
        """
        src_reg_i = opcode & 0x7
        src_reg = r8_map.get(src_reg_i, None)
        if src_reg is None:
            operand_value = self.memory[self.registers.HL]
            operand_repr = "(HL)"
        else:
            operand_value = getattr(self.registers, src_reg)
            operand_repr = src_reg

        if (opcode >> 3) & 0x7 == 0x0:
            print(f"> ADD {operand_repr}")
            self._add_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x1:
            print(f"> ADC {operand_repr}")
            self._add_with_carry_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x2:
            print(f"> SUB {operand_repr}")
            self._subtract_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x3:
            print(f"> SBC {operand_repr}")
            self._subtract_with_carry_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x4:
            print(f"> AND {operand_repr}")
            self._and_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x5:
            print(f"> XOR {operand_repr}")
            self._xor_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x6:
            print(f"> OR {operand_repr}")
            self._or_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x7:
            print(f"> CP {operand_repr}")
            self._compare_uint8(operand_value)
        else:
            raise ValueError(f"Unexpected opcode {opcode}, expected generic ALU instruction!")

    def _handle_inc_dec_r8(self, opcode: int):
        """
        Handles INC/DEC instructions for single register or indirect memory address
        :param opcode:
        :return:
        """
        dst_reg_i = (opcode >> 3) & 0x7
        dst_reg = r8_map.get(dst_reg_i, None)
        if dst_reg is None:
            operand_value = self.memory[self.registers.HL]
            operand_repr = "(HL)"
        else:
            operand_value = getattr(self.registers, dst_reg)
            operand_repr = dst_reg

        if opcode & 1 == 0:
            print(f"> INC {operand_repr}")
            upper_nibble_pre = (operand_value >> 4) & 0xF
            new_value = operand_value + 1
            if dst_reg is None:
                self.memory[self.registers.HL] = new_value
                new_value = self.memory[self.registers.HL]  # set again to guarantee overflow works alright
            else:
                setattr(self.registers, dst_reg, new_value)
                new_value = getattr(self.registers, dst_reg) # set again to guarantee overflow works alright
            upper_nibble_post = (new_value >> 4) & 0xF
            self.registers.clear_N()
            if new_value == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            if upper_nibble_pre != upper_nibble_post:
                self.registers.set_H()
            else:
                self.registers.clear_H()
        elif opcode & 1 == 1:
            print(f"> DEC {operand_repr}")
            upper_nibble_pre = (operand_value >> 4) & 0xF
            new_value = operand_value - 1
            if dst_reg is None:
                self.memory[self.registers.HL] = new_value
                new_value = self.memory[self.registers.HL]  # set again to guarantee overflow works alright
            else:
                setattr(self.registers, dst_reg, new_value)
                new_value = getattr(self.registers, dst_reg)  # set again to guarantee overflow works alright
            upper_nibble_post = (new_value >> 4) & 0xF
            self.registers.set_N()
            if new_value == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            if upper_nibble_pre != upper_nibble_post:
                self.registers.set_H()
            else:
                self.registers.clear_H()
        else:
            raise ValueError(f"Unexpected opcode {opcode}, expected generic ALU instruction!")

    def _handle_d8_alu(self, opcode: int, extra_bytes: List[int]):
        """
        Handles instructions in the special immediate arithmetic/logic groups (0xC6:0xF6:0x10 and 0xCE:0xFE:0x10)
        :param opcode:
        :return:
        """
        operand_value = extra_bytes[0]
        operand_repr = "n"

        if (opcode >> 3) & 0x7 == 0x0:
            print(f"> ADD {operand_repr}")
            self._add_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x1:
            print(f"> ADC {operand_repr}")
            self._add_with_carry_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x2:
            print(f"> SUB {operand_repr}")
            self._subtract_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x3:
            print(f"> SBC {operand_repr}")
            self._subtract_with_carry_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x4:
            print(f"> AND {operand_repr}")
            self._and_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x5:
            print(f"> XOR {operand_repr}")
            self._xor_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x6:
            print(f"> OR {operand_repr}")
            self._or_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x7:
            print(f"> CP {operand_repr}")
            self._compare_uint8(operand_value)
        else:
            raise ValueError(f"Unexpected opcode {opcode}, expected generic ALU instruction!")

    def _handle_d8_loads(self, opcode: int, extra_bytes: List[int]):
        """
        Handles instructions in the special immediate single register load group (0x06 : 0x36 : 0x10 and 0x0A : 0x3A : 0x10)
        :param opcode:
        :return:
        """
        operand_value = extra_bytes[0]

        dst_reg_i = (opcode >> 3) & 0x7
        dst_reg = r8_map.get(dst_reg_i, None)
        if dst_reg is None:
            print(f"> LD (HL), n")
            self.memory[self.registers.HL] = operand_value
        else:
            print(f"> LD {dst_reg}, n")
            self._load_to_r8(dst_reg, operand_value)

    def _push_stack(self, d16_operand: int):
        self.registers.SP -= 1
        self.memory[self.registers.SP] = (d16_operand >> 8) & 0xFF
        # tick(1)

        self.registers.SP -= 1
        self.memory[self.registers.SP] = d16_operand & 0xFF
        # tick(1)

    def _pop_stack(self) -> int:
        # d16_value = (self.memory[self.registers.SP + 1] << 8) | self.memory[self.registers.SP]
        d16_value = self.memory[self.registers.SP]
        self.registers.SP += 1
        # tick(1)

        d16_value |= self.memory[self.registers.SP] << 8
        self.registers.SP += 1
        # tick(1)
        return d16_value

    def _handle_reset_vector(self, opcode: int):
        """
        Handles RST instructions
        :param opcode:
        :return:
        """
        # 0b11xxx111
        dst_address = (opcode >> 3) * 8
        print(f"> RST 0x{dst_address:02X}")
        self._push_stack(self.registers.PC)
        self.registers.write_PC(dst_address)
        # tick(1)

    def _handle_r16_pop(self, opcode: int):
        """
        Handles double register POP instructions
        :param opcode:
        :return:
        """
        dst_reg_i = (opcode >> 4) & 0x3
        dst_reg = r16_map[dst_reg_i]
        print(f"> POP {dst_reg}")

        d16_value = self._pop_stack()
        setattr(self.registers, dst_reg, d16_value)

    def _handle_r16_push(self, opcode: int):
        """
        Handles double register PUSH instructions
        :param opcode:
        :return:
        """
        src_reg_i = (opcode >> 4) & 0x3
        src_reg = r16_map[src_reg_i]
        print(f"> PUSH {src_reg}")

        d16_operand = getattr(self.registers, src_reg)
        self._push_stack(d16_operand)

    def _handle_inc_dec_r16(self, opcode: int):
        """
        Handles INC/DEC instructions for double register or indirect memory address
        :param opcode:
        :return:
        """
        dst_reg_i = (opcode >> 4) & 0x5
        dst_reg = r16_map[dst_reg_i]
        operand_value = getattr(self.registers, dst_reg)
        operand_repr = dst_reg

        if (opcode >> 3) & 1 == 0:
            print(f"> INC {operand_repr}")
            new_value = operand_value + 1
        elif (opcode >> 3) & 1 == 1:
            print(f"> DEC {operand_repr}")
            new_value = operand_value - 1
        else:
            raise ValueError(f"Unexpected opcode {opcode}, expected generic ALU instruction!")

        setattr(self.registers, dst_reg, new_value)

    def __str__(self):
        return f'{self.registers} | IME: {self.IME} | T: {self.clock}'

    def __repr__(self):
        return self.__str__()