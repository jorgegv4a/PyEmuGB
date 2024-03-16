from typing import Dict, List

from gb_memory import AddressSpace, RegisterBank
from gb_ops import opcodes
from gb_constants import CARTRIDGE_ROM_ONLY
from gb_graphics import LCDController, get_tiles

# DEBUG = True
DEBUG = False

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
    3: 'SP',
}


def byte_to_int8(value: int) -> int:
    value = value & 0xFF
    negative = (value >> 7) & 1
    if negative:
        value -= 256
    return value


def bytes_to_uint16(value: List[int]) -> int:
    # LSB first
    return ((value[1] & 0xFF) << 8) | (value[0] & 0xFF)


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

        if opcode < 0xFF: # unprefixed
            if opcode == 0x00: # NOP
                if DEBUG:
                    print("> NOP")
                pass
            elif opcode == 0x10: # STOP
                if DEBUG:
                    print("> STOP")
                pass
                # TODO: stop clock, disable LCD, wait until joypad interrupt

            # LOADS
            elif (0x40 <= opcode < 0x80) and opcode != 0x76:
                self._handle_no_param_loads(opcode)

            elif opcode & 0xC7 == 0x06:
                self._handle_d8_loads(opcode, extra_bytes)

            elif opcode & 0xCF == 0x01:
                self._handle_load_d16_to_r16(opcode, extra_bytes)

            elif opcode & 0xC7 == 0x02:
                self._handle_indirect_loads(opcode)

            elif opcode & 0xFE == 0xF8:
                self._handle_load_r16_to_r16(opcode, extra_bytes)

            elif opcode & 0xE5 == 0xE0 and opcode & 0xEF != 0xE8:
                self._handle_misc_indirect_loads(opcode, extra_bytes)
            # -----------------

            # JUMPS
            elif opcode & 0xE7 == 0xC2:
                branch = self._handle_jump_d16_cond(opcode, extra_bytes)
                if not branch:
                    remaining_cycles = opcode_dict["cycles"][1] - (self.clock - start_clock_t)

            elif opcode == 0xC3:
                self._handle_jump_absolute_d16(opcode, extra_bytes)

            elif opcode == 0xE9:
                self._handle_jump_absolute_HL(opcode, extra_bytes)

            elif opcode & 0xE7 == 0x20:
                branch = self._handle_jump_relative_cond(opcode, extra_bytes)
                if not branch:
                    remaining_cycles = opcode_dict["cycles"][1] - (self.clock - start_clock_t)

            elif opcode == 0x18:
                self._handle_jump_relative(opcode, extra_bytes)
            # -----------------

            # ARITHMETIC/LOGIC
            elif 0x80 <= opcode < 0xC0:
                self._handle_no_param_alu(opcode)

            elif opcode & 0xC7 == 0xC6:
                self._handle_d8_alu(opcode, extra_bytes)

            elif opcode & 0xE7 == 0x27:
                self._handle_accumulator_misc(opcode)

            elif opcode & 0xC6 == 0x04:
                self._handle_inc_dec_r8(opcode)

            elif opcode & 0xC7 == 0x03:
                self._handle_inc_dec_r16(opcode)

            elif opcode & 0xCF == 0x09:
                self._handle_add_r16(opcode)

            elif opcode == 0xE8:
                self._handle_add_SP_int8(opcode, extra_bytes)

            elif opcode & 0xCF == 0xC1:
                self._handle_r16_pop(opcode)

            elif opcode & 0xCF == 0xC5:
                self._handle_r16_push(opcode)

            elif opcode & 0xE7 == 0x07:
                self._handle_rotate_accumulator(opcode)
            # -----------------

            # CALL/RESET/RETURN
            elif opcode & 0xE7 == 0xC4:
                self._handle_call_cond(opcode, extra_bytes)

            elif opcode == 0xCD:
                self._handle_call_d16(opcode, extra_bytes)

            elif opcode & 0xC7 == 0xC7:
                self._handle_reset_vector(opcode)

            elif opcode & 0xEF == 0xC9:
                self._handle_return(opcode)

            elif opcode & 0xE7 == 0xC0:
                branch = self._handle_return_cond(opcode)
                if not branch:
                    remaining_cycles = opcode_dict["cycles"][1] - (self.clock - start_clock_t)
            # -----------------

            # -- INTERRUPT CONTROL
            elif opcode == 0xF3: # DI
                if DEBUG:
                    print(F"> DI")
                self.IME = 0

            elif opcode == 0xFB: # EI
                if DEBUG:
                    print(f"> EI")
                self.IME = 1 # TODO: should be done after the next cycle, not immediately
            # -----------------
            else:
                raise NotImplementedError(F"Unprefixed opcode not implemented: 0x{opcode:02X} ({opcode_dict['mnemonic']})")

        elif (opcode >> 8) == 0xCB:  # prefixed
            raise NotImplementedError(F"Prefixed opcode not implemented: 0x{opcode:02X} ({opcode_dict['mnemonic']})")
        else:
            raise NotImplementedError(F"Opcode not implemented: 0x{opcode:02X} ({opcode_dict['mnemonic']})")

        # Calls or Jumps take extra cycles
        self.tick(remaining_cycles)
        if self.memory[0xFF02] == 0x81:
            print(f"{self.memory[0xFF01]}", end="")
            self.memory[0xFF02] = 0

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
            if DEBUG:
                print(F"{self}")
            opcode = self.fetch()
            opcode_dict, opcode = self.decode(opcode)
            instr_str = f"{opcode:02X} ({opcode_dict['cycles']}) {opcode_dict['mnemonic']}"
            if "operand1" in opcode_dict:
                instr_str += f" {opcode_dict['operand1']}"
            if "operand2" in opcode_dict:
                instr_str += f", {opcode_dict['operand2']}"
            if DEBUG:
                print(f"\t{instr_str}")
            self.execute(opcode, opcode_dict)

    def _load_to_r8(self, dst_reg: str, value: int):
        setattr(self.registers, dst_reg, value)

    def _read_r8(self, src_reg: str) -> int:
        return getattr(self.registers, src_reg)

    def _load_to_HL(self, src_reg: str):
        if DEBUG:
            print(f"> LD (HL), {src_reg}")
        value = self._read_r8(src_reg)
        self.memory[self.registers.HL] = value

    def _load_from_HL(self, dst_reg: str):
        if DEBUG:
            print(f"> LD {dst_reg}, (HL)")
        value = self.memory[self.registers.HL]
        self._load_to_r8(dst_reg, value)

    def _load_r8_to_r8(self, dst_reg: str, src_reg: str):
        if DEBUG:
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
                self._load_from_HL(dst_reg)
            elif dst_reg is None:
                self._load_to_HL(src_reg)
            else:
                self._load_r8_to_r8(dst_reg, src_reg)
        else:
            raise ValueError(f"Unexpected opcode {opcode}, expected generic load instruction!")

    def _add_uint8(self, operand_byte: int):
        value_pre = self.registers.A
        self.registers.A += operand_byte
        if (value_pre & 0xF) + (operand_byte & 0xF) > 0xF:
            self.registers.set_H()
        else:
            self.registers.clear_H()
        if self.registers.A == 0:
            self.registers.set_Z()
        else:
            self.registers.clear_Z()
        if (value_pre + operand_byte) > 0xFF:
            self.registers.set_C()
        else:
            self.registers.clear_C()
        self.registers.clear_N()

    def _add_with_carry_uint8(self, operand_byte: int):
        value_pre = self.registers.A
        self.registers.A += operand_byte + self.registers.read_C()
        if (value_pre & 0xF) + (operand_byte & 0xF) + self.registers.read_C() > 0xF:
            self.registers.set_H()
        else:
            self.registers.clear_H()
        if self.registers.A == 0:
            self.registers.set_Z()
        else:
            self.registers.clear_Z()
        if (value_pre + operand_byte + self.registers.read_C()) > 0xFF:
            self.registers.set_C()
        else:
            self.registers.clear_C()
        self.registers.clear_N()

    def _subtract_uint8(self, operand_byte: int):
        value_pre = self.registers.A
        self.registers.A -= operand_byte
        if (value_pre & 0xF) - (operand_byte & 0xF) < 0:
            self.registers.set_H()
        else:
            self.registers.clear_H()
        if self.registers.A == 0:
            self.registers.set_Z()
        else:
            self.registers.clear_Z()
        if value_pre - operand_byte < 0:
            self.registers.set_C()
        else:
            self.registers.clear_C()
        self.registers.set_N()

    def _subtract_with_carry_uint8(self, operand_byte: int):
        value_pre = self.registers.A
        self.registers.A -= operand_byte + self.registers.read_C()
        if (value_pre & 0xF) - (operand_byte & 0xF) - self.registers.read_C() < 0:
            self.registers.set_H()
        else:
            self.registers.clear_H()
        if self.registers.A == 0:
            self.registers.set_Z()
        else:
            self.registers.clear_Z()
        if value_pre - operand_byte - self.registers.read_C() < 0:
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
        value = (self.registers.A + ((operand_byte ^ 0xFF) + 1) & 0xFF) & 0xFF
        if (value_pre & 0xF) - (operand_byte & 0xF) < 0:
            self.registers.set_H()
        else:
            self.registers.clear_H()
        if value == 0:
            self.registers.set_Z()
        else:
            self.registers.clear_Z()
        if value_pre - operand_byte < 0:
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
            if DEBUG:
                print(f"> ADD {operand_repr}")
            self._add_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x1:
            if DEBUG:
                print(f"> ADC {operand_repr}")
            self._add_with_carry_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x2:
            if DEBUG:
                print(f"> SUB {operand_repr}")
            self._subtract_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x3:
            if DEBUG:
                print(f"> SBC {operand_repr}")
            self._subtract_with_carry_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x4:
            if DEBUG:
                print(f"> AND {operand_repr}")
            self._and_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x5:
            if DEBUG:
                print(f"> XOR {operand_repr}")
            self._xor_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x6:
            if DEBUG:
                print(f"> OR {operand_repr}")
            self._or_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x7:
            if DEBUG:
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
            if DEBUG:
                print(f"> INC {operand_repr}")
            new_value = operand_value + 1
            if dst_reg is None:
                self.memory[self.registers.HL] = new_value
                new_value = self.memory[self.registers.HL]  # set again to guarantee overflow works alright
            else:
                setattr(self.registers, dst_reg, new_value)
                new_value = getattr(self.registers, dst_reg) # set again to guarantee overflow works alright
            self.registers.clear_N()
            if new_value == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            if (operand_value & 0xF) + 1 > 0xF:
                self.registers.set_H()
            else:
                self.registers.clear_H()
        elif opcode & 1 == 1:
            if DEBUG:
                print(f"> DEC {operand_repr}")
            new_value = operand_value - 1
            if dst_reg is None:
                self.memory[self.registers.HL] = new_value
                new_value = self.memory[self.registers.HL]  # set again to guarantee overflow works alright
            else:
                setattr(self.registers, dst_reg, new_value)
                new_value = getattr(self.registers, dst_reg)  # set again to guarantee overflow works alright
            self.registers.set_N()
            if new_value == 0:
                self.registers.set_Z()
            else:
                self.registers.clear_Z()
            if (operand_value & 0xF) - 1 < 0:
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
            if DEBUG:
                print(f"> ADD {operand_repr} ({operand_value:02X})")
            self._add_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x1:
            if DEBUG:
                print(f"> ADC {operand_repr} ({operand_value:02X})")
            self._add_with_carry_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x2:
            if DEBUG:
                print(f"> SUB {operand_repr} ({operand_value:02X})")
            self._subtract_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x3:
            if DEBUG:
                print(f"> SBC {operand_repr} ({operand_value:02X})")
            self._subtract_with_carry_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x4:
            if DEBUG:
                print(f"> AND {operand_repr} ({operand_value:02X})")
            self._and_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x5:
            if DEBUG:
                print(f"> XOR {operand_repr} ({operand_value:02X})")
            self._xor_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x6:
            if DEBUG:
                print(f"> OR {operand_repr} ({operand_value:02X})")
            self._or_uint8(operand_value)
        elif (opcode >> 3) & 0x7 == 0x7:
            if DEBUG:
                print(f"> CP {operand_repr} ({operand_value:02X})")
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
            if DEBUG:
                print(f"> LD (HL), n ({operand_value:02X})")
            self.memory[self.registers.HL] = operand_value
        else:
            if DEBUG:
                print(f"> LD {dst_reg}, n ({operand_value:02X})")
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
        dst_address = ((opcode >> 3) & 0x7) * 8
        if DEBUG:
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
        if dst_reg == "SP":
            dst_reg = "AF"
        if DEBUG:
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
        if src_reg == "SP":
            src_reg = "AF"
        if DEBUG:
            print(f"> PUSH {src_reg}")

        d16_operand = getattr(self.registers, src_reg)
        self._push_stack(d16_operand)

    def _handle_inc_dec_r16(self, opcode: int):
        """
        Handles INC/DEC instructions for double register or indirect memory address
        :param opcode:
        :return:
        """
        dst_reg_i = (opcode >> 4) & 0x3
        dst_reg = r16_map[dst_reg_i]
        operand_value = getattr(self.registers, dst_reg)
        operand_repr = dst_reg

        if (opcode >> 3) & 1 == 0:
            if DEBUG:
                print(f"> INC {operand_repr}")
            new_value = operand_value + 1
        elif (opcode >> 3) & 1 == 1:
            if DEBUG:
                print(f"> DEC {operand_repr}")
            new_value = operand_value - 1
        else:
            raise ValueError(f"Unexpected opcode {opcode}, expected generic ALU instruction!")

        setattr(self.registers, dst_reg, new_value)

    def _handle_add_r16(self, opcode: int):
        """
        Handles ADD instructions for double register
        :param opcode:
        :return:
        """
        dst_reg_i = (opcode >> 4) & 0x3
        dst_reg = r16_map[dst_reg_i]
        operand_value = getattr(self.registers, dst_reg)
        operand_repr = dst_reg

        if DEBUG:
            print(F"> ADD HL, {operand_repr}")
        value_pre = self.registers.HL
        # 16 bit addition uses the 8 bit ALU, LSB first then MSB, so the resulting flags apply to the high byte
        self.registers.HL += operand_value
        if (value_pre & 0xF) + (operand_value & 0xF) > 0xF:
            self.registers.set_H()
        else:
            self.registers.clear_H()
        if value_pre + operand_value > 0xFF:
            self.registers.set_C()
        else:
            self.registers.clear_C()
        self.registers.clear_N()

    def _handle_load_d16_to_r16(self, opcode: int, extra_bytes: List[int]):
        """
        Handles LD instructions for double immediate to double register
        :param opcode:
        :return:
        """
        immediate = bytes_to_uint16(extra_bytes)
        dst_reg_i = (opcode >> 4) & 0x3
        dst_reg = r16_map[dst_reg_i]
        operand_repr = dst_reg

        if DEBUG:
            print(F"> LD {operand_repr}, d16 ({immediate:04X})")
        setattr(self.registers, dst_reg, immediate)

    def _load_to_r16_address(self, dst_reg: str):
        value = self._read_r8('A')
        dst_address = getattr(self.registers, dst_reg)
        self.memory[dst_address] = value

    def _load_from_r16_address(self, src_reg: str):
        src_address = getattr(self.registers, src_reg)
        value = self.memory[src_address]
        self._load_to_r8('A', value)

    def _handle_indirect_loads(self, opcode: int):
        """
        Handles LD instructions for indirect double register to A and viceversa
        :param opcode:
        :return:
        """
        if (opcode >> 5) & 1 == 1:
            dst_reg = "HL"
        else:
            dst_reg_i = (opcode >> 4) & 0x3
            dst_reg = r16_map[dst_reg_i]

        load_to_acc = (opcode >> 3) & 1 == 1

        if load_to_acc: # LD A, (r16)
            self._load_from_r16_address(dst_reg)
        else: # LD (r16), A
            self._load_to_r16_address(dst_reg)

        if dst_reg == "HL":
            if (opcode >> 4) & 1 == 0:
                self.registers.HL += 1
                if DEBUG:
                    if load_to_acc:
                        print(f"> LD A, (HL+)")
                    else:
                        print(f"> LD (HL+), A")
            else:
                self.registers.HL -= 1
                if DEBUG:
                    if load_to_acc:
                        print(f"> LD A, (HL-)")
                    else:
                        print(f"> LD (HL-), A")
        else:
            if DEBUG:
                if load_to_acc:
                    print(f"> LD A, ({dst_reg})")
                else:
                    print(f"> LD ({dst_reg}), A")

    def _handle_jump_relative_cond(self, opcode: int, extra_bytes: List[int]) -> bool:
        """
        Handles JR conditional instructions
        :param opcode:
        :return: True if condition is met (branch execution), False otherwise
        """
        if (opcode >> 3) & 0x3 == 0x0:
            condition = not self.registers.read_Z()
            cond_repr = "NZ"
        elif (opcode >> 3) & 0x3 == 0x1:
            condition = self.registers.read_Z()
            cond_repr = "Z"
        elif (opcode >> 3) & 0x3 == 0x2:
            condition = not self.registers.read_C()
            cond_repr = "NC"
        else:
            condition = self.registers.read_C()
            cond_repr = "C"

        immediate = byte_to_int8(extra_bytes[0])
        if DEBUG:
            print(F"> JR {cond_repr}, e, ({immediate:02X})")

        if not condition:
            return False

        self.registers.write_PC(self.registers.PC + immediate)
        return True

    def _handle_jump_relative(self, opcode: int, extra_bytes: List[int]):
        """
        Handles JR, e instruction
        :param opcode:
        :return: True if condition is met (branch execution), False otherwise
        """
        immediate = byte_to_int8(extra_bytes[0])
        if DEBUG:
            print(F"> JR e ({immediate:02X})")
        self.registers.write_PC(self.registers.PC + immediate)
        return True

    def _handle_load_d16_to_SP(self, opcode: int, extra_bytes: List[int]):
        """
        Handles 'LD (nn), SP'
        :param opcode:
        :return:
        """
        immediate = bytes_to_uint16(extra_bytes)
        if DEBUG:
            print(F"> LD (d16), SP ({immediate:04X})")
        self.memory[immediate] = self.registers.SP & 0xFF
        self.memory[immediate + 1] = (self.registers.SP >> 8) & 0xFF

    def _handle_load_r16_to_r16(self, opcode: int, extra_bytes: List[int]):
        """
        Handles 'LD HL, SP+e' and 'LD SP, HL'
        :param opcode:
        :return:
        """
        if opcode & 1 == 0: # LD HL, SP+e
            immediate = byte_to_int8(extra_bytes[0])
            if DEBUG:
                print(F"> LD HL, SP+e ({immediate:02X})")

            value_pre = self.registers.SP
            result = self.registers.SP + immediate

            # GB does 8 bit unsigned addition on the lower byte and then inc/dec the the upper one accordingly
            # therefore flags are only affected by the lower byte
            self.registers.HL = result
            if immediate > 0:
                if (value_pre & 0xF) + (immediate & 0xF) > 0xF:
                    self.registers.set_H()
                else:
                    self.registers.clear_H()
                if (value_pre & 0xFF) + (immediate & 0xFF) > 0xFF:
                    self.registers.set_C()
                else:
                    self.registers.clear_C()
            else:
                if (value_pre & 0xF) - (immediate & 0xF) < 0:
                    self.registers.set_H()
                else:
                    self.registers.clear_H()
                if (value_pre & 0xFF) - (immediate & 0xFF) < 0:
                    self.registers.set_C()
                else:
                    self.registers.clear_C()

            self.registers.clear_Z()
            self.registers.clear_N()
        else:
            if DEBUG:
                print(F"> LD SP, HL")
            self.registers.SP = self.registers.HL

    def _handle_rotate_accumulator(self, opcode: int):
        """
        Handles rotate accumulator instructions
        :param opcode:
        :return:
        """
        if (opcode >> 3) & 0x3 == 0x0:
            if DEBUG:
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
        elif (opcode >> 3) & 0x3 == 0x1:
            if DEBUG:
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
        elif (opcode >> 3) & 0x3 == 0x2:
            if DEBUG:
                print(F"> RLA")
            top_bit = (self.registers.A >> 7) & 1
            self.registers.A = ((self.registers.A & 0x7F) << 1) | self.registers.read_C()
            if top_bit:
                self.registers.set_C()
            else:
                self.registers.clear_C()
            self.registers.clear_Z()
            self.registers.clear_N()
            self.registers.clear_H()
        elif (opcode >> 3) & 0x3 == 0x3:
            if DEBUG:
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

    def _decimal_adjust_acc(self):
        if self.registers.read_N():
            if self.registers.read_C():
                self.registers.A -= 0x60
            if self.registers.read_H():
                self.registers.A -= 0x06
        else:
            if self.registers.read_C() or self.registers.A > 0x99:
                self.registers.A += 0x60
                self.registers.set_C()
            if self.registers.read_H() or (self.registers.A & 0x0F) > 0x99:
                self.registers.A += 0x06
            if self.registers.read_H():
                self.registers.A -= 0x06
        if self.registers.A == 0:
            self.registers.set_Z()
        else:
            self.registers.clear_Z()
        self.registers.clear_H()

    def _complement_acc(self):
        self.registers.A ^= 0xFF
        self.registers.set_H()
        self.registers.set_N()

    def _set_carry_flag(self):
        self.registers.set_C()
        self.registers.clear_H()
        self.registers.clear_N()

    def _complement_carry_flag(self):
        if self.registers.read_C():
            self.registers.clear_C()
        else:
            self.registers.set_C()
        self.registers.clear_H()
        self.registers.clear_N()

    def _handle_accumulator_misc(self, opcode: int):
        """
        Handles misc accumulator instructions (DAA, CPL, SCF, CCF)
        :param opcode:
        :return:
        """
        if (opcode >> 3) & 0 == 0:
            if DEBUG:
                print(f"> DAA")
            self._decimal_adjust_acc()
        elif (opcode >> 3) & 0 == 1:
            if DEBUG:
                print(f"> CPL")
            self._complement_acc()
        elif (opcode >> 3) & 0 == 2:
            self._set_carry_flag()
        elif (opcode >> 3) & 0 == 3:
            self._complement_carry_flag()

    def _handle_misc_indirect_loads(self, opcode: int, extra_bytes: List[int]):
        """
        Handles indirect high loads and d16 loads
        :param opcode:
        :return:
        """
        if (opcode >> 1) & 1 == 1:
            if (opcode >> 3) & 1 == 0:
                address = 0xFF00 | self.registers.C
            else:
                address = bytes_to_uint16(extra_bytes)

            if (opcode >> 4) & 0x1 == 0:
                if DEBUG:
                    if (opcode >> 3) & 1 == 0:
                        print(F"> LDH (C), A")
                    else:
                        print(F"> LD (nn), A ({address:04X})")
                self.memory[address] = self.registers.A
            else:
                if DEBUG:
                    if (opcode >> 3) & 1 == 0:
                        print(F"> LDH A, (C)")
                    else:
                        print(F"> LD A, (nn) ({address:04X})")
                self.registers.A = self.memory[address]
        else:
            address = 0xFF00 | extra_bytes[0]
            if (opcode >> 4) & 1 == 0:
                if DEBUG:
                    print(F"> LDH (n), A ({extra_bytes[0]:02X})")
                self.memory[address] = self.registers.A
            else:
                if DEBUG:
                    print(F"> LDH A, (n) ({extra_bytes[0]:02X})")
                self.registers.A = self.memory[address]

    def _handle_return_cond(self, opcode: int) -> bool:
        """
        Handles conditional RET instructions
        :param opcode:
        :return:
        """
        if (opcode >> 3) & 0x3 == 0x0:
            condition = not self.registers.read_Z()
            cond_repr = "NZ"
        elif (opcode >> 3) & 0x3 == 0x1:
            condition = self.registers.read_Z()
            cond_repr = "Z"
        elif (opcode >> 3) & 0x3 == 0x2:
            condition = not self.registers.read_C()
            cond_repr = "NC"
        else:
            condition = self.registers.read_C()
            cond_repr = "C"

        if DEBUG:
            print(F"> RET {cond_repr}")

        if not condition:
            return False

        address = self._pop_stack()
        self.registers.write_PC(address)
        return True

    def _handle_jump_d16_cond(self, opcode: int, extra_bytes: List[int]) -> bool:
        """
        Handles JP conditional instructions
        :param opcode:
        :return: True if condition is met (branch execution), False otherwise
        """
        if (opcode >> 3) & 0x3 == 0x0:
            condition = not self.registers.read_Z()
            cond_repr = "NZ"
        elif (opcode >> 3) & 0x3 == 0x1:
            condition = self.registers.read_Z()
            cond_repr = "Z"
        elif (opcode >> 3) & 0x3 == 0x2:
            condition = not self.registers.read_C()
            cond_repr = "NC"
        else:
            condition = self.registers.read_C()
            cond_repr = "C"

        address = bytes_to_uint16(extra_bytes)
        if DEBUG:
            print(F"> JP {cond_repr}, nn ({address:04X})")

        if not condition:
            return False

        self.registers.write_PC(address)
        return True

    def _handle_jump_absolute_d16(self, opcode: int, extra_bytes: List[int]):
        """
        Handles JP nn
        :param opcode:
        :return: True if condition is met (branch execution), False otherwise
        """
        address = bytes_to_uint16(extra_bytes)
        if DEBUG:
            print(F"> JP nn ({address:04X})")
        self.registers.write_PC(address)

    def _handle_jump_absolute_HL(self, opcode: int, extra_bytes: List[int]):
        """
        Handles JP HL
        :param opcode:
        :return: True if condition is met (branch execution), False otherwise
        """
        if DEBUG:
            print(F"> JP HL")
        address = self.registers.HL
        self.registers.write_PC(address)

    def _handle_call_cond(self, opcode: int, extra_bytes: List[int]) -> bool:
        """
        Handles conditional CALL instructions
        :param opcode:
        :return: True if condition is met (branch execution), False otherwise
        """
        if (opcode >> 3) & 0x3 == 0x0:
            condition = not self.registers.read_Z()
            cond_repr = "NZ"
        elif (opcode >> 3) & 0x3 == 0x1:
            condition = self.registers.read_Z()
            cond_repr = "Z"
        elif (opcode >> 3) & 0x3 == 0x2:
            condition = not self.registers.read_C()
            cond_repr = "NC"
        else:
            condition = self.registers.read_C()
            cond_repr = "C"

        address = bytes_to_uint16(extra_bytes)
        if DEBUG:
            print(F"> CALL {cond_repr}, nn ({address:04X})")

        if not condition:
            return False

        self._push_stack(self.registers.PC)
        self.registers.write_PC(address)
        return True

    def _handle_call_d16(self, opcode: int, extra_bytes: List[int]):
        """
        Handles conditional CALL nn
        :param opcode:
        :return:
        """
        address = bytes_to_uint16(extra_bytes)
        if DEBUG:
            print(f"> CALL nn ({address:04X})")
        self._push_stack(self.registers.PC)
        self.registers.write_PC(address)

    def _handle_return(self, opcode: int):
        """
        Handles RET
        :param opcode:
        :return:
        """
        address = self._pop_stack()
        self.registers.write_PC(address)
        if (opcode >> 4) & 1 == 1:
            if DEBUG:
                print("> RETI")
            self.IME = 1
        else:
            if DEBUG:
                print("> RET")
            pass

    def _handle_add_SP_int8(self, opcode: int, extra_bytes: List[int]):
        """
        Handles 'ADD SP,e'
        :param opcode:
        :return:
        """
        immediate = byte_to_int8(extra_bytes[0])
        if DEBUG:
            print(F"> ADD SP, e ({immediate:02X})")

        value_pre = self.registers.SP
        # 16 bit addition uses the 8 bit ALU, LSB first then MSB, so the resulting flags apply to the high byte
        result = self.registers.SP + immediate

        self.registers.SP = result
        if immediate > 0:
            if (value_pre & 0xF) + (immediate & 0xF) > 0xF:
                self.registers.set_H()
            else:
                self.registers.clear_H()

            if (value_pre & 0xFF) + (immediate & 0xFF) > 0xFF:
                self.registers.set_C()
            else:
                self.registers.clear_C()
        else:
            if (value_pre & 0xF) - (immediate & 0xF) < 0:
                self.registers.set_H()
            else:
                self.registers.clear_H()

            if (value_pre & 0xFF) - (immediate & 0xFF) < 0:
                self.registers.set_C()
            else:
                self.registers.clear_C()

        self.registers.clear_Z()
        self.registers.clear_N()

    def __str__(self):
        return f'{self.registers} | IME: {self.IME} | T: {self.clock} | LCDC: {self.memory[0xFF40]:02X} | STAT: {self.memory[0xFF41]:02X} | LY: {self.memory[0xFF44]:02X}'

    def __repr__(self):
        return self.__str__()