from gb_constants import GB_ROM_BANK_SIZE, GB_INTERNAL_RAM_SIZE, GB_VRAM_SIZE, OAM_SIZE, CARTRIDGE_ROM_ONLY, Interrupt

FLAG_Z_MASK = 0x80
FLAG_N_MASK = 0x40
FLAG_H_MASK = 0x20
FLAG_C_MASK = 0x10


class AddressSpace:
    def __init__(self):
        self.size = 64 * 1024
        self.rom_bank = bytearray(GB_ROM_BANK_SIZE)         # 0x0000 - 0x3FFF, initially empty except for the boot ROM 0x0000 - 0x00FF
        self.active_rom_bank = bytearray(GB_ROM_BANK_SIZE)  # 0x4000 - 0x7FFF, switchable additional bank memory
        self.vram = bytearray(GB_VRAM_SIZE)                 # 0x8000 - 0x9FFF, video RAM
        self.ram_bank = bytearray(GB_INTERNAL_RAM_SIZE)     # 0xA000 - 0xBFFF, switchable additional bank memory
        self.internal_ram = bytearray(GB_INTERNAL_RAM_SIZE) # 0xC000 - 0xDFFF, internal RAM
                                                            # 0xE000 - 0xFDFF, echo of internal RAM
        self.oam = bytearray(OAM_SIZE)                      # 0xFE00 - 0xFE9F, object attribute memory
        self.empty_io = bytearray(96)                       # 0xFEA0 - 0xFEFF, empty but unusable for I/O
        self.standard_io = bytearray(76)                    # 0xFF00 - 0xFF4B, I/O ports
        self.empty_io2 = bytearray(52)                      # 0xFF4C - 0xFF7F, empty but unusable for I/O
        self.hram = bytearray(127)                          # 0xFF80 - 0xFFFE, high RAM
        self.interrupt_enable = bytearray(1)                # 0xFFFF, interrupt enable register

        self.memory_boundaries = [
            0x0000,
            0x4000,
            0x8000,
            0xA000,
            0xC000,
            0xE000,
            0xFE00,
            0xFEA0,
            0xFF00,
            0xFF4C,
            0xFF80,
            0xFFFF,
        ]
        self.dma_start_address = None
        self.dma_clock_t = 0

    def load_rom(self, rom: bytes, cartridge_type: int):
        rom_size = len(rom)
        if rom_size < 0x8000:
            raise ValueError('ROM size too small')

        if rom_size > 0x8000:
            raise ValueError('ROM size too large')

        if cartridge_type != CARTRIDGE_ROM_ONLY:
            raise NotImplementedError('Only ROM only cartridges are supported')
        else:
            # ROM only cartridge, reads as a single 32KB ROM bank (0x0000 - 0x7FFF)
            for i in range(len(self.rom_bank)):
                self.rom_bank[i] = rom[i]
            for i in range(len(self.active_rom_bank)):
                self.active_rom_bank[i] = rom[len(self.rom_bank) + i]

    def request_interrupt(self, interrupt: Interrupt):
        interrupt_bit_mask = (1 << interrupt.value)
        self.standard_io[0xFF0F - 0xFF00] = self.standard_io[0xFF0F - 0xFF00] | interrupt_bit_mask

    def tick(self):
        if self.dma_start_address is None:
            return
        self.oam[self.dma_clock_t // 4] = self[self.dma_start_address + self.dma_clock_t // 4]
        self.dma_clock_t += 1
        if self.dma_clock_t >= 0xa0:
            self.dma_clock_t = 0
            self.dma_start_address = None


    def __getitem__(self, index):
        if isinstance(index, slice):
            start = index.start if index.start is not None else 0
            stop = index.stop if index.stop is not None else self.size
            step = index.step if index.step is not None else 1
            return [self[i] for i in range(start, stop, step)]
        if index < 0 or index >= self.size:
            raise ValueError(f'Address {index:04X} out of range')

        if index < 0x4000:
            return self.rom_bank[index]

        elif index < 0x8000:
            normalized_index = index - 0x4000
            return self.active_rom_bank[normalized_index]

        elif index < 0xA000:
            normalized_index = index - 0x8000
            return self.vram[normalized_index]

        elif index < 0xC000:
            normalized_index = index - 0xA000
            return self.ram_bank[normalized_index]

        elif index < 0xE000:
            normalized_index = index - 0xC000
            return self.internal_ram[normalized_index]

        elif index < 0xFE00:
            # echo of internal RAM
            normalized_index = index - 0xE000
            return self.internal_ram[normalized_index]

        elif index < 0xFEA0:
            normalized_index = index - 0xFE00
            return self.oam[normalized_index]

        elif index < 0xFF00:
            normalized_index = index - 0xFEA0
            return self.empty_io[normalized_index]

        elif index < 0xFF4C:
            normalized_index = index - 0xFF00
            return self.standard_io[normalized_index]

        elif index < 0xFF80:
            normalized_index = index - 0xFF4C
            return self.empty_io2[normalized_index]

        elif index < 0xFFFF:
            normalized_index = index - 0xFF80
            return self.hram[normalized_index]

        elif index == 0xFFFF:
            return self.interrupt_enable[0]
        else:
            print(f"Unaccounted for index {index} on read")

    def __setitem__(self, index, value):
        if index < 0 or index >= self.size:
            raise ValueError(f'Address {index:04X} out of range')

        if index < 0x4000:
            # ROM bank 0, not writeable
            print(f"tried to access {index:02X} which is not writeable")
            return

        elif index < 0x8000:
            # ROM bank 1, not writeable
            print(f"tried to access {index:02X} which is not writeable")
            return

        elif index < 0xA000:
            normalized_index = index - 0x8000
            self.vram[normalized_index] = value

        elif index < 0xC000:
            normalized_index = index - 0xA000
            self.ram_bank[normalized_index] = value

        elif index < 0xE000:
            normalized_index = index - 0xC000
            self.internal_ram[normalized_index] = value

        elif index < 0xFE00:
            # echo of internal RAM
            normalized_index = index - 0xE000
            self.internal_ram[normalized_index] = value

        elif index < 0xFEA0:
            normalized_index = index - 0xFE00
            self.oam[normalized_index] = value

        elif index < 0xFF00:
            normalized_index = index - 0xFEA0
            self.empty_io[normalized_index] = value

        elif index < 0xFF4C:
            if index == 0xFF46:
                self.dma_start_address = value * 0x100
                return
            normalized_index = index - 0xFF00
            self.standard_io[normalized_index] = value

        elif index < 0xFF80:
            normalized_index = index - 0xFF4C
            self.empty_io2[normalized_index] = value

        elif index < 0xFFFF:
            normalized_index = index - 0xFF80
            self.hram[normalized_index] = value

        elif index == 0xFFFF:
            self.interrupt_enable[0] = value
            print(f"interrupt enable now = {value:08b}")
            # TODO: trigger interrupt enable
        else:
            print(f"Unaccounted for index {index:02X} on write")


class SingleRegister:
    def __init__(self, value=0):
        self.max_value = (2 ** 8) - 1
        self.value = value

    def __getitem__(self, index):
        if index < 0 or index >= 8:
            raise ValueError("Index must be between 0 and 7")

        return (self.value >> index) & 1

    def __setitem__(self, index, value):
        if index < 0 or index >= 8:
            raise ValueError("Index must be between 0 and 7")

        if value == 1:
            self.value = self.value & (1 << index)
        elif value == 0:
            self.value = self.value | (1 << index)
        if value not in (0, 1):
            raise ValueError("Value must be 0 or 1")

    def __str__(self):
        return f'{self.value:02X}'

    def __repr__(self):
        return self.__str__()

    def __setattr__(self, key, value):
        if key == 'value':
            if value < 0:
                raise ValueError('Value must be greater than or equal to 0')
            if value > self.max_value:
                raise ValueError(f'Value must be less than or equal to {self.max_value}')
        super().__setattr__(key, value)


class DoubleRegister:
    def __init__(self, value=0):
        self.max_value = ((2 ** 8) ** 2) - 1
        self.value = value

    def get_high(self):
        return self.value >> 8

    def get_low(self):
        return self.value & 0xFF

    def __getitem__(self, item):
        if item == 0:
            return self.get_low()
        elif item == 1:
            return self.get_high()
        else:
            raise ValueError('Index must be 0 or 1')

    def __setitem__(self, key, value):
        if key == 0:
            if value < 0:
                self.value = ((self.get_high() << 8) | (abs(value) ^ 0xFF) + 1) & 0xFFFF
            else:
                self.value = (self.get_high() << 8) | (value & 0xFF) & 0xFFFF
        elif key == 1:
            self.value = ((value << 8) | self.get_low()) & 0xFFFF
        else:
            raise ValueError('Index must be 0 or 1')

    def __str__(self):
        return f'{self.value:04X}'

    def __repr__(self):
        return self.__str__()

    def __setattr__(self, key, value):
        if key == 'value':
            if value < 0:
                value = (abs(value) ^ 0xFFFF) + 1
            if value > self.max_value:
                # raise ValueError(f'Value must be less than or equal to {self.max_value}')
                value = value & 0xFFFF
        super().__setattr__(key, value)


class RegisterBank:
    def __init__(self):
        # self.registers = [Register(0) for _ in range(4)]
        self._AF = DoubleRegister()
        self._BC = DoubleRegister()
        self._DE = DoubleRegister()
        self._HL = DoubleRegister()
        self._SP = DoubleRegister() # SP
        self._PC = DoubleRegister() # PC

    def increment_pc(self, value=1):
        self._PC.value += value

    @property
    def AF(self):
        return self._AF.value

    @AF.setter
    def AF(self, value):
        self._AF.value = value & 0xFFF0 # lower nibble of F is always 0

    @property
    def BC(self):
        return self._BC.value

    @BC.setter
    def BC(self, value):
        self._BC.value = value

    @property
    def DE(self):
        return self._DE.value

    @DE.setter
    def DE(self, value):
        self._DE.value = value

    @property
    def HL(self):
        return self._HL.value

    @HL.setter
    def HL(self, value):
        self._HL.value = value

    @property
    def A(self):
        return self._AF[1]

    @A.setter
    def A(self, value):
        self._AF[1] = value

    @property
    def F(self):
        return self._AF[0]

    # @F.setter
    # def F(self, value):
    #     self._AF[0] = value

    @property
    def B(self):
        return self._BC[1]

    @B.setter
    def B(self, value):
        self._BC[1] = value

    @property
    def C(self):
        return self._BC[0]

    @C.setter
    def C(self, value):
        self._BC[0] = value

    @property
    def D(self):
        return self._DE[1]

    @D.setter
    def D(self, value):
        self._DE[1] = value

    @property
    def E(self):
        return self._DE[0]

    @E.setter
    def E(self, value):
        self._DE[0] = value

    @property
    def H(self):
        return self._HL[1]

    @H.setter
    def H(self, value):
        self._HL[1] = value

    @property
    def L(self):
        return self._HL[0]

    @L.setter
    def L(self, value):
        self._HL[0] = value

    @property
    def SP(self):
        return self._SP.value

    @SP.setter
    def SP(self, value):
        self._SP.value = value

    @property
    def PC(self):
        return self._PC.value

    def write_PC(self, value):
        self._PC.value = value

    def read_Z(self):
        return (self.F >> 7) & 1

    def read_N(self):
        return (self.F >> 6) & 1

    def read_H(self):
        return (self.F >> 5) & 1

    def read_C(self):
        return (self.F >> 4) & 1

    def set_Z(self):
        self._AF[0] = self.F | FLAG_Z_MASK

    def set_N(self):
        self._AF[0] = self.F | FLAG_N_MASK

    def set_H(self):
        self._AF[0] = self.F | FLAG_H_MASK

    def set_C(self):
        self._AF[0] = self.F | FLAG_C_MASK

    def clear_Z(self):
        self._AF[0] = self.F & (0xFF ^ FLAG_Z_MASK)

    def clear_N(self):
        self._AF[0] = self.F & (0xFF ^ FLAG_N_MASK)

    def clear_H(self):
        self._AF[0] = self.F & (0xFF ^ FLAG_H_MASK)

    def clear_C(self):
        self._AF[0] = self.F & (0xFF ^ FLAG_C_MASK)

    def __str__(self):
        flags = ('Z' if self.read_Z() else '-') + ('N' if self.read_N() else '-') + ('H' if self.read_H() else '-') + ('C' if self.read_C() else '-')
        return f'AF: {self._AF}, BC: {self._BC}, DE: {self._DE}, HL: {self._HL}, SP: {self.SP:04X}, PC: {self.PC:04X}, F: {flags}'

    def __repr__(self):
        return self.__str__()


if __name__ == "__main__":
    # register = Register(12345)
    registers = RegisterBank()
    registers.BC = 0x0012
    registers.B -= 1
    print()

    # memory = AddressSpace()
    # memory[100:]
    print()