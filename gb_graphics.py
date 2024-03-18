import numpy as np
import cv2


from typing import List
from queue import Queue

from gb_memory import AddressSpace
from gb_constants import Interrupt, SCREEN_HEIGHT, SCREEN_WIDTH


LCD_MODE_0 = 0 # H-Blank
LCD_MODE_1 = 1 # V-Blank
LCD_MODE_2 = 2 # OAM Scan
LCD_MODE_3 = 3 # Drawing


class MaskedMemoryAccess:
    # Prevents PPU from accessing memory it's not supposed to know about
    def __init__(self, cpu_memory: AddressSpace):
        self.memory = cpu_memory

    def request_interrupt(self, interrupt: Interrupt):
        if interrupt == Interrupt.VBlank:
            self.memory.request_interrupt(interrupt)
        elif interrupt == Interrupt.LCD:
            pass
        else:
            print(f"PPU requested unexpected interrupt: {interrupt}")
            return

    def get_stat_mode(self) -> int:
        status = self.memory[0xFF41]
        mode_bitmap = (status >> 3) & 7
        return mode_bitmap

    def set_ly(self, ly: int, prev_ly: int):
        self.memory[0xFF44] = ly
        lyc = self.memory[0xFF45]
        lyc_compare_mode = self.memory[0xFF41]
        interrupt_on_equal_lyc = ((lyc_compare_mode >> 6) & 1) == 1

        ly_equals_lyc = ly == lyc
        if ly_equals_lyc:
            self.memory[0xFF41] = self.memory[0xFF41] | (1 << 2)
            if interrupt_on_equal_lyc and prev_ly != ly:
                self.request_interrupt(Interrupt.LCD)
        else:
            self.memory[0xFF41] = self.memory[0xFF41] & (0xFF ^ (1 << 2))

    def __getitem__(self, index):
        if isinstance(index, slice):
            start = index.start if index.start is not None else 0
            stop = index.stop if index.stop is not None else self.memory.size
            step = index.step if index.step is not None else 1
            return [self[i] for i in range(start, stop, step)]

        if 0x8000 <= index < 0xA000: # VRAM
            return self.memory[index]
        elif 0xFE00 <= index < 0xFEA0: # OAM
            return self.memory[index]
        elif index == 0xFF40: # LCD control
            return self.memory[index]
        elif index == 0xFF41: # LCD STATUS
            return self.memory[index]
        elif index == 0xFF45: # LCD LY COMPARE VALUE
            return self.memory[index]
        else:
            raise ValueError(f'PPU cannot access  #{index:04X}')

    def __setitem__(self, index, value):
        if 0x8000 <= index < 0xA000: # VRAM
            self.memory[index] = value
        elif 0xFE00 <= index < 0xFEA0:  # OAM
            self.memory[index] = value
        elif index == 0xFF41:  # LCD status
            self.memory[index] = value
        else:
            raise ValueError(f'PPU cannot access  #{index:04X}')


LCDC_OBJ_ENABLE_BIT = 1
LCDC_OBJ_SIZE_BIT = 2
LCDC_BG_TILE_MAP_BIT = 3
LCDC_BG_WINDOW_MAP_BIT = 4
LCDC_WINDOW_ENABLE_BIT = 5
LCDC_WINDOW_TILE_MAP_BIT = 6
LCDC_PPU_ENABLE_BIT = 7


class LCDController:
    def __init__(self, memory: AddressSpace):
        self.dot: int = 0
        self.ly: int = 0
        self.drawing_current_line: bool = False
        self.memory = MaskedMemoryAccess(memory)
        self.bg_pixel_fifo: Queue = Queue(16)
        self.obj_pixel_fifo: Queue = Queue(16)
        self.line_objs: List[SpriteData] = []
        self.past_mode = None
        self.past_ly: int = 0

        self.window_name = "Game"
        self.image = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
        self.tick_i = 0
        self.show()

    def show(self):
        if self.tick_i % 10000 == 0:
            tiles_per_row = SCREEN_WIDTH // 8
            max_tiles_per_screen = (SCREEN_WIDTH // 8) * (SCREEN_HEIGHT // 8)
            for tile_start_i in np.arange(0x8000, 0x8000 + max_tiles_per_screen * 16, 16):
                tile_i = (tile_start_i - 0x8000) // 16
                tile_x0 = ((tile_i * 8) % SCREEN_WIDTH)
                tile_y0 = (tile_i // tiles_per_row) * 8
                for j in range(8):
                    tile_low_byte = self.memory[tile_start_i + 2 * j]
                    tile_high_byte = self.memory[(tile_start_i + 2 * j) + 1]
                    for i in range(8):
                        low_byte = ((tile_low_byte >> i) & 1)
                        high_byte = (((tile_high_byte >> i) & 1) << 1)
                        color_id = high_byte | low_byte
                        self.image[tile_y0 + j, tile_x0 + 7 - i] = 255 - int(255 / 3 * color_id)

            cv2.imshow(self.window_name, cv2.resize(self.image, None, fx=4, fy=4))
            cv2.waitKey(1)

    @property
    def mode(self) -> int:
        if self.ly >= 144:
            return LCD_MODE_1
        elif self.dot < 80:
            return LCD_MODE_2
        elif self.drawing_current_line:
            return LCD_MODE_3
        elif self.ly < 144:
            return LCD_MODE_0

    def __get_obj_size(self) -> int:
        return (self.memory[0xFF40] >> LCDC_OBJ_SIZE_BIT) & 1

    def __get_ppu_enabled(self) -> int:
        return (self.memory[0xFF40] >> LCDC_PPU_ENABLE_BIT) & 1

    def __get_obj_enabled(self) -> int:
        return (self.memory[0xFF40] >> LCDC_OBJ_ENABLE_BIT) & 1

    def tick(self):
        """
        Tile Data
        ---------
        VRAM 0x8000 - 0x97FF holds 384 tiles of 16 bytes each.
        Each tile contains 8 rows of pixels
        Each row is defined by two bytes
        Each pixel is defined by a 2bit color index
        ---------

        Tile Map
        ---------
        VRAM 0x9800 - 0x9FFF holds two 32x32 tile maps
        Each tile map holds the 1 byte tile index (maps to Tile Data)


        Object Attribute Memory
        ---------
        OAM 0xFE00 - 0xFE9F holds 40 objects (sprites)
        Each object is defined by 4 bytes
        All sprites are either 8x8 or 8x16 pixels
        """
        if not self.__get_ppu_enabled():
            return
        if self.mode == LCD_MODE_2:
            continue_scan = True
            if len(self.line_objs) == 10:
                # can only draw up to 10 sprites per line
                self.dot += 1
                continue_scan = False
            obj_i = self.dot
            object_data = SpriteData(self.memory[0xFE00 + obj_i: 0xFE00 + obj_i + 4])
            if object_data.y < 8 and not self.__get_obj_size():
                # 8x8 sprite outside screen
                self.dot += 1
                continue_scan = False
            elif object_data.y >= 160:
                # 8x8/8x16 sprite outside screen
                self.dot += 1
                continue_scan = False

            if continue_scan:
                self.line_objs.append(object_data)
                self.dot += 1
                if self.dot == 80:
                    self.drawing_current_line = True

        elif self.mode == LCD_MODE_3:
            self.dot += 1
            if self.dot == 80 + 172:
                self.drawing_current_line = False

        elif self.mode == LCD_MODE_0:
            self.dot += 1
            if self.dot == 456:
                self.dot = 0
                self.ly += 1

        elif self.mode == LCD_MODE_1:
            self.dot += 1
            if self.dot == 456:
                self.dot = 0
                self.ly += 1
            if self.ly == 154:
                self.ly = 0

        self.memory.set_ly(self.ly, self.past_ly)

        if self.past_mode is not None:
            if self.mode == LCD_MODE_1:
                self.memory.request_interrupt(Interrupt.VBlank)
            if self.mode != self.past_mode:
                modes = self.memory.get_stat_mode()
                if (modes & 1 == 1) and self.mode == LCD_MODE_0:
                    self.memory.request_interrupt(Interrupt.LCD)
                elif ((modes >> 1) & 1 == 1) and self.mode == LCD_MODE_1:
                    self.memory.request_interrupt(Interrupt.LCD)
                elif ((modes >> 2) & 1 == 1) and self.mode == LCD_MODE_2:
                    self.memory.request_interrupt(Interrupt.LCD)
        self.past_mode = self.mode
        self.past_ly = self.ly
        self.show()
        self.tick_i += 1
        return


SPRITE_PALETTE_BIT = 4
SPRITE_X_FLIP = 5
SPRITE_Y_FLIP = 5
SPRITE_PRIO = 7


class SpriteData:
    def __init__(self, data_bytes: List[int]):
        assert len(data_bytes) == 4
        self.x = data_bytes[0]
        self.y = data_bytes[1]
        self.raw_tile_index = data_bytes[2]
        self.attrs = data_bytes[3]

    @property
    def palette(self) -> int:
        return (self.attrs >> SPRITE_PALETTE_BIT) & 1

    @property
    def x_flip(self) -> int:
        return (self.attrs >> SPRITE_X_FLIP) & 1

    @property
    def y_flip(self) -> int:
        return (self.attrs >> SPRITE_Y_FLIP) & 1

    @property
    def priority(self) -> int:
        return (self.attrs >> SPRITE_PRIO) & 1

    def __str__(self):
        return f"Sprite #{self.raw_tile_index:02X} (X: {self.x:02X}, Y: {self.y:02X}, Fl X: {self.x_flip}, Fl Y: {self.y_flip}, Palette: {self.palette}, P: {self.priority})"

    def __repr__(self):
        return self.__str__()


def get_tiles(memory):
    print()
