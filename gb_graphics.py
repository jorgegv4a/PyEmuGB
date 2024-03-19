import numpy as np
import cv2


from typing import List
from queue import Queue

from gb_memory import AddressSpace
from gb_constants import Interrupt, SCREEN_HEIGHT, SCREEN_WIDTH, LCDC_OBJ_ENABLE_BIT, LCDC_OBJ_SIZE_BIT, \
    LCDC_BG_TILE_MAP_BIT, LCDC_BG_WINDOW_TILE_DATA_AREA_BIT, LCDC_WINDOW_ENABLE_BIT, LCDC_WINDOW_TILE_MAP_BIT, \
    LCDC_PPU_ENABLE_BIT


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

    def _get_win_tile_map(self) -> int:
        return (self.memory[0xFF40] >> LCDC_WINDOW_TILE_MAP_BIT) & 1

    def _get_bg_win_tile_data_zone(self) -> int:
        return (self.memory[0xFF40] >> LCDC_BG_WINDOW_TILE_DATA_AREA_BIT) & 1

    def _get_bg_tile_map(self) -> int:
        return (self.memory[0xFF40] >> LCDC_BG_TILE_MAP_BIT) & 1

    def get_obj_size(self) -> int:
        return (self.memory[0xFF40] >> LCDC_OBJ_SIZE_BIT) & 1

    def get_ppu_enabled(self) -> int:
        return (self.memory[0xFF40] >> LCDC_PPU_ENABLE_BIT) & 1

    def get_obj_enabled(self) -> int:
        return (self.memory[0xFF40] >> LCDC_OBJ_ENABLE_BIT) & 1

    @property
    def scx(self):
        return self.memory[0xFF43]

    @property
    def scy(self):
        return self.memory[0xFF42]

    @property
    def wx(self):
        return self.memory[0xFF4A]

    @property
    def wy(self):
        return self.memory[0xFF4B]

    def get_background_tile_map(self) -> np.ndarray:
        tile_map = np.zeros((32, 32), dtype=np.uint16)
        if self._get_bg_tile_map():
            tile_map_base_address = 0x9C00
        else:
            tile_map_base_address = 0x9800

        for tile_map_idx in range(1024):
            tile_x = tile_map_idx % 32
            tile_y = tile_map_idx // 32
            tile_offset = self.memory[tile_map_base_address + tile_map_idx]
            if self._get_bg_win_tile_data_zone():
                tile_address = 0x8000 + tile_offset * 16
            else:
                tile_address = 0x9000 + (tile_offset - 128) * 16
            # tile_idx = self.memory[tile_address]
            tile_map[tile_y, tile_x] = tile_address
        return tile_map

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

    def get_tile(self, tile_start_i: int):
        tile = np.zeros((8, 8), np.uint8)
        for j in range(8):
            tile_low_byte = self.memory[tile_start_i + 2 * j]
            tile_high_byte = self.memory[(tile_start_i + 2 * j) + 1]
            for i in range(8):
                low_byte = ((tile_low_byte >> i) & 1)
                high_byte = (((tile_high_byte >> i) & 1) << 1)
                color_id = high_byte | low_byte
                tile[j, 7 - i] = 255 - int(255 / 3 * color_id)
        return tile

    def show(self):
        if self.tick_i % 10000 == 0:
            full_image = np.zeros((256, 256), dtype=np.uint8)
            bg_map = self.memory.get_background_tile_map()
            tiles = {i: self.get_tile(i) for i in np.unique(bg_map)}
            for j in range(32):
                tile_y0 = j * 8
                for i in range(32):
                    tile_x0 = i * 8
                    tile_idx = bg_map[j, i]
                    full_image[tile_y0: tile_y0 + 8, tile_x0: tile_x0 + 8] = tiles[tile_idx]
            view_x0 = self.memory.scx
            view_y0 = self.memory.scy

            # rect from viewport x0y0 to tilemap x1y1
            margin_x = 255 - view_x0
            margin_y = 255 - view_y0
            # how many pixels fit from view_x0 to the edge of the tile map
            fit_x = min(SCREEN_WIDTH, margin_x)
            fit_y = min(SCREEN_HEIGHT, margin_y)
            self.image[:fit_y, :fit_x] = full_image[view_y0: view_y0 + fit_y, view_x0: view_x0 + fit_x]

            # rect from tilemap x0y0 to viewport x1y1
            # how many more pixels we need to fit the edge of the screen
            leftover_x = SCREEN_WIDTH - fit_x
            leftover_y = SCREEN_HEIGHT - fit_y
            self.image[fit_y:, fit_x:] = full_image[margin_y: margin_y + leftover_y, margin_x: margin_x + leftover_x]

            # edge, corner @ viewport x0, tilemap y0 to tilemap x1, viewport y1
            self.image[fit_y:, :fit_x] = full_image[margin_y :margin_y + leftover_y, view_x0: view_x0 + fit_x]

            # edge, corner @ tilemap x0, viewport y0 to viewport x1, tilemap y1
            self.image[:fit_y, fit_x:] = full_image[view_y0: view_y0 + fit_y, margin_x: margin_x + leftover_x]
            cv2.imshow(self.window_name, cv2.resize(self.image, None, fx=2, fy=2))
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
        if not self.memory.get_ppu_enabled():
            return
        if self.mode == LCD_MODE_2:
            continue_scan = True
            if len(self.line_objs) == 10:
                # can only draw up to 10 sprites per line
                self.dot += 1
                continue_scan = False
            obj_i = self.dot
            object_data = SpriteData(self.memory[0xFE00 + obj_i: 0xFE00 + obj_i + 4])
            if object_data.y < 8 and not self.memory.get_obj_size():
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
