from enum import Enum


class Interrupt(Enum):
    VBlank = 0
    LCD = 1
    Timer = 2
    Serial = 3
    Joypad = 4


GB_ROM_BANK_SIZE = 16 * 1024
GB_INTERNAL_RAM_SIZE = 8 * 1024
GB_VRAM_SIZE = 8 * 1024
SCREEN_HEIGHT = 144
SCREEN_WIDTH = 160
CLOCK_SPEED_MHZ = 4.194304
H_SYNC_KHZ = 9198
V_SYNC_HZ = 59.73

OAM_SIZE = 160

CARTRIDGE_RAM_SIZE = 8 * 1024

CARTRIDGE_ROM_ONLY = 0x00
CARTRIDGE_MBC1 = 0x01


# LCD
NUM_DOTS_PER_LINE = 456
NUM_SCAN_LINES = 154
