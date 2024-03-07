from gb_constants import NUM_SCAN_LINES, NUM_DOTS_PER_LINE


LCD_MODE_0 = 0 # H-Blank
LCD_MODE_1 = 1 # V-Blank
LCD_MODE_2 = 2 # OAM Scan
LCD_MODE_3 = 3 # Drawing


class LCDController:
    def __init__(self):
        self.dot: int = 0
        self.ly: int = 0
        self.drawing_current_line: bool = False

    @property
    def mode(self) -> int:
        if self.dot < 80:
            return LCD_MODE_2
        elif self.drawing_current_line:
            return LCD_MODE_3
        elif self.ly < 144:
            return LCD_MODE_0
        else:
            return LCD_MODE_1
