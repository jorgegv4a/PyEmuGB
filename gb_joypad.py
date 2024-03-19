from pynput import keyboard

from typing import Union, Optional

from gb_memory import AddressSpace
from gb_constants import Interrupt


class JoypadMaskedMemoryAccess:
    # Prevents PPU from accessing memory it's not supposed to know about
    def __init__(self, cpu_memory: AddressSpace):
        self.memory = cpu_memory

    def request_interrupt(self, interrupt: Interrupt):
        if interrupt == Interrupt.Joypad:
            self.memory.request_interrupt(interrupt)
        else:
            print(f"Joypad requested unexpected interrupt: {interrupt}")
            return

    def _get_register(self):
        return (self.memory[0xFF00] >> 4) & 0x3

    def update_register(self, state: int):
        register = self._get_register()
        selection = (register >> 4) & 0x3
        if selection == 0x3: # no selection, returns no key presses
            self.memory[0xFF00] = (selection << 4) | 0xF

        if selection & 1 == 0: # select d-pad
            self.memory[0xFF00] = (selection << 4) | (state & 0xF)

        if (selection >> 1) & 1 == 0: # select buttons
            self.memory[0xFF00] = (selection << 4) | ((state >> 4) & 0xF)


class JoypadController:
    def __init__(self, memory: AddressSpace):
        self.drawing_current_line: bool = False
        self.memory = JoypadMaskedMemoryAccess(memory)
        # high nibble is Start-Select-B-A, low nibble is Down-Up-Left-Right
        self.state = 0xFF

        listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release)
        listener.start()

    def _on_press(self, key: Optional[Union[keyboard.Key, keyboard.KeyCode]]):
        if key is None:
            return
        try:
            key = key.char
        except AttributeError:
            key = key.name
        if key in ("right", "d"):
            self.state &= 0

        if key in ("left", "a"):
            self.state &= ((1 << 1) ^ 0xFF)

        if key in ("up", "w"):
            self.state &= ((1 << 2) ^ 0xFF)

        if key in ("down", "s"):
            self.state &= ((1 << 3) ^ 0xFF)

        if key == "enter": # A
            self.state &= ((1 << 4) ^ 0xFF)

        if key in ("backspace", "q"): # B
            self.state &= ((1 << 5) ^ 0xFF)

        if key == "e": # Select
            self.state &= ((1 << 6) ^ 0xFF)

        if key == "space": # Start
            self.state &= ((1 << 7) ^ 0xFF)

    def _on_release(self, key):
        if key is None:
            return
        if key in ("right", "d"):
            self.state |= 1

        if key in ("left", "a"):
            self.state |= (1 << 1)

        if key in ("up", "w"):
            self.state |= (1 << 2)

        if key in ("down", "s"):
            self.state |= (1 << 3)

        if key == "enter":  # A
            self.state |= (1 << 4)

        if key in ("backspace", "q"):  # B
            self.state |= (1 << 5)

        if key == "e":  # Select
            self.state |= (1 << 6)

        if key == "space":  # Start
            self.state |= (1 << 7)

    def tick(self):
        self.memory.update_register(self.state)
