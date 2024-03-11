from pathlib import Path
from typing import Dict


from gb_cpu import CPU


def main():
    file_path = Path("/home/mojonero/Downloads/Tetris (JUE) (V1.1)/Tetris (JUE) (V1.1) [!].gb")
    with open(file_path, "rb") as file:
        ROM = file.read()

    cpu = CPU(ROM)
    cpu.run()
    print()


if __name__ == '__main__':
    main()
