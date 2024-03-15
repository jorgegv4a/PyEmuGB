from general import txt


def main():
    src_regs = {
        0: 'B',
        1: 'C',
        2: 'D',
        3: 'E',
        4: 'H',
        5: 'L',
        7: 'A',
    }
    dst_regs = {
        0: 'B',
        1: 'C',
        2: 'D',
        3: 'E',
        4: 'H',
        5: 'L',
        7: 'A',
    }

    text = ""
    for opcode in range(0x00, 0x100):
        bin_str = f"{opcode:02X}"
        # bin_str = f"{opcode:08b}"
        if opcode in (0xD3, 0xE3, 0x34, 0xF4, 0xDB, 0xEB, 0xEC, 0xFC, 0xDD, 0xED, 0xFD):
            text += txt(f"%KkF ----")

        elif 0x40 <= opcode < 0x80:
            src_reg_i = opcode & 0x7
            src_reg = src_regs.get(src_reg_i, None)

            dst_reg_i = (opcode >> 3) & 0x7
            dst_reg = dst_regs.get(dst_reg_i, None)
            # if opcode == 0x76:
            #     text += txt(f"{bin_str}")
            # elif (opcode >> 3) == 0xE:
            #     # text += txt(f"% b {bin_str}")
            #     text += txt(f"% b LD (HL), {src_reg}")
            # elif ((opcode >> 6) & 0x03 == 1) and (opcode & 0x07 == 0x06):
            #     # text += txt(f"% r {bin_str}")
            #     text += txt(f"% r LD {dst_reg}, (HL)")
            # elif (opcode >> 6) & 0x03 == 1:
            #     # text += txt(f"% g {bin_str}")
            #     text += txt(f"% g LD {dst_reg}, {src_reg}")
            # else:
            #     text += txt(f"{bin_str}")

            if opcode == 0x76:
                text += txt(f"%MW HALT")
            elif (opcode >> 6) & 0x03 == 1:
                # text += txt(f"% g {bin_str}")
                bg_color = "b"
                if src_reg is None:
                    src_reg = "(HL)"
                    bg_color = "b"
                elif dst_reg is None:
                    dst_reg = "(HL)"
                    bg_color = "b"
                text += txt(f"% {bg_color} LD {dst_reg}, {src_reg}")
            else:
                text += txt(f"{bin_str}")

        elif 0x80 <= opcode < 0xC0:
            src_reg_i = opcode & 0x7
            src_reg = src_regs.get(src_reg_i, None)
            if src_reg is None:
                src_reg = "(HL)"
            if (opcode >> 3) == 0x10:
                text += txt(f"% y ADD {src_reg}")
            elif (opcode >> 3) == 0x11:
                text += txt(f"% y ADC {src_reg}")
            elif (opcode >> 3) == 0x12:
                text += txt(f"% y SUB {src_reg}")
            elif (opcode >> 3) == 0x13:
                text += txt(f"% y SBC {src_reg}")
            elif (opcode >> 3) == 0x14:
                text += txt(f"% y AND {src_reg}")
            elif (opcode >> 3) == 0x15:
                text += txt(f"% y XOR {src_reg}")
            elif (opcode >> 3) == 0x16:
                text += txt(f"% y OR {src_reg}")
            elif (opcode >> 3) == 0x17:
                text += txt(f"% y CP {src_reg}")
            else:
                text += txt(f"% y {bin_str}")

        elif opcode & 0xC6 == 0x04:
            text += txt(f"% y {bin_str}")

        elif opcode & 0xC7 == 0xC6:
            text += txt(f"% y {bin_str}")

        elif opcode & 0xC7 == 0x06:
            text += txt(f"% b {bin_str}")

        elif opcode & 0xC7 == 0xC7:
            text += txt(f"%R   {bin_str}")

        elif opcode & 0xCF == 0xC1:
            text += txt(f"%g  {bin_str}")

        elif opcode & 0xCF == 0xC5:
            text += txt(f"%g  {bin_str}")

        elif opcode & 0xC7 == 0x03:
            text += txt(f"% m {bin_str}")

        elif opcode & 0xCF == 0x09:
            text += txt(f"% M {bin_str}")

        elif opcode & 0xCF == 0x01:
            text += txt(f"%kG {bin_str}")

        elif opcode & 0xC7 == 0x02:
            text += txt(f"%kb {bin_str}")

        elif opcode == 0x00:
            text += txt(f"%MW NOP")

        elif opcode == 0x10:
            text += txt(f"%MW STOP")

        elif opcode == 0xF3:
            text += txt(f"%MW DI")

        elif opcode == 0xFB:
            text += txt(f"%MW  EI")

        elif opcode & 0xE7 == 0x20 or opcode == 0x18:
            text += txt(f"%Yw {bin_str}")

        elif opcode == 0x08:
            text += txt(f"%kG {bin_str}")

        elif opcode & 0xFE == 0xF8:
            text += txt(f"%kG {bin_str}")

        elif opcode & 0xE7 == 0x07:
            text += txt(f"%Kg {bin_str}")

        elif opcode & 0xE7 == 0x27:
            text += txt(f"% Y {bin_str}")

        elif opcode & 0xE5 == 0xE0 and opcode & 0xEF != 0xE8:
            text += txt(f"% b {bin_str}")

        elif opcode & 0xE7 == 0xC0:
            text += txt(f"%Yw {bin_str}")

        else:
            text += txt(f"{bin_str}")


        if (opcode + 1) % 0x10 != 0:
            text += ", "
        else:
            text += "\n"
    print(text)


if __name__ == "__main__":
    main()