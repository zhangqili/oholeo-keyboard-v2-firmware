#!/usr/bin/env python3
"""Generate an ILM-resident C array from the bootloader binary."""

from pathlib import Path
import sys


BOOTLOADER_IMAGE_FLASH_OFFSET = 0x400
BOOTLOADER_RESERVED_LENGTH = 0x20000
MAX_BOOTLOADER_SIZE = BOOTLOADER_RESERVED_LENGTH - BOOTLOADER_IMAGE_FLASH_OFFSET


def main() -> int:
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} INPUT.bin OUTPUT.c", file=sys.stderr)
        return 2

    input_path = Path(sys.argv[1])
    output_path = Path(sys.argv[2])
    image = input_path.read_bytes()

    if not image:
        print("bootloader image is empty", file=sys.stderr)
        return 1
    if len(image) > MAX_BOOTLOADER_SIZE:
        print(
            f"bootloader image is {len(image)} bytes, "
            f"larger than the {MAX_BOOTLOADER_SIZE}-byte image area "
            f"inside the 0x{BOOTLOADER_RESERVED_LENGTH:x}-byte reserved region",
            file=sys.stderr,
        )
        return 1
    if len(image) % 4:
        print("bootloader image size must be 4-byte aligned", file=sys.stderr)
        return 1

    lines = []
    for offset in range(0, len(image), 16):
        values = ", ".join(f"0x{value:02x}" for value in image[offset : offset + 16])
        lines.append(f"    {values},")

    output = """/* Generated from the current Bootloader build. */
/* The raw flash_xip binary starts at flash offset 0x400. */
#include <stdint.h>
#include "hpm_common.h"

ATTR_PLACE_AT(".fast.data") __attribute__((aligned(4), used))
const uint32_t bootloader_image_size = %dU;

ATTR_PLACE_AT(".fast.data") __attribute__((aligned(4), used))
const uint8_t bootloader_image[%d] = {
%s
};
""" % (len(image), len(image), "\n".join(lines))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(output, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
