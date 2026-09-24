"""Check ARM ELF load segments, Intel HEX checksums and identical BIN bytes."""
import hashlib
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = 0x08003000
FLASH_END = BASE + 500 * 1024
RAM_BASE, RAM_END = 0x20000000, 0x20010000


def main():
    binary = (ROOT / "CM530.bin").read_bytes()
    elf = (ROOT / "CM530.elf").read_bytes()
    assert elf[:7] == b"\x7fELF\x01\x01\x01", "expected little-endian ELF32"
    assert struct.unpack_from("<H", elf, 18)[0] == 40, "expected ARM machine"
    entry, phoff = struct.unpack_from("<II", elf, 24)
    phsize, phcount = struct.unpack_from("<HH", elf, 42)
    assert phsize == 32
    segments = []
    for i in range(phcount):
        kind, offset, vaddr, paddr, size, memsize, flags, align = struct.unpack_from("<8I", elf, phoff + i * phsize)
        if kind != 1:
            continue
        if RAM_BASE <= vaddr < RAM_END:
            assert vaddr + memsize <= RAM_END, "RAM segment exceeds allocation"
        segments.append((offset, vaddr, paddr, size))
    # ELF LOAD segments can include headers/page padding below the application
    # origin. objcopy emits allocated sections only, not that segment padding.
    shoff = struct.unpack_from("<I", elf, 32)[0]
    shsize, shcount = struct.unpack_from("<HH", elf, 46)
    assert shsize == 40
    loaded = 0
    for i in range(shcount):
        _, kind, flags, vaddr, offset, size, _, _, _, _ = struct.unpack_from("<10I", elf, shoff + i * shsize)
        if not (flags & 2) or kind == 8 or not size:
            continue
        matches = [(po, va, pa, ps) for po, va, pa, ps in segments
                   if po <= offset and offset + size <= po + ps and vaddr == va + offset - po]
        assert len(matches) == 1
        po, va, pa, ps = matches[0]
        paddr = pa + offset - po
        assert BASE <= paddr < FLASH_END and paddr + size <= FLASH_END
        assert binary[paddr - BASE:paddr - BASE + size] == elf[offset:offset + size]
        loaded += size
    assert loaded > 0
    memory, upper, eof = {}, 0, False
    for text in (ROOT / "CM530.hex").read_text().splitlines():
        assert not eof, "data after HEX EOF"
        assert text.startswith(":")
        record = bytes.fromhex(text[1:])
        assert len(record) == record[0] + 5 and sum(record) % 256 == 0
        address = int.from_bytes(record[1:3], "big")
        kind, data = record[3], record[4:-1]
        if kind == 0:
            for i, value in enumerate(data):
                absolute = upper + address + i
                assert absolute not in memory, "overlapping HEX records"
                memory[absolute] = value
        elif kind == 1:
            assert not data
            eof = True
        elif kind == 4:
            assert len(data) == 2
            upper = int.from_bytes(data, "big") << 16
        elif kind == 5:
            assert int.from_bytes(data, "big") == entry
        else:
            raise AssertionError("unexpected HEX record type {}".format(kind))
    assert eof and min(memory) == BASE
    assert max(memory) + 1 == BASE + len(binary) <= FLASH_END
    reconstructed = bytes(memory.get(BASE + i, 0) for i in range(len(binary)))
    assert reconstructed == binary, "HEX and BIN differ"
    sp, reset = struct.unpack_from("<II", binary)
    assert sp == RAM_END and sp % 8 == 0
    assert reset & 1 and BASE <= reset - 1 < BASE + len(binary)
    assert entry == reset, "ELF entry must be the Thumb Reset_Handler"
    assert bytes((17, 3, 2, 15, 12, 1, 8, 16)) in binary, "A/B ID map missing"
    print("PASS: ARM ELF, HEX checksums, identical BIN, vectors and A/B ID map")
    print("Flash base=0x{:08X}; stack=0x{:08X}; reset=0x{:08X}".format(BASE, sp, reset))
    for name in ("CM530.hex", "CM530.bin"):
        data = (ROOT / name).read_bytes()
        print("{}: {} bytes; SHA256 {}".format(name, len(data), hashlib.sha256(data).hexdigest()))


if __name__ == "__main__":
    main()
