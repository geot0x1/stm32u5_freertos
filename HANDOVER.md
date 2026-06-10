# Handover — NVS on STM32U545 Internal Flash

**Date:** 2026-06-10
**Board:** NUCLEO-U545RE-Q (STM32U545, LQFP64, 512 KB flash, 256 KB RAM)
**Branch:** `develop`
**Status:** NVS mounts, but entry writes fail. Root cause identified (see below) — fix not yet applied.

---

## 1. Build

```
cd build
cmake .. -G Ninja
ninja
```

- Toolchain: `arm-none-eabi-gcc 14.3.1` at `C:\ARMToolchain\arm-gnu-toolchain-14.3.rel1`
- Output: `build/stmtest.elf` (linker script `STM32U545xx_FLASH.ld`)
- The build generator is **Ninja**. If CMake ever errors with "does not match the generator used previously", delete `build/CMakeCache.txt` + `build/CMakeFiles` and re-run with `-G Ninja`.

## 2. Module map (created/changed this session)

| Module | Path | Role |
|---|---|---|
| `nvs` | [Core/Src/nvs/](Core/Src/nvs/) | Key-value log-structured storage. **User replaced this with a new version designed for STM32C0** (8-byte program granularity). Takes a driver via dependency injection (`nvs_flash_driver_t`). Public API: `nvs_mount/nvs_write/nvs_read/nvs_delete`. |
| `stm32u5_flash_driver` | [Core/Src/stm32u5_flash_driver/](Core/Src/stm32u5_flash_driver/) | Implements `nvs_flash_driver_t` on U5 internal flash. **This is where the bugs are.** |
| `crc_gen` (lib target `crc8`) | [Core/Src/crc_gen/](Core/Src/crc_gen/) | Provides `crc32_gen()` (table-driven) used by NVS, plus a stub `crc8()`. |
| `trace_logger`, `assert_macro` | [Core/Src/trace_logger/](Core/Src/trace_logger/), [Core/Src/assert_macro/](Core/Src/assert_macro/) | Header-only stubs created to satisfy older NVS includes. |
| `serial_flash_mem` | [Core/Src/serial_flash_mem/](Core/Src/serial_flash_mem/) | **Leftover** from the previous NVS iteration. Built but no longer linked into the executable. Candidate for deletion. |

Integration point: [main.c:39-51](Core/Src/main.c#L39-L51) (`nvs_init()`) and the test sequence in `test_task` ([main.c:53-82](Core/Src/main.c#L53-L82)) which writes key `"device_id"` = `{0x12,0x34,0x56,0x78}` and reads it back.

NVS flash region (defined in [stm32u5_flash_driver.c](Core/Src/stm32u5_flash_driver/stm32u5_flash_driver.c)): last 64 KB of flash, per user request —
- `NVS_BASE_ADDR = 0x08070000`, 2 sectors × 32 KB (`0x08070000`, `0x08078000`).

## 3. Observed behavior (board logs)

```
NVS mounted successfully            ← header magic "NVS!" exists at 0x08070000 from an earlier run
[FLASH] Write qword at 0x08070008 ...
[FLASH] Status: 0x000000a0          ← write FAILS
[FLASH] Read: ff ff ff ff           ← nothing landed in flash
NVS write result: 0                 ← NVS doesn't check driver errors (driver API returns void)
NVS read result: 1 (NOT_FOUND)
```

## 4. Root cause — CONFIRMED

`NSSR = 0x000000A0` decodes to **PGAERR (bit 5) + PGSERR (bit 7)**: programming *alignment* error + programming *sequence* error. It is **not** write protection (an earlier theory in the session — disregard it; WRPERR is bit 4 and is not set).

Two hard constraints of STM32U5 flash collide with the new NVS layout:

1. **16-byte program granularity.** U5 programs only full quadwords at 16-byte-aligned addresses. The new NVS was written for STM32C0 (8-byte double-words): `align8()` and `NVS_SECTOR_HDR_SIZE = 8` in [nvs.c:27](Core/Src/nvs/nvs.c#L27) / [nvs.h:42](Core/Src/nvs/nvs.h#L42). So the first entry is written at sector offset `0x8` → not quadword-aligned → **PGAERR**.
2. **One program per quadword (ECC).** Each quadword can be programmed only once between erases. The 8-byte sector header occupies quadword `0x00–0x0F` (driver pads with 0xFF); the first entry at offset `0x8` would re-program that same quadword → **PGSERR**. This means *no driver-only workaround is fully correct* — RAM-caching half-written quadwords breaks across reboots.

**Conclusion: NVS's write granularity must become 16 bytes on this part.** Minimal change (2 lines in NVS, user approval needed since they asked not to modify NVS):
- `align8()` → align to 16: `(v + 15U) & ~15U`
- `NVS_SECTOR_HDR_SIZE` → `16U`

Everything else in NVS derives offsets from these, so the change is self-consistent. (The 8-byte entry header + key + data simply get padded to a 16-byte boundary by `entry_total_size()`.)

## 5. Additional bugs in the driver (must fix regardless)

Current HEAD of [stm32u5_flash_driver.c](Core/Src/stm32u5_flash_driver/stm32u5_flash_driver.c) is an **untested experiment using raw register indices — revert it to HAL calls.** Specifically:

1. **Wrong register indices.** `flash_base[9]` is `SECSR` (0x24), not `NSSR` (0x20 = index 8); `flash_base[11]` is `SECCR` (0x2C) — writing `0xFFFFFFFF` there is dangerous. There is **no `NSCCR` register on U5**; error flags are cleared by writing 1s to `NSSR` itself (`FLASH->NSSR = FLASH->NSSR & 0x...` style write-1-to-clear), which is what `HAL_FLASH_Program` already does internally. Just use `HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, addr, (uint32_t)buf)` — it was returning HAL_ERROR *because of the alignment issue above*, not because the HAL was wrong. (Note: HAL clears NSSR flags after recording them in `pFlash.ErrorCode`, which is why NSSR read `0x0` after HAL calls — check `HAL_FLASH_GetError()` instead.)
2. **Wrong erase bank.** U545 flash is **dual-bank** (`FLASH_BANK_SIZE = 256 KB`, 32 × 8 KB pages per bank). The NVS region `0x08070000+` is in **Bank 2**, but `stm32u5_flash_erase_sector()` hardcodes `FLASH_BANK_1` and an absolute page number (56) that doesn't exist in a 32-page bank. Correct computation:
   ```c
   uint32_t offset = addr - 0x08000000UL;
   erase_init.Banks = (offset < FLASH_BANK_SIZE) ? FLASH_BANK_1 : FLASH_BANK_2;
   erase_init.Page  = (offset % FLASH_BANK_SIZE) / FLASH_PAGE_SIZE;   /* FLASH_PAGE_SIZE = 0x2000 */
   ```
3. **ICACHE staleness.** `MX_ICACHE_Init()` is enabled and ICACHE caches flash reads on U5. After any program/erase, call `HAL_ICACHE_Invalidate()` before reading back, or read-back may return stale data even after writes start succeeding.
4. **Driver API swallows errors.** `nvs_flash_driver_t.write/erase` return `void`, so NVS reported `NVS_OK` on a failed write. At minimum, hard-fault/assert or log on HAL error inside the driver.
5. The debug `printf`s use `\n` only (BSP COM port wants `\n\r`) — that's why the log output staircases. Remove or fix them once writes work.

## 6. Recommended fix sequence

1. Revert driver write path to `HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, ...)` with the existing static `flash_write_buffer` (16-byte padded). Log `HAL_FLASH_GetError()` on failure.
2. Fix erase bank/page math (item 5.2) and add `HAL_ICACHE_Invalidate()` after program + erase.
3. Get user sign-off to change NVS granularity to 16 bytes (item 4) — two-line diff in `nvs.c`/`nvs.h`.
4. **Mass-erase the NVS region once** (ST-LINK or a one-shot erase at boot) — the region currently holds a stale 8-byte-format header at `0x08070000` that the new 16-byte layout would misparse.
5. Re-run the `test_task` round-trip; expect `NVS read result: 0, len: 4` and matching bytes.
6. Remove debug printfs, remove the `serial_flash_mem` leftover module, and clean compile warnings (unused `args`/`step_count` in main.c, unused functions warned during NVS build).

## 7. Reference — flash geometry (STM32U545, 512 KB)

| | |
|---|---|
| Page size | 8 KB (`0x2000`) |
| Banks | 2 × 256 KB; Bank 2 starts at `0x08040000` |
| Program unit | 16-byte quadword, 16-byte aligned, once per erase cycle (ECC) |
| Error decode used here | `NSSR`: EOP=b0, OPERR=b1, PROGERR=b3, WRPERR=b4, PGAERR=b5, SIZERR=b6, PGSERR=b7, BSY=b16 |
| App usage | ~40 KB → pages 0–4 of Bank 1; NVS at last 64 KB (Bank 2 pages 24–31) — no overlap |
