/* Linker memory layout for the STM32H743xx (2 MB flash, dual-bank).
 *
 * IMPORTANT: this layout assumes the firmware OWNS the whole chip and is
 * flashed at the very start of flash (0x0800_0000) via SWD or the STM32
 * system DFU bootloader. It deliberately does NOT coexist with the original
 * ArduPilot ChibiOS bootloader (which expects the application at an offset,
 * typically 0x0802_0000). See README.md -> "Flashing" for the two paths.
 *
 * RAM is mapped to DTCM (tightly-coupled, zero-wait-state) for fast, jitter
 * free real-time code. DTCM is NOT reachable by the DMA/MDMA engines; the
 * current firmware does no DMA, so this is fine. If you later add DMA-based
 * SPI, move buffers into AXISRAM instead.
 */
MEMORY
{
    FLASH   : ORIGIN = 0x08000000, LENGTH = 2048K  /* on-chip flash, both banks */
    RAM     : ORIGIN = 0x20000000, LENGTH = 128K   /* DTCM  - main RAM / stack   */
    AXISRAM : ORIGIN = 0x24000000, LENGTH = 512K   /* AXI SRAM (DMA-capable)     */
    SRAM1   : ORIGIN = 0x30000000, LENGTH = 128K
    SRAM4   : ORIGIN = 0x38000000, LENGTH = 64K
}

/* Stack lives at the top of DTCM. */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
