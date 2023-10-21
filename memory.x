MEMORY
{
  /* Leave 16k for the default bootloader on the Feather M4 */
  FLASH (rx) : ORIGIN = 0x00000000 + 16K, LENGTH = 512K - 16K
  /* RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 192K */
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 191K
  PANDUMP: ORIGIN = 0x2002FC00, LENGTH = 1K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM) + LENGTH(PANDUMP);

_panic_dump_start = ORIGIN(PANDUMP);
_panic_dump_end   = ORIGIN(PANDUMP) + LENGTH(PANDUMP);