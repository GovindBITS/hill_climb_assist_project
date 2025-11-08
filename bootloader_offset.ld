/* bootloader_offset.ld */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08002000, LENGTH = 512K - 0x2000
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K
}
