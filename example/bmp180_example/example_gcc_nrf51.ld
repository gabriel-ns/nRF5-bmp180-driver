/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  UNPLACED_SECTIONS (wx) : ORIGIN = 0x100000000, LENGTH = 0
  RAM (wx) : ORIGIN = 0x20001fe8, LENGTH = 0x00002018
  FLASH (rx) : ORIGIN = 0x0001b000, LENGTH = 0x00025000
}

SECTIONS
{
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "nrf5x_common.ld"