/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00027000, LENGTH = 0x00100000 - 0x00027000
  RAM  (rwx) : ORIGIN = 0x20002500, LENGTH = 0x20040000 - 0x20002500
}

SECTIONS
{
  . = ALIGN(4);
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "nrf5x_common.ld"
