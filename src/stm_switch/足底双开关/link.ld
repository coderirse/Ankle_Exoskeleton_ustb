MEMORY { FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K }
SECTIONS {
    .vectors : { KEEP(*(.vectors)) } > FLASH
    .text : { *(.text*) } > FLASH
}
