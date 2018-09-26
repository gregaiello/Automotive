/*
 * GENERATED FILE - DO NOT EDIT
 * (c) Code Red Technologies Ltd, 2008-2013
 * (c) NXP Semiconductors 2013-2018
 * Generated linker script file for LPC11C24/301
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.23
 * MCUXpresso IDE v10.1.1 [Build 606] [2018-01-02] on Sep 26, 2018 9:03:09 PM
 */

MEMORY
{
  /* Define each memory region */
  MFlash32 (rx) : ORIGIN = 0x0, LENGTH = 0x8000 /* 32K bytes (alias Flash) */  
  RamLoc8 (rwx) : ORIGIN = 0x10000100, LENGTH = 0x1f00 /* 7936 bytes (alias RAM) */  
  RAM_00 (rwx) : ORIGIN = 0x10000000, LENGTH = 0x100 /* 256 bytes (alias RAM2) */  
}

  /* Define a symbol for the top of each memory region */
  __base_MFlash32 = 0x0  ; /* MFlash32 */  
  __base_Flash = 0x0 ; /* Flash */  
  __top_MFlash32 = 0x0 + 0x8000 ; /* 32K bytes */  
  __top_Flash = 0x0 + 0x8000 ; /* 32K bytes */  
  __base_RamLoc8 = 0x10000100  ; /* RamLoc8 */  
  __base_RAM = 0x10000100 ; /* RAM */  
  __top_RamLoc8 = 0x10000100 + 0x1f00 ; /* 7936 bytes */  
  __top_RAM = 0x10000100 + 0x1f00 ; /* 7936 bytes */  
  __base_RAM_00 = 0x10000000  ; /* RAM_00 */  
  __base_RAM2 = 0x10000000 ; /* RAM2 */  
  __top_RAM_00 = 0x10000000 + 0x100 ; /* 256 bytes */  
  __top_RAM2 = 0x10000000 + 0x100 ; /* 256 bytes */  
