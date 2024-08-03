/*
 * sys_io.h
 *
 */
#ifndef SRC_SYS_IO_H_
#define SRC_SYS_IO_H_

#include "types.h"
#include "xparameters.h"

#if 0
#define	XPAR_MIG_0_BASEADDR         XPAR_DDR_MEM_BASEADDR


#define DMA_ADC0_DATA_START			XPAR_MIG_0_BASEADDR+0x10000000
#define DMA_ADC1_DATA_START			XPAR_MIG_0_BASEADDR+0x14000000
#define DMA_ADC2_DATA_START			XPAR_MIG_0_BASEADDR+0x18000000
#define DMA_ADC3_DATA_START			XPAR_MIG_0_BASEADDR+0x1C000000
#define DDR_BXB_XY_DATA_START		XPAR_MIG_0_BASEADDR+0x20000000
#define DDR_PM_BASE					XPAR_MIG_0_BASEADDR+0x24000000
#endif


//#define XPAR_DDR_MEM_HIGHADDR		0x3FFFFFFFU
#define	FLASH_BOOT_MEM				XPAR_DDR_MEM_BASEADDR+0x38000000U //0x20000000

#define DDR_FLASH_WR_u8(addr, val)  (* (volatile unsigned char *) 	(addr + FLASH_BOOT_MEM)) = (val)	/*.. for 8bit format */
#define DDR_FLASH_RD_u8(addr)       (* (volatile unsigned char *) 	(addr + FLASH_BOOT_MEM))		/*.. for 8bit format */

#define Get_I_DDR4(addr)       		(* (volatile Xint32 *) 	(addr))
#define Set_I_DDR4(addr, val)  		(* (volatile Xint32 *) 	(addr)) = (val)


#endif
