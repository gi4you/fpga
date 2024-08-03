/*
 * fpga_system.h
 *
 *  Created on: Jun 14, 2024
 *      Author: kha
 */

#ifndef SRC_FPGA_SYSTEM_H_
#define SRC_FPGA_SYSTEM_H_


#include "types.h"
#include "xparameters.h"

#define	XPAR_MIG_0_BASEADDR  XPAR_DDR_MEM_BASEADDR


#define	DDR3_FOFB_DATA_BASE	 0x10000000

#define	BASE_UT			DDR3_FOFB_DATA_BASE+0x00A00000
#define	BASE_VV			DDR3_FOFB_DATA_BASE+0x00C80000

#define	DDR3_RX_REG		DDR3_FOFB_DATA_BASE+0x00D00000
#define DDR2_CIO_BASE   DDR3_FOFB_DATA_BASE+0x00D80000
#define DDR_SA_BASE		DDR3_FOFB_DATA_BASE+0x00E00000
#define DDR_DATA_TX_BUF_BASE		DDR3_FOFB_DATA_BASE+0x00F00000


//DMA
#define DMA_ADC0_DATA_START			XPAR_MIG_0_BASEADDR+0x20000000
#define DMA_ADC1_DATA_START			XPAR_MIG_0_BASEADDR+0x24000000
#define DMA_ADC2_DATA_START			XPAR_MIG_0_BASEADDR+0x28000000
#define DMA_ADC3_DATA_START			XPAR_MIG_0_BASEADDR+0x2C000000

//#define DDR_BXB_XY_DATA_START		XPAR_MIG_0_BASEADDR+0x20000000
//#define DDR_PM_BASE					XPAR_MIG_0_BASEADDR+0x24000000



/* control registers for DDR memory */
typedef struct _DDR2_CONTROLs {
	DDR2_UI32BIT_REG	ioReg[262];			/* control Regs : 4 byte * 20 signals = 80 */
	DDR2_UI32BIT_REG	sysInfo[64];
	DDR2_UI32BIT_REG	trigEvent;
} DDR2_CONTROLs, *DFE_CONTROL_REGS;

//used flash
typedef struct {
	DDR2_UI32BIT_REG	ioReg[250];
} __attribute__((packed)) DDR_CtrlData_t;

extern DDR_CtrlData_t		DDR_CtrlData;


/* DDR-2 base address is Modbus base address *2 times */
/* 0x0E000000 + OFFSET */
#define DFE_CONTROL_REGS  	( DDR2_MEM_CAST(DFE_CONTROL_REGS) 	(DDR2_CIO_BASE + 0) )



#endif /* SRC_FPGA_SYSTEM_H_ */
