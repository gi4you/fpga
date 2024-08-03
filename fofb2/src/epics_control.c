/*
 * epics_control.c
 *
 *  Created on: Jun 14, 2024
 *      Author: kha
 */

#include "types.h"
#include "fpga_system.h"

#include "fofb.h"



int 	cmd_Void( int chn, int data);
int 	cmd_Copy( int chn, int data);


/*
 *
 */
int cmd_Void( int chn, int data)
{

	xil_printf("cmd_Void: %d, %d\r\n", chn, data);
	return(0);
}

/*
 *  EPICS control Memory Map commands.
 *	Mapping with EPICS Database
 *	BO
 *	AO
 */
int (*Epics_InterpreterPtr[320])(int, int) =
{
	&cmd_Void,
	&cmd_Void,
	&cmd_Void,
	&cmd_Void
};


/**/
int	EpicsCmdsInterpreter( int addr, int data)
{
	int	rv;
	{
		addr = addr & 0xFF;
		DFE_CONTROL_REGS->ioReg[addr] = data;		/* same structure with slow */
		//Epics_InterpreterPtr[addr](addr, data);
		//printf("CMD addr=%d, data=%d\r\n", addr, data);

		switch(addr) {
			case 0:
				Xil_Out32(UP_AXI + addr, data);
				break;

			default:
				//general registers for control
				Xil_Out32(UP_AXI + addr, data);
				//xil_printf("RX ADDR: %d, DATA: %d (0x%X)\r\n", addr, data);
				break;
		}
		return 0;
	}
}

