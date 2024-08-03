/*
 * pscEpics_sock_tx.c
 *
 *  Created on: Jun 14, 2024
 *      Author: kha
 */


#include <stdio.h>
#include <string.h>
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwipopts.h"
#include "xparameters.h"
#include "xbasic_types.h"
#include "time.h"

#include "task.h"
#include "fpga_system.h"
#include "pscEpics.h"
#include "types.h"

#include "fofb.h"



u16_t tx_port = PSC_TX_SOCKET_PORT;

static int lwip_writeall(int s, const void *data, int size);


static int
lwip_writeall(int s, const void *data, int size)
{
	int sent = 0;
	int ret;
	do {
		ret = write(s,data+sent, size-sent);
		if(ret<1)
			return ret;
		sent += ret;
	} while(sent<size);
	return size;
}


void pscEpicsProcess_tx_request(void *p)
{
	int sd = (int)p;
	int totalSendByte=0;
	int	i;
	int N;
	Xuint32 pVoutData[64];

	static volatile  int *pDdrRegTxAddr;
	pDdrRegTxAddr = ( int *) (DDR_SA_BASE);

	static volatile  int *pDdrBufTxAddr;
	pDdrBufTxAddr = ( int *) (DDR_DATA_TX_BUF_BASE);


	//////////////////////////////////
	/// 10Hz data tx to IOC
	//////////////////////////////////
	while (1)
	{
		vTaskDelay(portTICK_PERIOD_MS);

		N=124;
		for(i=0; i<121; i++) {
			DDR_SLOW->tx[i] = htonl (Xil_In32(UP_AXI+(i*4)) );
		}
		DDR_SLOW->tx[121] = htonl (FOFB_DATA->err_cnt);  //reference ram error
		DDR_SLOW->tx[122] = htonl (DDR_UT->err_cnt);
		DDR_SLOW->tx[123] = htonl (DDR_VV->err_cnt);
		//
		totalSendByte = N*4;
		DDR_SLOW->head.idA     = 'P';
		DDR_SLOW->head.idB     = 'S';
		DDR_SLOW->head.MsgId   =  htons (TX_REG_MSG_ID);
		DDR_SLOW->head.bodyLen =  htonl (totalSendByte);

		//send DDR DATA to epics IOC
		totalSendByte += HEAD_SIZE;
		if (lwip_writeall(sd, pDdrRegTxAddr, totalSendByte) != totalSendByte) {
			xil_printf("Closing 10 Hz TX socket place 1 %d\n\r", sd);
			break;
		}
		//
		//VOUT reading
		/*
		 * [124] PS OUT:Head
		 * [125] PS_OUT:0
		 */
		N=45;
		fofb_VOUT_RAMRead(0, N, &DDR_SLOW->tx[0]);
		totalSendByte = N*4;
		DDR_SLOW->head.idA     = 'P';
		DDR_SLOW->head.idB     = 'S';
		DDR_SLOW->head.MsgId   =  htons (TX_REG_MSG_ID+1);
		DDR_SLOW->head.bodyLen =  htonl (totalSendByte);

		//send DDR DATA to epics IOC
		totalSendByte += HEAD_SIZE;
		if (lwip_writeall(sd, pDdrRegTxAddr, totalSendByte) != totalSendByte) {
			xil_printf("Closing 10 Hz TX socket place 1 %d\n\r", sd);
			break;
		}

		/////////////////////////////////
		//SDI Link data send
		N = 782;
		fofb_SDI_DataLinkRead(1, 781, &DDR_SLOW->tx[0]);
		totalSendByte = N*4;
		DDR_SLOW->head.idA     = 'P';
		DDR_SLOW->head.idB     = 'S';
		DDR_SLOW->head.MsgId   =  htons (TX_REG_MSG_ID+2);
		DDR_SLOW->head.bodyLen =  htonl (totalSendByte);

		//send DDR DATA to epics IOC
		totalSendByte += HEAD_SIZE;
		if (lwip_writeall(sd, pDdrRegTxAddr, totalSendByte) != totalSendByte) {
			xil_printf("Closing 10 Hz TX socket place 1 %d\n\r", sd);
			break;
		}

#if 0
		/////////////////////////////////
		// 1. orbit error update
		fofb_F32_ErrorOrbitRead (1, BPM_XY_SIZE, &DDR_DATA_TX_BUF->tx_f32data[0]);
		totalSendByte = BPM_XY_SIZE*4;
		DDR_DATA_TX_BUF->head.idA     = 'P';
		DDR_DATA_TX_BUF->head.idB     = 'S';
		DDR_DATA_TX_BUF->head.MsgId   =  htons (RX_ORBIT_ERR_WFM_MSG_ID);
		DDR_DATA_TX_BUF->head.bodyLen =  htonl (totalSendByte);
		//send DDR DATA to epics IOC
		totalSendByte += HEAD_SIZE;
		if (lwip_writeall(sd, pDdrBufTxAddr, totalSendByte) != totalSendByte) {
			xil_printf("Closing 10 Hz TX socket place 1 %d\n\r", sd);
			break;
		}

		/////////////////////////////////
		// 2. eigen0,1,2 calculation result send every 10 Hz
		fofb_EigenRegFloatRead (1, V_PRAM, PS_HV_SIZE, &DDR_DATA_TX_BUF->tx_f32data[0]);
		totalSendByte = PS_HV_SIZE*4;
		DDR_DATA_TX_BUF->head.idA     = 'P';
		DDR_DATA_TX_BUF->head.idB     = 'S';
		DDR_DATA_TX_BUF->head.MsgId   =  htons (RX_EIGEN0_WFM_MSG_ID);
		DDR_DATA_TX_BUF->head.bodyLen =  htonl (totalSendByte);
		//send DDR DATA to epics IOC
		totalSendByte += HEAD_SIZE;
		if (lwip_writeall(sd, pDdrBufTxAddr, totalSendByte) != totalSendByte) {
			xil_printf("Closing 10 Hz TX socket place 1 %d\n\r", sd);
			break;
		}

		/////////////////////////////////
		fofb_EigenRegFloatRead (1, V_IRAM, PS_HV_SIZE, &DDR_DATA_TX_BUF->tx_f32data[0]);
		totalSendByte = PS_HV_SIZE*4;
		DDR_DATA_TX_BUF->head.idA     = 'P';
		DDR_DATA_TX_BUF->head.idB     = 'S';
		DDR_DATA_TX_BUF->head.MsgId   =  htons (RX_EIGEN1_WFM_MSG_ID);
		DDR_DATA_TX_BUF->head.bodyLen =  htonl (totalSendByte);
		//send DDR DATA to epics IOC
		totalSendByte += HEAD_SIZE;
		if (lwip_writeall(sd, pDdrBufTxAddr, totalSendByte) != totalSendByte) {
			xil_printf("Closing 10 Hz TX socket place 1 %d\n\r", sd);
			break;
		}

		/////////////////////////////////
		fofb_EigenRegFloatRead (1, V_DRAM, PS_HV_SIZE, &DDR_DATA_TX_BUF->tx_f32data[0]);
		totalSendByte = PS_HV_SIZE*4;
		DDR_DATA_TX_BUF->head.idA     = 'P';
		DDR_DATA_TX_BUF->head.idB     = 'S';
		DDR_DATA_TX_BUF->head.MsgId   =  htons (RX_EIGEN2_WFM_MSG_ID);
		DDR_DATA_TX_BUF->head.bodyLen =  htonl (totalSendByte);
		//send DDR DATA to epics IOC
		totalSendByte += HEAD_SIZE;
		if (lwip_writeall(sd, pDdrBufTxAddr, totalSendByte) != totalSendByte) {
			xil_printf("Closing 10 Hz TX socket place 1 %d\n\r", sd);
			break;
		}
#endif


		//////
	}

	close(sd);
	xil_printf("----- Closing Tx socket place 2 %d -----\n\r", sd);


}
/*
 *
 */
/*
 *
 */
void EpicsTx_thread()
{
	int sock, new_sd;
	struct sockaddr_in address, remote;
	int size;

	xil_printf("\r\n**********************************************************\n\r");
	    xil_printf("** EpicsTx_thread TCP/IP communication thread port  **\n\r");
	    xil_printf("**********************************************************\n\r");


	/* Setup Socket: create */
	if ((sock = lwip_socket(AF_INET, SOCK_STREAM, 0)) < 0)
		return;

	address.sin_family = AF_INET;
	address.sin_port = htons(tx_port);
	address.sin_addr.s_addr = INADDR_ANY;

	/* Setup Socket: bind */
	if (lwip_bind(sock, (struct sockaddr *)&address, sizeof (address)) < 0)
		return;
	lwip_listen(sock, 5);
	size = sizeof(remote);

	/* accept */
#if 0
	while (1) {
		xil_printf("pscEpicsTx_server for 10 Hz...accepted port %d\r\n", tx_port );
		if ((new_sd = lwip_accept(sock, (struct sockaddr *)&remote, (socklen_t *)&size)) > 0)
			if(!sys_thread_new("pscEpicsTx_server", pscEpicsProcess_tx_request,(void*) new_sd, THREAD_STACKSIZE, DEFAULT_THREAD_PRIO)) {
				xil_printf("Failed to start pscEpicsProcess_tx_request thread...\r\n");
			}
	}
#else
	//No sys_thread_new, call  pscEpicsProcess_tx_request() function
	while (1) {
		xil_printf("pscEpicsTx_server for 10 Hz...accepted port %d\r\n", tx_port );
		if ((new_sd = lwip_accept(sock, (struct sockaddr *)&remote, (socklen_t *)&size)) > 0) {
			pscEpicsProcess_tx_request((void*) new_sd);
		}
	}

#endif
}
