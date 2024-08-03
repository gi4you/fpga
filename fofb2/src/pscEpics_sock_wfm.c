/*
 * thread_Epics_sock_wfm.c
 *
 *  Created on: Jun 15, 2024
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
#include "xil_io.h"
#include "task.h"
#include "timers.h"

#include "xil_cache.h"


#include "fpga_system.h"
#include "pscEpics.h"
#include "fofb.h"
#include "sys_io.h"


static u16_t tx_port = 2000;

static int
lwip_writeall(int s, const void *data, int size)
{
	int sent = 0;
	int ret;
	do {
		ret = lwip_write(s,data+sent, size-sent);
		if(ret<1) {
			xil_printf("TCP socket write error.\r\n");
			return ret;
		}
		sent += ret;
		//xil_printf("sent=%d\r\n", ret);
	} while(sent<size);
	return size;
}


void pscEpicsProcess_wfm_tx_request(void *p)
{
	int i;
	int start_addr;
	int sd = (int)p;
	int totalSendByte;
	Xuint32 hex, d2;
	u32 prev_trig_sec, trig_sec;
	float f;
	char	txHeadBuf[64];
	psc_TxHead_t	*psc_TxHead;
	int bytes_sent;
	int dma2_cnt;
	int cnt;
	//
	static volatile  int *pDdrBufTxAddr;
	pDdrBufTxAddr = ( int *) (DDR_DATA_TX_BUF_BASE);

	//DMA
	static volatile  char *pDdr0TxAddr;
	static volatile  char *pDdr1TxAddr;
	static volatile  char *pDdr2TxAddr;

	pDdr0TxAddr = ( char *) (DMA_ADC0_DATA_START);
	pDdr1TxAddr = ( char *) (DMA_ADC1_DATA_START);
	pDdr2TxAddr = ( char *) (DMA_ADC2_DATA_START);

	prev_trig_sec = 0;

	while (1)
	{
		vTaskDelay(1000/portTICK_PERIOD_MS);

		////////////////////////////////
		// Reference RAM read
		if(FOFB_DATA->received > 0) {
			totalSendByte = BPM_XY_SIZE*4;
			DDR_INT_DATA_TX_BUF->head.idA     = 'P';
			DDR_INT_DATA_TX_BUF->head.idB     = 'S';
			DDR_INT_DATA_TX_BUF->head.MsgId   =  htons (RX_REF_READ_WFM_MSG_ID);
			DDR_INT_DATA_TX_BUF->head.bodyLen =  htonl (totalSendByte);
			if(FOFB_DATA->received == 1)start_addr = 0;
			else
				start_addr = ACTIVE_RAM_BLOCK_SIZE;

			fofb_ReferenceOrbitRead(start_addr, BPM_XY_SIZE, &DDR_INT_DATA_TX_BUF->tx_data[0]);
			//send DDR DATA to epics IOC
			totalSendByte += HEAD_SIZE;
			if (lwip_writeall(sd, pDdrBufTxAddr, totalSendByte) != totalSendByte) {
				xil_printf("Closing 10 Hz TX socket place 1 %d\n\r", sd);
				break;
			}
			FOFB_DATA->received = 0; //clear
			xil_printf("Orbit reference Readback ... \n\r");
		}


		//VRAM read
		if(DDR_VV->received > 0) {
			//fofb_V_VectorRamRead(0, char Mode, int v_ram_block_i, int len, Xuint32 *pData);

		}

		///////////////////////////////////////////////
		//read trigger time stamp
		trig_sec = Xil_In32(UP_AXI + (FOFB_REG_TRIG_SEC_TS_READ*4));
		if(trig_sec != prev_trig_sec)
		{
			cnt=0;
			//Read DMA data and send to IOC
			xil_printf("Wait for DMA data update...\n\r");
			// Check DMA update count.
			do {
				dma2_cnt = Xil_In32(UP_AXI + (FOFB_REG_DMA2_COUNT*4));
				vTaskDelay(100/portTICK_PERIOD_MS);
				cnt++;
				xil_printf(".\n\r");
			}while (dma2_cnt < (DMA_SAMPLE_SIZE/2) && cnt < 10);
			xil_printf("Send DMA data...\n\r");

			/////
			totalSendByte = 360*4*DMA_DATA_SEND_SIZE;
			psc_TxHead = (psc_TxHead_t *)txHeadBuf;
			psc_TxHead->idA     = 'P';
			psc_TxHead->idB     = 'S';
			psc_TxHead->MsgId   =  htons(RX_DMA0_EIGEN_WFM_MSG_ID);
			psc_TxHead->bodyLen =  htonl(totalSendByte);
			// send HEAD to epics IOC
			if ((bytes_sent = lwip_write(sd, txHeadBuf, HEAD_SIZE)) != HEAD_SIZE) {
				printf("%s: ERROR rcvd = %d, written = %d\n",__FUNCTION__, bytes_sent, HEAD_SIZE);
				break;
			}
			if (lwip_writeall(sd, &pDdr0TxAddr, totalSendByte) != totalSendByte) {
				printf("Closing DMA0 Wfm TX socket place 1, %d\n\r", sd);
				break;
			}

			//////
			totalSendByte = 460*4*DMA_DATA_SEND_SIZE;
			psc_TxHead = (psc_TxHead_t *)txHeadBuf;
			psc_TxHead->idA     = 'P';
			psc_TxHead->idB     = 'S';
			psc_TxHead->MsgId   =  htons(RX_DMA1_VOUT_WFM_MSG_ID);
			psc_TxHead->bodyLen =  htonl(totalSendByte);
			// send HEAD to epics IOC
			if ((bytes_sent = lwip_write(sd, txHeadBuf, HEAD_SIZE)) != HEAD_SIZE) {
				printf("%s: ERROR rcvd = %d, written = %d\n",__FUNCTION__, bytes_sent, HEAD_SIZE);
				break;
			}
			if (lwip_writeall(sd, &pDdr1TxAddr, totalSendByte) != totalSendByte) {
				printf("Closing DMA1 Wfm TX socket place 1, %d\n\r", sd);
				break;
			}

			//////
			totalSendByte = 480*4*DMA_DATA_SEND_SIZE;
			psc_TxHead = (psc_TxHead_t *)txHeadBuf;
			psc_TxHead->idA     = 'P';
			psc_TxHead->idB     = 'S';
			psc_TxHead->MsgId   =  htons(RX_DMA2_POSXY_WFM_MSG_ID);
			psc_TxHead->bodyLen =  htonl(totalSendByte);
			// send HEAD to epics IOC
			if ((bytes_sent = lwip_write(sd, txHeadBuf, HEAD_SIZE)) != HEAD_SIZE) {
				printf("%s: ERROR rcvd = %d, written = %d\n",__FUNCTION__, bytes_sent, HEAD_SIZE);
				break;
			}
			if (lwip_writeall(sd, &pDdr2TxAddr, totalSendByte) != totalSendByte) {
				printf("Closing DMA2 Wfm TX socket place 1, %d\n\r", sd);
				break;
			}

			//////////
			prev_trig_sec = trig_sec;
		}
		else
		{
	#if 1
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

			//
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



			//// Eigen calculation test //////////////
			for(i=0; i<180; i++) {
				//hex = Float2DWordReg(DDR_UT->f32Ut_Eigen_Pdata[i]);  //ARM CPU
				hex = Float2DWordReg(DDR_UT->f32Ut_Eigen_PdataCal[i]); //Python client

				d2 = htonl(hex);   //to IOC
				f = *(float *)&d2;
				DDR_DATA_TX_BUF->tx_f32data[i] = f;
			}
			totalSendByte = PS_HV_SIZE*4;
			DDR_DATA_TX_BUF->head.idA     = 'P';
			DDR_DATA_TX_BUF->head.idB     = 'S';
			DDR_DATA_TX_BUF->head.MsgId   =  htons (RX_SIM_EIGEN0_WFM_MSG_ID);
			DDR_DATA_TX_BUF->head.bodyLen =  htonl (totalSendByte);
			//send DDR DATA to epics IOC
			totalSendByte += HEAD_SIZE;
			if (lwip_writeall(sd, pDdrBufTxAddr, totalSendByte) != totalSendByte) {
				xil_printf("Closing WFM TX socket place 1 %d\n\r", sd);
				break;
			}
			/////

		}

	}


__end:
	close(sd);
	xil_printf("----- Closing WFM Tx socket %d -----\n\r", sd);
}




/*
 *
 */
void EpicsWfm_thread()
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
		xil_printf("pscEpicsTx_server for Waveform ...accepted port %d\r\n", tx_port );
		if ((new_sd = lwip_accept(sock, (struct sockaddr *)&remote, (socklen_t *)&size)) > 0)
			if(!sys_thread_new("pscEpicsTx_server", pscEpicsProcess_wfm_tx_request,(void*) new_sd, THREAD_STACKSIZE, DEFAULT_THREAD_PRIO)) {
				xil_printf("Failed to start pscEpicsProcess_tx_request thread...\r\n");
			}
	}
#else
	while (1) {
		xil_printf("pscEpicsTx_server for Waveform ...accepted port %d\r\n", tx_port );
		if ((new_sd = lwip_accept(sock, (struct sockaddr *)&remote, (socklen_t *)&size)) > 0) {
			pscEpicsProcess_wfm_tx_request((void*) new_sd);
		}
	}
#endif
}
