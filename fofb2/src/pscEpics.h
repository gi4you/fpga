/*
 * pscEpics.h
 *
 *  Created on: Jun 14, 2024
 *      Author: kha
 */

#ifndef SRC_PSCEPICS_H_
#define SRC_PSCEPICS_H_


#include "fpga_system.h"

#define	THREAD_STACKSIZE			1024


#define	HEAD_SIZE					8


#define	PSC_RX_SOCKET_PORT			7
#define	PSC_TX_SOCKET_PORT			20

#define	TX_REG_MSG_ID				30

// epics interface for received waveform from IOC
#define	RX_REF_READ_WFM_MSG_ID		52
#define	RX_ORBIT_ERR_WFM_MSG_ID		54
#define	RX_EIGEN0_WFM_MSG_ID		55
#define	RX_EIGEN1_WFM_MSG_ID		56
#define	RX_EIGEN2_WFM_MSG_ID		57

#define	RX_SIM_EIGEN0_WFM_MSG_ID	58

#define	RX_H_REF_WFM_MSG_ID			70		// H reference orbit
#define	RX_V_REF_WFM_MSG_ID			71		// V reference orbit
#define	RX_UT_WFM_VECTOR_MSG_ID		72		// Ut
#define	RX_VP_WFM_VECTOR_MSG_ID		73		// V
#define	RX_VI_WFM_VECTOR_MSG_ID		74
#define	RX_VD_WFM_VECTOR_MSG_ID		75
#define	RX_VP2_WFM_VECTOR_MSG_ID	76		// V
#define	RX_VI2_WFM_VECTOR_MSG_ID	77
#define	RX_VD2_WFM_VECTOR_MSG_ID	78
#define	RX_H_REF2_WFM_MSG_ID		79		// H2 reference orbit
#define	RX_V_REF2_WFM_MSG_ID		80		// V2 reference orbit

#define	RX_EIGEN_P_CLIENT_CAL_WFM_MSG_ID	81


#define	RX_DMA0_EIGEN_WFM_MSG_ID	90
#define	RX_DMA1_VOUT_WFM_MSG_ID		91
#define	RX_DMA2_POSXY_WFM_MSG_ID	92

#define	RX_PM_WFM__MSG_ID			85



typedef struct {
    char  	idA;		//'P'
    char  	idB;		//'S'
    short 	MsgId;		// 1 ~ 65535
    int 	bodyLen;	// 8 ~
} __attribute__((packed)) psc_TxHead_t;


typedef struct _srvTxDatas
{
    char  	idA;
    char  	idB;
    short 	MsgId;
    int 	bodyLen;
    int		data[2048];
    int 	txLength;
} srvTxDatas, *pSRV_TXD_S;

#define pSrvTxDdrOffAddr  	( DDR2_MEM_CAST(pSRV_TXD_S) 	(DDR3_TX_REG-8) )


typedef struct {
    char  	idA;
    char  	idB;
    short 	MsgId;
    int 	bodyLen;
    int		data[1500];
} __attribute__((packed)) psc_RxBuffs_t;


#if 0
typedef struct _srvTxDdrs0
{
    char  	idA;
    char  	idB;
    short 	MsgId;
    int 	bodyLen;
    int		data[58720000];	//C20000000 - C2000000  / 4, 4 bytes
    int 	txLength;
} srvTxDdrs0, *pSRV_TXDDR_0;

#define DDR3_TX0  	( DDR2_MEM_CAST(pSRV_TXDDR_0) 	(DDR3_NPI0 ) )


typedef struct _srvTxDdrs1
{
    char  	idA;
    char  	idB;
    short 	MsgId;
    int 	bodyLen;
    int		data[58720000];	//0xD0000000
    int 	txLength;
} srvTxDdrs1, *pSRV_TXDDR_1;

//#define DDR3_TX1  	( DDR2_MEM_CAST(pSRV_TXDDR_1) 	(DDR3_NPI1 - 8) )
#define DDR3_TX1  	( DDR2_MEM_CAST(pSRV_TXDDR_1) 	(DDR3_NPI1) )
#endif


// RX Space
typedef struct _srvRxDatas
{
    //char  	idA;
    //char  	idB;
    //short 	MsgId;
    //int 	bodyLen;
    //int		data[64000];
    int		data[512000];
} srvRxDatas, *pSRV_RX_DATA;
#define pSrvRxDdrOffAddr  	( DDR2_MEM_CAST(pSRV_RX_DATA) 	(DDR3_RX_REG) )




int	EpicsCmdsInterpreter( int addr, int data);


#endif /* SRC_PSCEPICS_H_ */
