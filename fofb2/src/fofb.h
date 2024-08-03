/*
 * fofb.h
 *
 *  Created on: Jun 8, 2024
 *      Author: kha
 */

#ifndef SRC_FOFB_H_
#define SRC_FOFB_H_

#include "xparameters.h"
#include "xil_io.h"
#include "xbasic_types.h"
//#include "xil_types.h"

#include "fpga_system.h"
#include "types.h"
#include "pscEpics.h"


//XPAR_UP_AXI_0_BASEADDR
#define	UP_AXI			XPAR_UP_AXI_0_BASEADDR


#define	PS_SIZE			90

//#define	PS_HV_SIZE		180
#define	PS_HV_SIZE 		180
#define	PS_PER_CELL 	6


#define	BPM_SIZE		240
#define	BPM_XY_SIZE		480
#define	V_XY_SIZE		540
#define	UT_XY_SIZE		(PS_HV_SIZE * BPM_XY_SIZE)

#define	DMA_SAMPLE_SIZE		50000		//DMA sample size
#define	DMA_DATA_SEND_SIZE	5000		//to IOC

//0, 512(0x200), 1024(0x400), 1536(0x600)
#define	UT_BRAM_PAGE_OFFSET		512


#define	V_PRAM	0
#define	V_IRAM	1
#define	V_DRAM	2

#define	ACTIVE_RAM0		0
#define	ACTIVE_RAM1		1

// Register map for controls
#define CMDR_SDI_RAM_CTRL         2
#define CMDR_UT_RAM_ENABLE        5
#define CMDR_UT_RAM_WR            6
#define	CMDR_UT_RAM_ADDR          7
#define	CMDR_UT_RAM_DATA          8
#define	CMDR_POS_ERROR_UNIT_DATA  9
#define	CMDR_POS_REF_ADDR         10
#define	CMDR_POS_REF_DATA         11
#define CMDR_UT_RDLY              12
#define CMDR_UT_CAL_LENGTH        13
#define CMDR_UT_SGAIN             14
#define CMDR_V_RAM_ADDR           15
#define CMDR_V_RAM_DATA           16
#define CMDR_V_P_CTRL             17
#define CMDR_V_I_CTRL             18
#define CMDR_V_D_CTRL             19
#define CMDR_V_CAL_DELAY          20
#define CMDR_PS_OUT_DELAY         21
#define CMDR_PS_PID_SUM_GAIN      22
#define CMDR_UT_RAM_BLK_IDX       23
#define CMDR_V_RAM_BLK_IDX        24
#define CMDR_F32_PS_SET           25
#define CMDR_VM_SEL               26
#define CMDR_VOUT_RAM_ADDR        27

#define CMDR_PS_AMP2DAC_GAIN      28
#define CMDR_NCO_FREQ             29
#define CMDR_NCO_GAIN             30
#define CMDR_DAM0_SAMPLE_SIZE     31
#define CMDR_DAM1_SAMPLE_SIZE     32
#define CMDR_DAM2_SAMPLE_SIZE     33

#define CMDR_USR_POS_ERR_LMT_SET  41


//READ Registers
#define	FOFB_REG_REF_ORBIT_READ			60
#define	FOFB_REG_POS_ERR_FLOAT_READ		61
#define	FOFB_REG_UT_DPRAM_DATA_READ		62
#define	FOFB_REG_EIGEN_DATA_READ		63
#define	FOFB_REG_SDILINK_DATA_READ		65
#define	FOFB_REG_V_P_DPRAM_DATA_READ	66
#define	FOFB_REG_V_I_DPRAM_DATA_READ	67
#define	FOFB_REG_V_D_DPRAM_DATA_READ	68
#define	FOFB_REG_VOUT_READ				69


#define	FOFB_REG_DMA2_COUNT				72
#define	FOFB_REG_TRIG_SEC_TS_READ		76
#define	FOFB_REG_TRIG_NSEC_TS_READ		77

#define	FOFB_REG_EIGEN_SUMI_DATA_READ	81
#define	FOFB_REG_EIGEN_SUMD_DATA_READ	82


#define	FOFB_REG_FIRMWARE_VER_READ		120




#define	ACTIVE_RAM_BLOCK_SIZE		500

/*
 * Copy from V6
 */

typedef struct _FOFB_BUFFERs {
	int			HV_RefOrbit[BPM_XY_SIZE];  //USE int not Xint32
	int			HV2_RefOrbit[BPM_XY_SIZE];
	int			H_RefOrbit[BPM_SIZE], V_RefOrbit[BPM_SIZE];
	int			H2_RefOrbit[BPM_SIZE], V2_RefOrbit[BPM_SIZE];
	int			HV_RefOrbit_RD[BPM_XY_SIZE];  //nm
	int			SDI_LinkData_RD[785];
	//int			H_OrbitMask[BPM_SIZE], V_OrbitMask[BPM_SIZE];
	//
	//int			H_Kp[PS_SIZE], V_Kp[PS_SIZE];
	//int			H_Ki[PS_SIZE], V_Ki[PS_SIZE];
	//int			H_Kd[PS_SIZE], V_Kd[PS_SIZE];
	//
	//int			V[12][PS_SIZE*3];
	//int			V1_x[PS_SIZE*12];
	//int			V1_y[PS_SIZE*12];
	//int			Xy_Pos_SimData[1024];
	int			err_cnt;
	int			received;
//} __attribute__((packed)) FofbPackage_t;
} FOFB_BUFFERs, *FOFB_DATA;

#define FOFB_DATA  	( DDR2_MEM_CAST(FOFB_DATA) 	(DDR3_FOFB_DATA_BASE + 0) )

//Ut memory
typedef struct __FOFB_UT_BUFFERs {
	Xuint32		data[PS_HV_SIZE][BPM_XY_SIZE];
	float		f32SetData[PS_HV_SIZE][BPM_XY_SIZE];
	float		f32ReadData[PS_HV_SIZE][BPM_XY_SIZE];
	float		f32Ut_Eigen_Pdata[PS_HV_SIZE];
	float		f32Ut_Eigen_PdataCal[PS_HV_SIZE];
	int			ram_wr_flag;
	int			ram_wr_cnt;
	int			err_cnt;
	int			received;
}	 _FOFB_UT_BUFFER, *DDR_UT;
#define DDR_UT  	( DDR2_MEM_CAST(DDR_UT) 	(BASE_UT) )

typedef struct __FOFB_V_BUFFERs {
	Xuint32		pdata[PS_PER_CELL][PS_HV_SIZE];
	Xuint32		idata[PS_PER_CELL][PS_HV_SIZE];
	Xuint32		ddata[PS_PER_CELL][PS_HV_SIZE];
	float		pfdata[PS_PER_CELL][PS_HV_SIZE];
	float		pfSumOutdata[PS_PER_CELL*12];
	//
	Xuint32		pdata2[PS_PER_CELL][PS_HV_SIZE];
	Xuint32		idata2[PS_PER_CELL][PS_HV_SIZE];
	Xuint32		ddata2[PS_PER_CELL][PS_HV_SIZE];
	int			v_ram_wr_flag;
	int			err_cnt;
	int			ram_wr_cnt;
	int			received;
}	 _FOFB_V_BUFFER, *DDR_VV;
#define DDR_VV  	( DDR2_MEM_CAST(DDR_VV) 	(BASE_VV) )


typedef struct EPICS_DDR_SLOW_MEM {
	psc_TxHead_t head;
	int 	   tx[1024];
} DDR_SLOW_MEM, *DDR_SLOW_PTR;
#define DDR_SLOW     ( DDR2_MEM_CAST(DDR_SLOW_PTR) 		(DDR_SA_BASE ) )


typedef struct EPICS_DDR_BUF_MEM {
	psc_TxHead_t head;
	float 	   tx_f32data[1024];
} EPICS_DDR_BUF_MEM, *EPICS_DDR_BUF_MEM_PTR;
#define DDR_DATA_TX_BUF     ( DDR2_MEM_CAST(EPICS_DDR_BUF_MEM_PTR) 		(DDR_DATA_TX_BUF_BASE ) )

typedef struct EPICS_DDR_INT_BUF_MEM {
	psc_TxHead_t head;
	int 	   tx_data[1024];
} EPICS_DDR_INT_BUF_MEM, *EPICS_DDR_INT_BUF_MEM_PTR;
#define DDR_INT_DATA_TX_BUF     ( DDR2_MEM_CAST(EPICS_DDR_INT_BUF_MEM_PTR) 	(DDR_DATA_TX_BUF_BASE ) )




Xuint32	Float2DWordReg (float val);

void fofb_init();

void fofb_V_PID_SumGain(float gain);
void fofb_U_I_StaticGain(float gain);

int  fofb_ReferenceOrbitWrite(int start_addr, int end_addr, int *pData);
int  fofb_ReferenceOrbitRead(int start_addr, int len, int *pData);
void fofb_F32_ErrorOrbitRead(int dispMode, int len, float *pData);
void fofb_UtVectorInt32DataWrite(int uRamBlock, int len, int *pData);
void fofb_IOC_UtVectorInt32DataWrite(int uRamBlock, int len, int *pData);
int  fofb_UtRAMInt32Read(int dispMode, int uRamBlock, int len, float *pData);

void fofb_V_VectorWrite(int activeRam, char Mode, int vRamBlock, int start_addr, int MaxAddr, int *pData);
void fofb_IOC_V_VectorWrite(int activeRam, char Mode, int vRamBlock, int MaxAddr, int *pData);
int	 fofb_V_VectorRamRead(int activeRAM, char Mode, int vRamBlock, int len, Xuint32 *pData);
void fofb_EigenRegFloatRead(int type, int mode, int len, float *pData);
void fofb_IOC_F32_SET(int addr, float fdata);
int fofb_SDI_DataLinkRead(int start_addr, int len, int *pData);

#endif /* SRC_FOFB_H_ */
