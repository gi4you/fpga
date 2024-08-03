/*
 * fofb.c
 *
 *  Created on: Jun 8, 2024
 *      Author: kha
 */


#include "fofb.h"
#include <stdio.h>

#include "lwip/def.h"



int	ut_data[UT_BRAM_PAGE_OFFSET*4];


typedef struct {
	Xuint32 addr 	: 10;		//[9:0]
	Xuint32 sp      : 1;        //[10]
	Xuint32 en   	: 1;        //[11]
	Xuint32 sp2 	: 20;       //[31:12]
    } w_reg2;
w_reg2	fofb_sdiLink_dpram;


typedef struct {
	Xuint32 addr 	: 10;		//[9:0]
	Xuint32 sp      : 19;
	Xuint32 en_pos_err_ram 	: 1;        //[29]
	Xuint32 en   	: 1;		//[30]
	Xuint32 wr 	    : 1;		//[31]
    } w_reg10;
w_reg10	fofb_ref_dpram;



typedef struct {
	Xuint32 addr 	: 10;		//[9:0]
	Xuint32 sp0     : 1;
	Xuint32 en      : 1;        //[11]
	Xuint32 wr      : 1;        //[12]
	Xuint32 data_en 	: 1;	//[13]
	Xuint32 data_wr 	: 1;    //[14]
	Xuint32 Ut_RamB_Out_enable 	: 1;    //[15]
	Xuint32 sp1	    : 16;		//[31:16]
    } w_reg5;
w_reg5	fofb_UT_RAM_EN;



typedef struct {
	Xuint32 addr 	: 8;		//[7:0]
	Xuint32 sp0     : 1;        //[8]
	Xuint32 en      : 1;        //[9]
	Xuint32 sp1	    : 22;		//[31:10]
    } w_reg27;
w_reg27	fofb_VOUT_RAM_ADDR;




float Ut_block[180][480];
/*
 *
 */
void fofb_init()
{
    xil_printf("FOFB register initialization ...\r\n");

    //clear error counters
    FOFB_DATA->err_cnt = 0;
    DDR_UT->err_cnt    = 0;
    DDR_VV->err_cnt    = 0;
    FOFB_DATA->received = 0;
    DDR_UT->received   = 0;
    DDR_VV->received   = 0;

    Xil_Out32(UP_AXI+(CMDR_POS_ERROR_UNIT_DATA*4),  0x3a83126f);  //0.001 um
    Xil_Out32(UP_AXI+(CMDR_UT_RDLY*4), 13);  //Ut read delay adjust
    Xil_Out32(UP_AXI+(CMDR_UT_CAL_LENGTH*4), 479); //Ut Cal length


    //Not used
    Xil_Out32(UP_AXI+(CMDR_V_CAL_DELAY*4), 529); //340+189);
    Xil_Out32(UP_AXI+(CMDR_PS_OUT_DELAY*4),750);  //PS final update to send to PS

    //error position limit = 200 um
    Xil_Out32(UP_AXI+(CMDR_USR_POS_ERR_LMT_SET*4), 500000);

    //Xil_Out32(UP_AXI+(CMDR_PS_PID_SUM_GAIN*4), 0x3a83126f);  //0.001
    //SG gain 1.0
    //Xil_Out32(UP_AXI+(CMDR_UT_SGAIN*4), 0x3f800000);

    fofb_V_PID_SumGain (1.0);
    fofb_U_I_StaticGain(0.1);


    //Reference orbit read/write and compare errors
    int i,j;
    uint32_t hex;
    ut_data[0] = 1;  //um
	for(i=0; i<BPM_XY_SIZE; i++) {
		hex = 0;
		ut_data[i+1] = hex;
	}
	ut_data[478] = 1;
	ut_data[479] = 2;
	// Reference orbit Write and Read : nm Unit
	fofb_ReferenceOrbitWrite(0, BPM_XY_SIZE, &ut_data[0]);  //OK

	//Ut RAM test
	float ut_sum[PS_HV_SIZE];

	//Ut Matrix download [0..189]
	for(i=0; i<PS_HV_SIZE; i++) {
		for(j=0; j<BPM_XY_SIZE; j++) {
#if 1
			Ut_block[i][j] = 0.0; //(float) (j+i);
			hex = Float2DWordReg(Ut_block[i][j]);
			ut_data[j] = hex;
#else
			ut_data[j] = j;  //for integer
#endif
			//printf("Ut%d = %X, %f\r\n", j, ut_data[j], fdata);
			//for manual test
		}
		fofb_UtVectorInt32DataWrite(i, BPM_XY_SIZE, &ut_data[0]); //OK
		fofb_UtRAMInt32Read(2, i, BPM_XY_SIZE, NULL);  //OK

		ut_sum[i] = 0.0;
	}
	printf("Ut RAM Write Done..\r\n");

#if 1
	//V DP RAM write Test
	float vram[PS_HV_SIZE];
	//float f1;
	//f1 = 0.01;
	for(i=0; i<PS_HV_SIZE; i++) {
		vram[i] = 0.0; //f1+(i/1000);
		hex = Float2DWordReg(vram[i]);
		ut_data[i] = hex;
	}
#if 0
/*
	for(i=10; i<170; i++) {
		hex = Float2DWordReg(0);
		ut_data[i] = hex;
	}
	*/
	j=0;
	for(i=170; i<180; i++) {
		j++;
		f1 = (float)(j+1);
		hex = Float2DWordReg(f1);
		ut_data[i] = hex;
	}
#endif

	//0
	for(i=0; i<PS_HV_SIZE; i++) {
		hex = Float2DWordReg(0);
		ut_data[200+i] = hex;
	}
	printf("I = %X\r\n", ut_data[200]);

	xil_printf("\r\n----- V-P RAM WRITE -----\r\n");
	//P
	fofb_V_VectorWrite(0, V_PRAM, 0,  0, PS_HV_SIZE, &ut_data[0]);
	fofb_V_VectorWrite(0, V_PRAM, 1,  0, PS_HV_SIZE, &ut_data[0]);
	fofb_V_VectorWrite(0, V_PRAM, 2,  0, PS_HV_SIZE, &ut_data[0]);
	fofb_V_VectorWrite(0, V_PRAM, 3,  0, PS_HV_SIZE, &ut_data[0]);
	fofb_V_VectorWrite(0, V_PRAM, 4,  0, PS_HV_SIZE, &ut_data[0]);
	fofb_V_VectorWrite(0, V_PRAM, 5,  0, PS_HV_SIZE, &ut_data[0]);
	//I
	fofb_V_VectorWrite(0, V_IRAM, 0,  0, PS_HV_SIZE, &ut_data[200]);
	fofb_V_VectorWrite(0, V_IRAM, 1,  0, PS_HV_SIZE, &ut_data[200]);
	fofb_V_VectorWrite(0, V_IRAM, 2,  0, PS_HV_SIZE, &ut_data[200]);
	fofb_V_VectorWrite(0, V_IRAM, 3,  0, PS_HV_SIZE, &ut_data[200]);
	fofb_V_VectorWrite(0, V_IRAM, 4,  0, PS_HV_SIZE, &ut_data[200]);
	fofb_V_VectorWrite(0, V_IRAM, 5,  0, PS_HV_SIZE, &ut_data[200]);
	//D
	fofb_V_VectorWrite(0, V_DRAM, 0,  0, PS_HV_SIZE, &ut_data[0]);
	fofb_V_VectorWrite(0, V_DRAM, 1,  0, PS_HV_SIZE, &ut_data[0]);
	fofb_V_VectorWrite(0, V_DRAM, 2,  0, PS_HV_SIZE, &ut_data[0]);
	fofb_V_VectorWrite(0, V_DRAM, 3,  0, PS_HV_SIZE, &ut_data[0]);
	fofb_V_VectorWrite(0, V_DRAM, 4,  0, PS_HV_SIZE, &ut_data[0]);
	fofb_V_VectorWrite(0, V_DRAM, 5,  0, PS_HV_SIZE, &ut_data[0]);
#endif
	printf("V RAM Write Done..\r\n");


	/////////////////////////////
    //UT Cal test
	float pErr[BPM_XY_SIZE];
	float fpga_eigen[180];
	fofb_F32_ErrorOrbitRead(0, BPM_XY_SIZE, &pErr[0]);

	fofb_EigenRegFloatRead(0, 0, 180, &fpga_eigen[0]);
	for(i=0; i<PS_HV_SIZE; i++) {
		for(j=0; j<BPM_XY_SIZE; j++) {
			ut_sum[i] += pErr[j] * Ut_block[i][j];
		}
		//0.001 error
		//printf("ut_sum [%d] = %.3f %.3f, %.3f\r\n", i, ut_sum[i], fpga_eigen[i], ut_sum[i] - fpga_eigen[i]);
	}
	//////

	float vsum[PS_PER_CELL];
	for(i=0; i<PS_PER_CELL; i++) vsum[i] = 0.0;
	for(i=0; i<PS_PER_CELL; i++) {
		for(j=0; j<PS_HV_SIZE; j++) {
			vsum[i] += ut_sum[j] * vram[j];
		}
		printf("V_sum [%d] = %.3f\r\n", i, vsum[i]);
	}

}


void Reg_Mask(u32 reg_base, u32 mask)
{
	*(volatile u32*) (UP_AXI + (reg_base*4)) = mask;
}


void fofb_Reg10_Mask(Xuint32 mask)
{
	*(volatile unsigned int*)(UP_AXI + (CMDR_POS_REF_ADDR*4)) = mask;
}

void fofb_DpRamDataWrite(Xint32 data)
{
	Xil_Out32(UP_AXI + (CMDR_POS_REF_DATA*4), data );
}

void fofb_Reg5_Mask(Xuint32 mask)
{
	*(volatile unsigned int*)(UP_AXI + (CMDR_UT_RAM_ENABLE*4)) = mask;
}
void fofb_Reg6_Mask(Xuint32 mask)
{
	*(volatile unsigned int*)(UP_AXI + (CMDR_UT_RAM_WR*4)) = mask;
}

void fofb_Reg27_Mask(Xuint32 mask)
{
	*(volatile unsigned int*)(UP_AXI + (CMDR_VOUT_RAM_ADDR*4)) = mask;
}

void fofb_Reg2_Mask(Xuint32 mask)
{
	*(volatile unsigned int*)(UP_AXI + (CMDR_SDI_RAM_CTRL*4)) = mask;
}


/*
 *	This function for epics floating point communication
 *	Microblaze Float -> Long    --------> EPICS Long -> Float
 */
Xuint32	Float2DWordReg (float val)
{
Xuint32* pRegValue;
//Xuint32 hex_val;

	pRegValue= (Xuint32*)(&val);
	//hex_val = *(pRegValue+0);
	//xil_printf("0x%X\r\n", hex_val);
	return *(pRegValue+0);
}

void fofb_IOC_F32_SET(int addr, float fdata)
{
	u32 hex;
	hex = Float2DWordReg(fdata);
	Xil_Out32(UP_AXI+(addr*4), hex);
}

//
void fofb_V_PID_SumGain(float gain)
{
	u32 hex;
	hex = Float2DWordReg(gain);
	Xil_Out32(UP_AXI+(CMDR_PS_PID_SUM_GAIN*4), hex);
}

void fofb_U_I_StaticGain(float gain)
{
	u32 hex;
	hex = Float2DWordReg(gain);
	Xil_Out32(UP_AXI+(CMDR_UT_SGAIN*4), hex);
}



void fofb_VOUT_RAMRead(int dispMode, int len, int *pData)
{
	int i;
	Xint32 data;
	float f;
	//int j=0;
	//int k=0;

	fofb_VOUT_RAM_ADDR.en = 1;
	fofb_Reg27_Mask( *((Xuint32 *)&fofb_VOUT_RAM_ADDR));
	//Reg_Mask(CMDR_VOUT_RAM_ADDR, *((Xuint32 *)&fofb_VOUT_RAM_ADDR));

	for(i=0; i<len; i++) {
		fofb_VOUT_RAM_ADDR.addr = i;
		fofb_Reg27_Mask( *((Xuint32 *)&fofb_VOUT_RAM_ADDR));
    	//
		data = Xil_In32(UP_AXI + (FOFB_REG_VOUT_READ*4));
		//*(pData+(i)) = data;   //to IOC
		//
		if(i < 7) {    //Int
			*(pData+(i)) = htonl(data);
		} else if (i>6 && i<37){ //float
			f = *(float *)&data;
			data = Float2DWordReg(f);
			*(pData+(i)) = htonl(data);
			DDR_VV->pfSumOutdata[i] = f;
		}
		else {
			*(pData+(i)) = htonl(data);
		}

		if(dispMode == 1) {
			if(i < 7) {
				printf("%3d = %X\r\n", i, (u32)data&0xFFFFF );
			} else if (i>6 && i<37){
				f = *(float *)&data;
				printf("%3d = %.3f\r\n", i, f );
			}
			else {
				printf("%3d = %X, %u\r\n", i, (u32)data, (u32)data );
			}
		}
	}

	fofb_VOUT_RAM_ADDR.en = 0;
	fofb_Reg27_Mask( *((Xuint32 *)&fofb_VOUT_RAM_ADDR));
}



/*
 * start address:
 * 	x = 0
 * 	y = 180
 *
 *	fpga :  refDpram_32x1024 ref_orbit_xy_buffer
 *
 *  parameter	CMDR_POS_REF_ADDR = 10;
    parameter	CMDR_POS_REF_DATA = 11;

    Unit nm
 */
int fofb_ReferenceOrbitWrite(int start_addr, int end_addr, int *pData)
{
int i;
int	Data;
int error;

	fofb_ref_dpram.wr = 0;		//disable write
	fofb_ref_dpram.en = 1;	//cs
	fofb_Reg10_Mask( *((Xuint32 *)&fofb_ref_dpram ));

	for(i=start_addr; i<end_addr; i++)
	{
		fofb_ref_dpram.addr   = i;
		fofb_Reg10_Mask( *((Xuint32 *)&fofb_ref_dpram ));
		Data = *pData++;
		fofb_DpRamDataWrite( Data );
		//
		fofb_ref_dpram.wr = 1;		//write
		fofb_Reg10_Mask( *((Xuint32 *)&fofb_ref_dpram ));
		fofb_ref_dpram.wr = 0;		//write
		fofb_Reg10_Mask( *((Xuint32 *)&fofb_ref_dpram ));
		//xil_printf("%d = %d\r\n", i, *pData );
		if(start_addr < ACTIVE_RAM_BLOCK_SIZE)
			FOFB_DATA->HV_RefOrbit[i] = Data;
		else
			FOFB_DATA->HV2_RefOrbit[i] = Data;
	}

	fofb_ref_dpram.en = 0;
	fofb_Reg10_Mask( *((Xuint32 *)&fofb_ref_dpram ));

	//read RAM data
	error = fofb_ReferenceOrbitRead (start_addr, end_addr, NULL);
	FOFB_DATA->err_cnt = error;
	xil_printf("!!! Reference ORBIT write [%d - %d], %d errors Done...\r\n", start_addr, end_addr, error);
	return error;
}

// reference orbit data read from DP RAM
int fofb_ReferenceOrbitRead(int start_addr, int len, int *pData)
{
	int i;
	int data;
	int error=0;

	fofb_ref_dpram.en = 1;
	fofb_ref_dpram.wr = 0;
	fofb_Reg10_Mask( *((Xuint32 *)&fofb_ref_dpram));

	for(i=start_addr; i<len; i++) {
		fofb_ref_dpram.addr = i;
		fofb_Reg10_Mask( *((Xuint32 *)&fofb_ref_dpram));
    	// nm
		data = Xil_In32(UP_AXI + (FOFB_REG_REF_ORBIT_READ*4));
		*(pData+(i)) = htonl(data);   //to IOC

		//Compare with DDR set values
		if(start_addr < ACTIVE_RAM_BLOCK_SIZE) {
			if(FOFB_DATA->HV_RefOrbit[i] != data) {
				xil_printf("compare error %3d = %X - %X\r\n", i, FOFB_DATA->HV_RefOrbit[i], data);
				error++;
			}
			FOFB_DATA->HV_RefOrbit_RD[i] = data;
		}
		else {
			if(FOFB_DATA->HV2_RefOrbit[i] != data) {
				xil_printf("compare error %3d = %d - %d\r\n", i, FOFB_DATA->HV2_RefOrbit[i], data);
				error++;
			}
		}

		//if(dispMode == 1) {
		//	printf("%3d = %.3f (um)\r\n", i, (float)(data* 0.001) );
		//}
	}

	fofb_ref_dpram.en = 0;
	fofb_ref_dpram.wr = 0;
	fofb_Reg10_Mask( *((Xuint32 *)&fofb_ref_dpram));
	return error;
}

//
int fofb_SDI_DataLinkRead(int start_addr, int len, int *pData)
{
	int i;
	int data;
	int error=0;

	fofb_sdiLink_dpram.en = 1;
	fofb_Reg2_Mask( *((Xuint32 *)&fofb_sdiLink_dpram));

	for(i=start_addr; i<len; i++) {
		fofb_sdiLink_dpram.addr = i;
		fofb_Reg2_Mask( *((Xuint32 *)&fofb_sdiLink_dpram));
    	// nm
		data = Xil_In32(UP_AXI + (FOFB_REG_SDILINK_DATA_READ*4));
		*(pData+(i-1)) = htonl(data);   //to IOC
		//FOFB_DATA->SDI_LinkData_RD[i] = data;

		//if(dispMode == 1) {
		//	printf("%3d = %.3f (um)\r\n", i, (float)(data* 0.001) );
		//}
	}

	fofb_sdiLink_dpram.en = 0;
	fofb_Reg2_Mask( *((Xuint32 *)&fofb_sdiLink_dpram));
	return error;
}




void fofb_F32_ErrorOrbitRead(int dispMode, int len, float *pData)
{
	int i;
	Xint32 data;
	float f;
	Xuint32 d2;


	fofb_ref_dpram.en_pos_err_ram = 1;
	fofb_Reg10_Mask( *((Xuint32 *)&fofb_ref_dpram));

	for(i=0; i<len; i++) {
		fofb_ref_dpram.addr = i;
		fofb_Reg10_Mask( *((Xuint32 *)&fofb_ref_dpram));
    	//
		data = Xil_In32(UP_AXI + (FOFB_REG_POS_ERR_FLOAT_READ*4));
		//ieee754 -> float
		if(dispMode == 1) {
			d2 = htonl(data);   //to IOC
			f = *(float *)&d2;
			*(pData+(i)) = f;
		}
		else {
			//ieee754 -> float
			f = *(float *)&data;
			*(pData+(i)) = f;
			//printf("%3d = %.3f (um)\r\n", i, f );
		}
	}
	fofb_ref_dpram.en_pos_err_ram = 0;
	fofb_Reg10_Mask( *((Xuint32 *)&fofb_ref_dpram));
}

/*
 * Ut RAM selection for enable/WR
 */
void fofb_UtRamEnableMaskCtrl(int addr, Xuint8 data_en, Xuint8 data_wr)
{
	//for(i=0; i<MaxAddr; i++)
	{
		fofb_UT_RAM_EN.wr = 1;
		fofb_UT_RAM_EN.en = 1;
		fofb_Reg5_Mask( *((Xuint32 *)&fofb_UT_RAM_EN ));
		//
		fofb_UT_RAM_EN.addr      = addr;
		fofb_UT_RAM_EN.data_en   = data_en;
		fofb_UT_RAM_EN.data_wr   = data_wr;
		fofb_UT_RAM_EN.Ut_RamB_Out_enable = 1;   //Ut RAM out enable
		fofb_Reg5_Mask( *((Xuint32 *)&fofb_UT_RAM_EN ));
	}
	fofb_UT_RAM_EN.wr = 0;
	fofb_UT_RAM_EN.en = 0;
	fofb_Reg5_Mask( *((Xuint32 *)&fofb_UT_RAM_EN ));
}


#define	RAM_WR_MODE	1
#define	RAM_RD_MODE	0
#define	RAM_ENABLE	1
#define	RAM_DISABLE	0


/*
 * Ut RAM write int32 type for test
 *
 * uRamBlock : Ut RAM block 0...179
 */
void fofb_UtVectorInt32DataWrite(int uRamBlock, int len, int *pData)
{
	int ram_addr;
	int Data;

	fofb_UtRamEnableMaskCtrl(uRamBlock, RAM_ENABLE, RAM_WR_MODE); //enable

	for(ram_addr=0; ram_addr<len; ram_addr++) {
		Xil_Out32(UP_AXI+CMDR_UT_RAM_ADDR*4, ram_addr);
		Data = *pData++;
		Xil_Out32(UP_AXI+CMDR_UT_RAM_DATA*4, Data);
		//
		DDR_UT->data[uRamBlock][ram_addr]  = Data;
	}
	fofb_UtRamEnableMaskCtrl(uRamBlock, RAM_DISABLE, RAM_RD_MODE); //disable
}

//wavefrom received from IOC, F32 format
void fofb_IOC_UtVectorInt32DataWrite(int uRamBlock, int len, int *pData)
{
	int ram_addr;
	int Data;
	int data2;
	float f;

	fofb_UtRamEnableMaskCtrl(uRamBlock, RAM_ENABLE, RAM_WR_MODE); //enable

	for(ram_addr=0; ram_addr<len; ram_addr++) {
		Xil_Out32(UP_AXI+CMDR_UT_RAM_ADDR*4, ram_addr);
		Data = *pData++;
		data2 = htonl(Data);
		Xil_Out32(UP_AXI+CMDR_UT_RAM_DATA*4, data2);
		DDR_UT->data[uRamBlock][ram_addr] = data2; //for compare with readback
		//save reference
		f = *(float *)&data2;
		DDR_UT->f32SetData[uRamBlock][ram_addr]  = f;
	}
	fofb_UtRamEnableMaskCtrl(uRamBlock, RAM_DISABLE, RAM_RD_MODE); //disable
}

//Int32 Mode for test
int fofb_UtRAMInt32Read(int dispMode, int uRamBlock, int len, float *pData)
{
	int addr;
	Xuint32 data;
	//Xuint32 d2;
	//float f;
	int error=0;

	fofb_UtRamEnableMaskCtrl(uRamBlock, RAM_ENABLE, RAM_RD_MODE);

	Xil_Out32(UP_AXI+(CMDR_UT_RAM_BLK_IDX*4), uRamBlock);  //select Port A output index
	for(addr=0; addr<len; addr++)
	{
		Xil_Out32(UP_AXI+(CMDR_UT_RAM_ADDR*4), addr); //select RAM address
		data = Xil_In32(UP_AXI+(FOFB_REG_UT_DPRAM_DATA_READ*4));
		*(pData+(addr)) = data;
#if 0
		//ieee754 -> float
		if(dispMode == 1) {
			d2 = htonl(data);   //to IOC
			f = *(float *)&d2;
			*(pData+(addr)) = f;
		}
		else {
			//ieee754 -> float
			f = *(float *)&data;
			*(pData+(addr)) = f;
			DDR_UT->f32ReadData[uRamBlock][addr] = f;
		}
#endif
		//
		if(data != DDR_UT->data[uRamBlock][addr]) {
			xil_printf("Ut compare error %d[%3d] = %X  %X\r\n", uRamBlock, addr, DDR_UT->data[uRamBlock][addr], data);
			error++;
		}
	}
	//
	fofb_UtRamEnableMaskCtrl(uRamBlock, RAM_DISABLE, RAM_RD_MODE); //disable
	DDR_UT->err_cnt = error;

	//printf("UT %d read done = %d\r\n", uRamBlock, error );
	return error;
}


//
void fofb_EigenRegFloatRead(int type, int mode, int len, float *pData)
{
	int addr;
	Xint32 data;
	float f;
	Xuint32 d2;

	for(addr=0; addr<len; addr++) {
		Xil_Out32(UP_AXI+CMDR_UT_RAM_BLK_IDX*4, addr);
		if(mode == 0)
			data = Xil_In32(UP_AXI + (FOFB_REG_EIGEN_DATA_READ*4));
		else if(mode == 1)
			data = Xil_In32(UP_AXI + (FOFB_REG_EIGEN_SUMI_DATA_READ*4));
		else
			data = Xil_In32(UP_AXI + (FOFB_REG_EIGEN_SUMD_DATA_READ*4));
		if(type == 0) {
			f = *(float *)&data;
			*(pData+(addr)) = f;
		}else {
			d2 = htonl(data);   //to IOC
			f = *(float *)&d2;
			*(pData+(addr)) = f;
		}
	}

}


// V RAM Control
// P, I, D RAM read/write, max 180

/*

#define CMDR_V_P_CTRL             17
#define CMDR_V_I_CTRL             18
#define CMDR_V_D_CTRL             19
*/
void fofb_VmRamEnable_MaskCtrl(int mode, int addr, Xuint8 data_en, Xuint8 data_wr)
{

	if(data_en)
	{
		if(data_wr) {
			//printf("V RAM write mode %d\r\n", mode);

			switch (addr) {
				case 0:
					Xil_Out32(UP_AXI+(CMDR_V_P_CTRL+mode)*4, 0x401); //0 WR/enable
					break;
				case 1:
					Xil_Out32(UP_AXI+(CMDR_V_P_CTRL+mode)*4, 0x802); //0 enable
					break;
				case 2:
					Xil_Out32(UP_AXI+(CMDR_V_P_CTRL+mode)*4, 0x1004); //0 enable
					break;
				case 3:
					Xil_Out32(UP_AXI+(CMDR_V_P_CTRL+mode)*4, 0x2008); //0 enable
					break;
				case 4:
					Xil_Out32(UP_AXI+(CMDR_V_P_CTRL+mode)*4, 0x4010); //0 enable
					break;
				case 5:
					Xil_Out32(UP_AXI+(CMDR_V_P_CTRL+mode)*4, 0x8020); //0 enable
					break;
			}
		}
		else {
			//printf("V RAM read mode %d\r\n", mode);

			switch (addr) {
				case 0:
					Xil_Out32(UP_AXI+(CMDR_V_P_CTRL+mode)*4, 0x001); //0 enable
					break;
				case 1:
					Xil_Out32(UP_AXI+(CMDR_V_P_CTRL+mode)*4, 0x002); //0 enable
					break;
				case 2:
					Xil_Out32(UP_AXI+(CMDR_V_P_CTRL+mode)*4, 0x004); //0 enable
					break;
				case 3:
					Xil_Out32(UP_AXI+(CMDR_V_P_CTRL+mode)*4, 0x008); //0 enable
					break;
				case 4:
					Xil_Out32(UP_AXI+(CMDR_V_P_CTRL+mode)*4, 0x010); //0 enable
					break;
				case 5:
					Xil_Out32(UP_AXI+(CMDR_V_P_CTRL+mode)*4, 0x020); //0 enable
					break;
			}
		}
	}
	else {
		//printf("V RAM disable mode %d\r\n", mode);
		Xil_Out32(UP_AXI+(CMDR_V_P_CTRL+mode)*4, 0);
	}
}

/*
 * Mode 0: P, 1:I, 2:D
 *
 */
void fofb_V_VectorWrite(int activeRam, char Mode, int vRamBlock, int start_addr, int MaxAddr, int *pData)
{
int i;
//int n;
//int rd;
Xuint32	wData[V_XY_SIZE];	//90 x 3
//Xuint32 rData[V_XY_SIZE];	//90 x 3

	fofb_VmRamEnable_MaskCtrl(Mode, vRamBlock, 1, 1); //WR/ENABLE

	//2. write address and data

	for(i=0; i<MaxAddr; i++)
	{
		wData[i] = *pData++;
		Xil_Out32( UP_AXI+CMDR_V_RAM_ADDR*4, i);
		Xil_Out32( UP_AXI+CMDR_V_RAM_DATA*4, wData[i] );
		if(activeRam == 0) {
			// copy to DDR
			if(Mode == 0)
				DDR_VV->pdata[vRamBlock][i] = wData[i];  //P RAM
			else if(Mode == 1)
				DDR_VV->idata[vRamBlock][i] = wData[i];  //I RAM
			else
				DDR_VV->ddata[vRamBlock][i] = wData[i];  //D RAM
		} else {  //RAM block 2
			if(Mode == 0)
				DDR_VV->pdata2[vRamBlock][i] = wData[i];  //P RAM
			else if(Mode == 1)
				DDR_VV->idata2[vRamBlock][i] = wData[i];  //I RAM
			else
				DDR_VV->ddata2[vRamBlock][i] = wData[i];  //D RAM
		}
		//printf("addr=%d, 0x%X\r\n", i, wData[i]);
	}
	fofb_V_VectorRamRead(0, Mode, vRamBlock, MaxAddr, NULL);
    //
	fofb_VmRamEnable_MaskCtrl(Mode, vRamBlock, 0, 0);

	//DDR_VV->v_ram_wr_flag++;
}

void fofb_IOC_V_VectorWrite(int activeRam, char Mode, int vRamBlock, int MaxAddr,  int *pData)
{
int i;
unsigned int wData;
int data;
float f;

	fofb_VmRamEnable_MaskCtrl(Mode, vRamBlock, 1, 1); //WR/ENABLE

	//2. write address and data
	for(i=0; i<MaxAddr; i++)
	{
		wData = *pData++;
		data = htonl(wData);
		//
		Xil_Out32( UP_AXI+CMDR_V_RAM_ADDR*4, i);
		Xil_Out32( UP_AXI+CMDR_V_RAM_DATA*4, data );
		// copy to DDR
		if(activeRam == 0) {
			if(Mode ==0) {
				f = *(float *)&data;
				DDR_VV->pdata[vRamBlock][i] = data;
				DDR_VV->pfdata[vRamBlock][i] =f;
			}
			else if(Mode ==1) DDR_VV->idata[vRamBlock][i] = data;
			else DDR_VV->ddata[vRamBlock][i] = data;
		} else {
			if(Mode ==0) DDR_VV->pdata2[vRamBlock][i] = data;
			else if(Mode ==1) DDR_VV->idata2[vRamBlock][i] = data;
			else DDR_VV->ddata2[vRamBlock][i] = data;
		}

		//printf("addr=%d, 0x%X\r\n", i, wData[i]);
	}
	fofb_V_VectorRamRead(activeRam, Mode, vRamBlock, MaxAddr, NULL);

	fofb_VmRamEnable_MaskCtrl(Mode, vRamBlock, 0, 0);

	//DDR_VV->v_ram_wr_flag++;
}



/*
 *
 */
int	fofb_V_VectorRamRead(int activeRAM, char Mode, int vRamBlock, int len, unsigned int *pData)
{
int addr;
Xuint32 data, sData;
//float f;
//Xuint32 d;
int	error = 0;
int base;

	fofb_VmRamEnable_MaskCtrl(Mode, vRamBlock, 1, 0);  //read mode

	if(Mode==0) base = FOFB_REG_V_P_DPRAM_DATA_READ;
	else if(Mode==1) base = FOFB_REG_V_I_DPRAM_DATA_READ;
	else base = FOFB_REG_V_D_DPRAM_DATA_READ;

	Xil_Out32(UP_AXI+(CMDR_V_RAM_BLK_IDX*4), vRamBlock);  //select Port A output index
	for(addr=0; addr<len; addr++) {
		Xil_Out32(UP_AXI+CMDR_V_RAM_ADDR*4, addr);
		data = Xil_In32(UP_AXI + (base*4));  //V-P RAM
		*(pData+(addr)) = data;

		if(activeRAM == 0) {
			if(Mode ==0) sData = DDR_VV->pdata[vRamBlock][addr];
			else if(Mode ==1) sData = DDR_VV->idata[vRamBlock][addr];
			else sData = DDR_VV->ddata[vRamBlock][addr];
		} else {
			if(Mode ==0) sData = DDR_VV->pdata2[vRamBlock][addr];
			else if(Mode ==1) sData = DDR_VV->idata2[vRamBlock][addr];
			else sData = DDR_VV->ddata2[vRamBlock][addr];
		}

		if( sData != data) {
			xil_printf("V compare error %3d = %d - %d\r\n", addr, sData , data);
			error++;
		}
		//printf("%d = %X %X\r\n", addr, sData, data );
#if 0
		if(dispMode == 1) {
			printf("%d = %X\r\n", addr, data );
		} else if (dispMode == 2){
			//ieee754 -> float
			d = (Xuint32)data;
			f = *(float *)&d;
			//*(pData+(j) ) = f;
			printf("%d = %.3f\r\n", addr, f );
		}
#endif
	}

	DDR_VV->err_cnt = error;
	fofb_VmRamEnable_MaskCtrl(Mode, vRamBlock, 0, 0);

	//printf("V write Mode=%d ram=%d, error=%d\r\n", Mode, vRamBlock, DDR_VV->err_cnt );
	return error;
}

