/*
 * console.c
 *
 *  Created on: Jun 27, 2024
 *      Author: kha
 */


#include "time.h"
#include "sys/time.h"
#include <stdio.h>
#include <stdlib.h>
#include "types.h"

#include "xparameters.h"
#include "xil_io.h"

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/def.h"
#include "xil_types.h"

#include "xuartps_hw.h"


#include "fofb.h"
#include "sys_io.h"



int cmd_unknown();
int ver();
int help();

int	cmd_fofb_V_Ram_WE_Test(int argc, char *argv[]);
int	cmd_fofb_V_Ram_READ_Test(int argc, char *argv[]);
int	cmd_fofb_Ut_Ram_Zero_Test(int argc, char *argv[]);
int	cmd_fofb_Ut_Matrix_Cal_Test(int argc, char *argv[]);
int cmd_DMA2_PosXy_Read(int argc, char *argv[]);



/**/
BYTE*   skipwh (BYTE *s);
BYTE*   scan (BYTE *s, BYTE *d);

static 	BYTE *tail;
struct 	table *lookup(struct table *, BYTE *);
void 	strlow(char *str);


struct table
{
        char    *entry;
        int     (*func)(int, char **);
};



struct table  commands[] =
{

    {"h", 		 help},
    {"?", 		 help},
    {"ver", 	 ver},

	{"vw",       cmd_fofb_V_Ram_WE_Test},
	{"vr",       cmd_fofb_V_Ram_READ_Test},
	{"utz",      cmd_fofb_Ut_Ram_Zero_Test},
	{"utcal",    cmd_fofb_Ut_Matrix_Cal_Test},

	{"dma2",     cmd_DMA2_PosXy_Read},

	//
	{"",  		 cmd_unknown}			/* command not fount */
};





/**/
struct table *lookup(p, token)
struct table *p;
BYTE *token;
{
    while(*(p -> entry) != '\0')
    {
            if(strcmp(p -> entry, token) == 0)
                    break;
            else
                    ++p;
    }
    return p;
}

/**/
BYTE*   skipwh (BYTE *s)
{
    while(*s && (*s == '\r' || *s == '\n' || *s == ' ' || *s == '\t'))
            ++s;
    return s;
}

/**/
BYTE*   scan (BYTE *s, BYTE *d)
{
    s = skipwh(s);
    while(*s &&
     !( *s == '\r'
     || *s == '\n'
     || *s == ' '
     || *s == '\t'
     || *s == '='))
            *d++ = *s++;
    *d = '\0';
    return s;
}

/**/
void strlow(char *str) {

  int i;

  for ( i = 0; i < (signed)strlen(str) ; i++)
    str[i] = tolower(str[i]);

}

/*
 *
 */
int   cmd_unknown()
{
 	print(("\r\nUnknown command...\r\n"));
    return TRUE;
}


/*
 * Version information.
 */
int  ver()
{
	printf("\r\nFree heap %d bytes\r\n", (int) xPortGetFreeHeapSize());

    return TRUE;
}

int  help()
{
    xil_printf("h         :  help\r\n");
    xil_printf("?         :  help\r\n");
    xil_printf("ver       :  ver\r\n");
    xil_printf("\r\n");

    xil_printf("vw        :  V ram Write test\r\n");
    xil_printf("vr        :  V ram Read test\r\n");
    xil_printf("utz       :  Ut RAM ram zero\r\n");
    xil_printf("utcal     :  Ut Matrix Calculation test\r\n");
    xil_printf("dma2      :  DMA2 read PS data\r\n");


    return TRUE;
}


/*
 * VRAM control test
 */
int	cmd_fofb_V_Ram_WE_Test(int argc, char *argv[])
{
	int	bram_i;
	int type;
	float data;
	int	hex;
	int i;
	int	ut_data[PS_HV_SIZE];
	static char UserResponse[32];
	int format=0;
	int mode;

	xil_printf("Enter P.I.D mode [0-2] : ");
	while(!GetUserInput(NULL, UserResponse, sizeof(UserResponse)-1));
	mode = strtoul(UserResponse, NULL, 10);

	xil_printf("Enter V Number [0-5] : ");
	while(!GetUserInput(NULL, UserResponse, sizeof(UserResponse)-1));
	bram_i = strtoul(UserResponse, NULL, 10);

	xil_printf("Enter Data type [0:count, 1:constant, 2:single ]: ");
	while(!GetUserInput(NULL, UserResponse, sizeof(UserResponse)-1));
	type = strtoul(UserResponse, NULL, 10) & 0x3;

	xil_printf("Enter Start of Data (F32) : ");
	while(!GetUserInput(NULL, UserResponse, sizeof(UserResponse)-1));
	data = strtod(UserResponse, NULL);

	if(type == 0) {	//counter
		for(i=0; i<PS_HV_SIZE; i++) {
			if(format == 0)
				ut_data[i] = Float2DWordReg(data+i);	//ieee754 format
			else
				ut_data[i] = (int)(data+i);	//int32 format
		}

		fofb_V_VectorWrite(0, mode, bram_i,  0, PS_HV_SIZE, &ut_data[0]);

	}
	else if(type == 1){  //constant
		printf("FLOAT Data = %d:%d %f\r\n", mode, bram_i, data);

		for(i=0; i<PS_HV_SIZE; i++) {
			hex = Float2DWordReg(data);
			ut_data[i] = hex;
		}
		fofb_V_VectorWrite(0, mode, bram_i,  0, PS_HV_SIZE, &ut_data[0]);

	}
	else {
		int start_addr;
		xil_printf("Enter RAM address [0..179] : ");
		while(!GetUserInput(NULL, UserResponse, sizeof(UserResponse)-1));
		start_addr = strtoul(UserResponse, NULL, 10);
		if(start_addr >= PS_HV_SIZE) start_addr = PS_HV_SIZE-1;

		hex = Float2DWordReg(data);
		fofb_V_VectorWrite(0, mode, bram_i,  start_addr, start_addr+1, &ut_data[0]);

		printf("V ram addr %d :  %f\r\n", start_addr, data);
	}
return 0;
}

/*
 * VRAM read test
 */
int	cmd_fofb_V_Ram_READ_Test(int argc, char *argv[])
{
	static char UserResponse[32];
	//float data;
	int	ram_i;
	int i;
	unsigned int	ut_data[PS_HV_SIZE];
	int mode;

	xil_printf("Enter PID mode [0-2] : ");
	while(!GetUserInput(NULL, UserResponse, sizeof(UserResponse)-1));
	mode = strtoul(UserResponse, NULL, 10);

	xil_printf("Enter RAM block [0-5] : ");
	while(!GetUserInput(NULL, UserResponse, sizeof(UserResponse)-1));
	ram_i = strtoul(UserResponse, NULL, 10);


	fofb_V_VectorRamRead(0, mode, ram_i, 180, &ut_data[0]);

	for(i=0; i<180; i++) {
		printf("VRAM mode=%d blk=%d addr=%d data=0x%X\r\n", mode, ram_i, i, ut_data[i]);
	}

return 0;
}


int	cmd_fofb_Ut_Ram_Zero_Test(int argc, char *argv[])
{
	int i, j;
	int ut_data[BPM_XY_SIZE];
	uint32_t hex;
	float fdata;
	static char UserResponse[32];

	xil_printf("Enter Ut Data (F32) : ");
	while(!GetUserInput(NULL, UserResponse, sizeof(UserResponse)-1));
	fdata = strtod(UserResponse, NULL);

	//Ut Matrix download [0..189]
	for(i=0; i<PS_HV_SIZE; i++) {
		for(j=0; j<BPM_XY_SIZE; j++) {
			DDR_UT->f32SetData[i][j] = fdata;
			hex = Float2DWordReg(DDR_UT->f32SetData[i][j]);
			ut_data[j] = hex;
		}
		fofb_UtVectorInt32DataWrite(i, BPM_XY_SIZE, &ut_data[0]);
		fofb_UtRAMInt32Read(2, i, BPM_XY_SIZE, NULL);

	}
printf("Ut RAM Write Done..\r\n");

}


/*
 *
 */
float pErr[BPM_XY_SIZE];
float fpga_eigen[PS_HV_SIZE];
float Ut_sum[PS_HV_SIZE];
int	cmd_fofb_Ut_Matrix_Cal_Test(int argc, char *argv[])
{
	//UT Matrix Cal test
	int i, j;


	fofb_EigenRegFloatRead (0, 0, PS_HV_SIZE, &fpga_eigen[0]);
	fofb_F32_ErrorOrbitRead(0, BPM_XY_SIZE, &pErr[0]);    //read Orbit error

	for(i=0; i<PS_HV_SIZE; i++) {
		DDR_UT->f32Ut_Eigen_Pdata[i] = 0.0; //clear
		for(j=0; j<BPM_XY_SIZE; j++) {
			DDR_UT->f32Ut_Eigen_Pdata[i] += pErr[j] * DDR_UT->f32SetData[i][j];
			//
			//fofb_UtRAMInt32Read(0, i, BPM_XY_SIZE, NULL);
			//Ut_sum[i] += pErr[j] * DDR_UT->f32ReadData[i][j];
		}
		//0.001 error
		printf("Ut_sum [%d] = %.3f %.3f, %.3f\r\n", i, DDR_UT->f32Ut_Eigen_Pdata[i], fpga_eigen[i], DDR_UT->f32Ut_Eigen_Pdata[i] - fpga_eigen[i]);
	}

#if 1
	float vsum[6];
	for(i=0; i<6; i++) {
		vsum[i] = 0.0;
		for(j=0; j<PS_HV_SIZE; j++) {
			vsum[i] += fpga_eigen[j] * DDR_VV->pfdata[i][j];
		}
		printf("V_P_sum [%d] = %.3f, %.3f\r\n", i, vsum[i], DDR_VV->pfSumOutdata[7+i]);
	}
#endif

	return 0;
}
//////


/*
 * PositionXy display
 */
int cmd_DMA2_PosXy_Read(int argc, char *argv[])
{
int offset;
u32 data;
	for(offset=0; offset<100; offset++) {
		data = Xil_In32(DMA_ADC2_DATA_START+(offset*4));
		//for console display use htonl
		printf("%d = %X\r\n", offset, htonl(data));
	}
	return 0;
}



////////////////////////////////////////////////////////////////////////
/**/
/*
 *
 */
void  dBpm_ConsoleCmd_Processer(char	*rx_buff)
{
register struct table *p;
struct table *lookup();
char    lbuf[128];
char   	*lp=lbuf;
BYTE   	*scan(), *skipwh();
int   	argc;
char 	*argv[128];
char 	args[5][128];

	for(argc = 0; argc < 5; argc++) {
    	argv[argc] = (char *)0;
        args[argc][0] = '\0';
    }

    strlow(rx_buff);
	lp = scan(rx_buff, args[0]);

    argv[0] = args[0];
    tail = skipwh(lp);

    for(argc = 1; ; argc++) {
    	lp = scan(lp, args[argc]);
        if(*args[argc] == '\0')
        	break;
        else {
        	argv[argc] = args[argc];
        }
	}

    if(*argv[0] != '\0') {
    	p = lookup(commands, argv[0]);
		(*(p->func)) (argc, argv);
    }

}

// 2/7/23 modified
u8 _XUartPs_RecvByte(u32 BaseAddress)
{
	u32 RecievedByte;
	/* Wait until there is data */
	while (!XUartPs_IsReceiveData(BaseAddress)) {
		vTaskDelay(2);
	}
	RecievedByte = XUartPs_ReadReg(BaseAddress, XUARTPS_FIFO_OFFSET);
	/* Return the byte received */
	return (u8)RecievedByte;
}


/*****************************************************************************/
/**
* Retrieve a line of input from the user. A line is defined as all characters
* up to a newline.
*
* @param    Prompt Printed before string is accepted to queue the user to enter
*           something.
* @param    Response Null terminated string with newline stripped
* @param    MaxChars Maximum number of characters to read (excluding null)
*
* @return   Number of characters read (excluding newline)
*
* @note     None
*
******************************************************************************/
int GetUserInput(char* Prompt, char* Response, int MaxChars)
{
    u32 Index;
    u8 Finished;

    /*
     * Display prompt
     */
    if (Prompt) xil_printf(Prompt);

    /*
     * This basically implements a fgets function. The standalone EDK stdin
     * is apparantly buggy because it behaves differently when newline is
     * entered by itself vs. when it is entered after a number of regular chars
     * have been entered.Characters entered are echoed back.
     */
    Finished = 0;
    Index = 0;

    while(!Finished)
    {
        /*
         * Flush out any output pending in stdout
         */
        fflush(stdout);

        /*
         * Wait for a character to arrive
         */


#if CONSOLE_UART  //Serial Console
CHK_KEY:
		if(! XUartPs_IsReceiveData(STDIN_BASEADDRESS) )
		{
			vTaskDelay(5);
           	goto CHK_KEY;
        }
		if(Index > 20) Finished = 1;
        Response[Index] = _XUartPs_RecvByte(STDIN_BASEADDRESS);
#else  //JTAG mode
		if(Index > 20) Finished = 1;
        Response[Index] = inbyte();   //for testONLY, it caused a tcp/ip socket slow down
#endif

        /*
         * Normal chars, add them to the string and keep going
         */
        if ((Response[Index] >= 0x20) && (Response[Index] <=0x7E))
        {
            xil_printf("%c", Response[Index++]);
            continue;
        }

        /*
         * Control chars
         */
        switch(Response[Index])
        {
            /*
             * Carriage return
             */
            case 0x0D:
                Response[Index] = 0;
                Finished = 1;
                xil_printf("\r\n");
                break;

            /*
             * Backspace
             */
            case 0x08:
                if (Index != 0)
                {
                    /*
                     * Erase previous character and move cursor back one space
                     */
                    xil_printf("\b \b");
                    Response[--Index] = 0;
                }
                break;

            /*
             * Ignore all other control chars
             */
            default:
                continue;
        }
    }

    return(Index);
}
