/*
 *
 * 6/14/24 create for the freeRTOS based FOFB ARM application.
 *
 *
 */

//
// ssh -L 3121:10.0.153.141:3121 kha@ssh-acc.nsls2.bnl.gov

//IOC location:
//    nsls2/users/kha/epics/fofb2



#include <stdio.h>
#include "xparameters.h"
#include "netif/xadapter.h"
#include "platform_config.h"
#include "xil_printf.h"

#include "FreeRTOS.h"
#include "task.h"


#include "fofb.h"
#include "sys_io.h"


int main_thread();
void echo_application_thread(void *);

void lwip_init();

void pscEpicsRx_thread(); //control
void EpicsWfm_thread();
void EpicsTx_thread(); //10 Hz
void dBpm_ShellDebug_Thread();


#define THREAD_STACKSIZE 1024

static struct netif server_netif;
struct netif *echo_netif;

void task_sleep();
//////////////////////////////////////////

#define	DMA_ADC_SAMPLE_SIZE		1000*8
//ADMA
#include "dmac_core.h"
dmac_core	bpm_dma[4];
dmac_xfer   bpm_adc_rx_xfer[4];
void	Thread_DMA0();
void	Thread_DMA1();
void	Thread_DMA2();



void
print_ip(char *msg, ip_addr_t *ip)
{
	xil_printf(msg);
	xil_printf("%d.%d.%d.%d\n\r", ip4_addr1(ip), ip4_addr2(ip),
			ip4_addr3(ip), ip4_addr4(ip));
}

void
print_ip_settings(ip_addr_t *ip, ip_addr_t *mask, ip_addr_t *gw)
{

	print_ip("Board IP: ", ip);
	print_ip("Netmask : ", mask);
	print_ip("Gateway : ", gw);
}


int main()
{

	sys_thread_new("main_thrd", (void(*)(void*))main_thread, 0,
	                THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

	vTaskStartScheduler();

	while(1);
	return 0;
}

void network_thread(void *p)
{
    struct netif *netif;
    /* the mac address of the board. this should be unique per board */
    unsigned char mac_ethernet_address[] = { 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };

    ip_addr_t ipaddr, netmask, gw;


    netif = &server_netif;

    xil_printf("\r\n\r\n");
    xil_printf("-----ZCU216 FOFB-2 Test Project ------\r\n");


    /* initialize IP addresses to be used */
#if 0
	IP4_ADDR(&ipaddr,  10, 69, 34, 52);
	IP4_ADDR(&netmask, 255, 255, 255,  0);
	IP4_ADDR(&gw,      10, 69, 34, 1);
#else  //) 10.0.143.157
	IP4_ADDR(&ipaddr,  10, 0, 143, 151);
	IP4_ADDR(&netmask, 255, 255, 254,  0);
	IP4_ADDR(&gw,      10, 0, 142, 254);
#endif

    /* print out IP settings of the board */

    print_ip_settings(&ipaddr, &netmask, &gw);
    /* print all application headers */

    /* Add network interface to the netif_list, and set it as default */
    if (!xemac_add(netif, &ipaddr, &netmask, &gw, mac_ethernet_address, PLATFORM_EMAC_BASEADDR)) {
	xil_printf("Error adding N/W interface\r\n");
	return;
    }


    netif_set_default(netif);

    /* specify that the network if is up */
    netif_set_up(netif);

    /* start packet receive thread - required for lwIP operation */
    sys_thread_new("xemacif_input_thread", (void(*)(void*))xemacif_input_thread, netif,
            THREAD_STACKSIZE,
            DEFAULT_THREAD_PRIO);


    xil_printf("\r\n");
    xil_printf("%20s %6s %s\r\n", "Server", "Port", "Connect With..");
    xil_printf("%20s %6s %s\r\n", "--------------------", "------", "--------------------");

    xil_printf("\r\n");


    /////////////////////////////////////////////////////
	fofb_init();
	//fofb_F32_ErrorOrbitRead(1, BPM_XY_SIZE, NULL);
	//fofb_EigenRegFloatRead(2, 180, NULL);
	//fofb_VOUT_RAMRead(1, 45, NULL);

	/////////////////////////////////////////////////////////
	// setup the base addresses
	bpm_dma[0].base_address          = XPAR_AXI_DMAC_0_BASEADDR;
	bpm_adc_rx_xfer[0].start_address = DMA_ADC0_DATA_START;
	// configure the receiver DMA
	bpm_dma[0].type     = DMAC_RX;
	bpm_dma[0].transfer = &bpm_adc_rx_xfer[0];
	bpm_adc_rx_xfer[0].id = 0;
	bpm_adc_rx_xfer[0].no_of_samples = (360*DMA_SAMPLE_SIZE)*1;  //DMA_ADC_SAMPLE_SIZE;
	//
	bpm_dma[1].base_address          = XPAR_AXI_DMAC_1_BASEADDR;
	bpm_adc_rx_xfer[1].start_address = DMA_ADC1_DATA_START;
	// configure the receiver DMA
	bpm_dma[1].type     = DMAC_RX;
	bpm_dma[1].transfer = &bpm_adc_rx_xfer[1];
	bpm_adc_rx_xfer[1].id = 1;
	bpm_adc_rx_xfer[1].no_of_samples = (45*DMA_SAMPLE_SIZE)*1;  //DMA_ADC_SAMPLE_SIZE;

	//DMA2
	bpm_dma[2].base_address          = XPAR_AXI_DMAC_2_BASEADDR;
	bpm_adc_rx_xfer[2].start_address = DMA_ADC2_DATA_START;
	// configure the receiver DMA
	bpm_dma[2].type     = DMAC_RX;
	bpm_dma[2].transfer = &bpm_adc_rx_xfer[2];
	bpm_adc_rx_xfer[2].id = 2;
	bpm_adc_rx_xfer[2].no_of_samples = (480*DMA_SAMPLE_SIZE)*1; //DMA_ADC_SAMPLE_SIZE;

/*
	Xil_Out32(UP_AXI+(CMDR_DAM0_SAMPLE_SIZE*4),  DMA_ADC_SAMPLE_SIZE+3);
	Xil_Out32(UP_AXI+(CMDR_DAM1_SAMPLE_SIZE*4),  DMA_ADC_SAMPLE_SIZE+3);
	Xil_Out32(UP_AXI+(CMDR_DAM2_SAMPLE_SIZE*4),  bpm_adc_rx_xfer[2].no_of_samples*1000000000);
*/
	// EPICS interface threads
    sys_thread_new("epics_ctrl", pscEpicsRx_thread, 0, THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
    sys_thread_new("epics_wfm" , EpicsWfm_thread,   0, THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
    sys_thread_new("epics_10hz", EpicsTx_thread, 0, THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

    //ADMA DMA threads
    sys_thread_new("dma0", Thread_DMA0, 0, THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
    sys_thread_new("dma1", Thread_DMA1, 0, THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
    sys_thread_new("dma2", Thread_DMA2, 0, THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

#if 1
    sys_thread_new("console", dBpm_ShellDebug_Thread, 0,
		THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
#endif

    vTaskDelete(NULL);

    return;
}

int main_thread()
{

	/* initialize lwIP before calling sys_thread_new */
    lwip_init();
    /* any thread using lwIP should be created using sys_thread_new */
    sys_thread_new("NW_THRD", network_thread, NULL,
		THREAD_STACKSIZE,
            DEFAULT_THREAD_PRIO);


    vTaskDelete(NULL);
    return 0;
}


//
void task_sleep()
{
	const TickType_t x100ms   = pdMS_TO_TICKS( 100UL );
	portTickType xLastExecutionTime;

	vTaskDelayUntil( &xLastExecutionTime, x100ms );
}


int cmd_DMA0_Start()
{
	if( !dmac_start_transaction(bpm_dma[0]) )
	{
		//xil_printf("DMA0: data capture Done.\r\n");
		task_sleep();
		return 0;
	}
	else {
		xil_printf("DMA 0 timeout...\r\n");
		return 1;
	}
}

int cmd_DMA1_Start()
{
	if( !dmac_start_transaction(bpm_dma[1]) )
	{
		//xil_printf("DMA1: data capture Done.\r\n");
		task_sleep();
		return 0;
	}
	else {
		xil_printf("DMA 1 timeout...\r\n");
		return 1;
	}
}

int cmd_DMA2_Start()
{
	if( !dmac_start_transaction(bpm_dma[2]) )
	{
		//xil_printf("DMA2: data capture Done.\r\n");
		task_sleep();
		return 0;
	}
	else {
		xil_printf("DMA 2 timeout...\r\n");
		return 1;
	}
}


void	Thread_DMA0()
{
	const TickType_t x100ms   = pdMS_TO_TICKS( 2000UL );
	portTickType xLastExecutionTime;

#if 1
	while(1)
	{
		cmd_DMA0_Start();
		vTaskDelayUntil( &xLastExecutionTime, x100ms );
	}
#else

	int status;
	status = axi_dmac_init(&rx_dmac, &rx_dmac_init);
	if (status) {
		printf("axi_dmac_init() rx init error: %d\n", status);
		//return -1;
	}

	while(1)
	{
		status = DMA2();
		printf("DMA status = %d\r\n", status);
		vTaskDelayUntil( &xLastExecutionTime, x100ms );
	}
#endif

}

void	Thread_DMA1()
{
	const TickType_t x100ms   = pdMS_TO_TICKS( 2000UL );
	portTickType xLastExecutionTime;

	while(1)
	{
		cmd_DMA1_Start();
		vTaskDelayUntil( &xLastExecutionTime, x100ms );
	}
}

void	Thread_DMA2()
{
	const TickType_t x100ms   = pdMS_TO_TICKS( 2000UL );
	portTickType xLastExecutionTime;

	while(1)
	{
		cmd_DMA2_Start();
		vTaskDelayUntil( &xLastExecutionTime, x100ms );
	}
}


int  GetUserInput(char* Prompt, char* Response, int MaxChars);
void dBpm_ConsoleCmd_Processer(char *rx_buff);

void	dBpm_ShellDebug_Thread()
{
static short SystemRunningFlag=1;
static char UserResponse[32]; /* User input for questions */

    xil_printf("Start dBpm_ShellDebug_Thread...\r\n");
/*
    while(1) {
    	xil_printf("%d:%d\r\n", Xil_In32(UP_AXI+(74*4)), Xil_In32(UP_AXI+(75*4)));
    	sleep(1);
    }
*/

	while(SystemRunningFlag)
	{
IDLE:
		if( !GetUserInput("zcu216-cc2dev->", UserResponse, sizeof(UserResponse)-1) ) goto IDLE;
		dBpm_ConsoleCmd_Processer(&UserResponse[0]);
	}
}
