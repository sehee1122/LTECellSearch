#include "NHAL_rx.h"
#include "vThread_rx.h"
#include "rt_wrapper.h"


#define NHAL_WATCHDOG_RXONOFF		100
#define NHAL_WATCHDOG_RXINIT		5000


static pthread_t nhal_vThread_rx;
static pthread_attr_t nhal_vthreadattr_rx;
static ix_halRx_t*nhal_rx_ix;

long long nhal_wdog_rxonoff;





typedef enum {
	nhal_rxNull=0,
	nhal_rxOff,
	nhal_rxOn_going,
	nhal_rxOn,
	nhal_rxOff_going
} nhal_rfStatus_e;



struct {
	nhal_rfStatus_e status;
	rxHwMode_e mode;
	int preambleSeq[VRX_NB_MAX_PREAMBLE_SEQ];
	uint16_t preamble_length;

	uint32_t detect_window;
	uint32_t detect_offset;
	
} nhal_rxInstance;




int NHAL_RXcmd_init(void)
{
	nhal_rx_ix = (ix_halRx_t*)malloc(sizeof(ix_halRx_t));
	if (nhal_rx_ix == NULL)
	{
		LOG_E(PHY, "[ERROR] Failed to make an instance for virtual H/W SRCH interface\n");
		return -1;
	}
	
	pthread_attr_init (&nhal_vthreadattr_rx);
	pthread_attr_setstacksize(&nhal_vthreadattr_rx, 8192);//5*PTHREAD_STACK_MIN);

	//RX vHW initial setting ---------------------------------------------------
	//set as default value
	
	nhal_rx_ix->woReg.detect_offset = 0;
	nhal_rx_ix->woReg.detect_window = 0;
	nhal_rx_ix->woReg.mode = rxMode_oneshot;
	nhal_rx_ix->rwReg.onoff = 0;
	memset(nhal_rx_ix->woReg.preambleSeq, 0, VRX_NB_MAX_PREAMBLE_SEQ*sizeof(int));
	nhal_rx_ix->woReg.preamble_length = 0;

	nhal_rx_ix->roReg.data = (int*)malloc16_clear(VHW_NB_MAXSAMPLES*sizeof(int));

	pthread_mutex_init(&(nhal_rx_ix->woReg.mutex_woHal), NULL);
	pthread_mutex_init(&(nhal_rx_ix->roReg.mutex_roHal), NULL);
	pthread_mutex_init(&(nhal_rx_ix->rwReg.mutex_rwHal), NULL);
	pthread_cond_init(&(nhal_rx_ix->rwReg.cond_rwHal), NULL);

	
	LOG_I(PHY,"Intializing RX virtual thread... \n");
	
	AssertFatal(0 == pthread_create(&nhal_vThread_rx,
								&nhal_vthreadattr_rx,
								vRX_mainThread,
								(void*)nhal_rx_ix), "");

	
	nhal_wdog_rxonoff = rdtsc_oai();
	//wait for status
	while (nhal_rx_ix->roReg.hwStatus != rxst_off)
	{
		usleep(500000);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_rxonoff)/cpuf/1000000.0) < NHAL_WATCHDOG_INIT, "virtual RX on off timeout!! (hwStatus : %i)\n", nhal_rx_ix->roReg.hwStatus);
	}

	return 0;
}






static void NHAL_initRxAbstract(void)
{
	nhal_rxInstance.mode = rxst_null;
	nhal_rxInstance.preamble_length = 0;
	nhal_rxInstance.detect_window = 0;
	nhal_rxInstance.detect_offset = 0;
	memset(nhal_rxInstance.preambleSeq, 0, VRX_NB_MAX_PREAMBLE_SEQ*sizeof(int));
}



int NHAL_RXcmd_on(rxHwMode_e rxmode, 
						uint16_t preamble_length, 
						int *preamble_seq,
						uint32_t detect_window,
						uint32_t detect_offset
						)
{




	//validity check (state)
	if (nhal_rxInstance.status == nhal_rxNull ||
		nhal_rxInstance.status == nhal_rxOn ||
		nhal_rxInstance.status == nhal_rxOn_going)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept on command : RX is already on! (%i)\n", nhal_rxInstance.status);
		return -1;
	}
	if (preamble_length == 0)
	{
		LOG_E(PHY,"[WARNING] HAL cmd is weird, RX preamble length is 0\n");
	}

	

	nhal_rxInstance.status = nhal_rxOn_going;	
	LOG_I(PHY, "[HALcmd] RX on (rxmode : %i, preamble length : %i, detect window : %i, detect offset %i\n",
		rxmode, preamble_length, detect_window, detect_offset);


	//1. wait for HW status to be off ---------------------------------------------
	nhal_wdog_rxonoff = rdtsc_oai();
	//wait for status
	while (nhal_rx_ix->roReg.hwStatus != rxst_off)
	{
		usleep(10);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_rxonoff)/cpuf/1000000.0) < NHAL_WATCHDOG_RXONOFF, "virtual RX initialization timeout!! (hwStatus : %i)\n", nhal_rx_ix->roReg.hwStatus);
	}

	//2. RX vHW configuration ---------------------------------------------------

	//memcpy(&(nhal_rx_ix->woReg.preambleSeq[0]), preamble_zc0, preamble_zcLength*sizeof(int));
	//nhal_rx_ix->woReg.preamble_length = preamble_zcLength;

	typedef enum {
		nhal_rxMode_oneshot =0,
		nhal_rxMode_continuous
	} nhal_rxMode_e;

	if (rxmode == nhal_rxMode_oneshot)
	{
		nhal_rx_ix->woReg.mode = rxMode_oneshot;
	}
	else
	{
		nhal_rx_ix->woReg.mode = rxMode_continuous;
	}


	nhal_rx_ix->woReg.preamble_length = preamble_length;
	memcpy(&(nhal_rx_ix->woReg.preambleSeq[0]), preamble_seq, preamble_length*sizeof(int));
	nhal_rx_ix->woReg.mode = rxMode_continuous;
	nhal_rx_ix->woReg.detect_offset = 0;
	nhal_rx_ix->woReg.detect_window = 0;
	
	nhal_rx_ix->rwReg.onoff = 1;
	pthread_cond_signal(&(nhal_rx_ix->rwReg.cond_rwHal));


	
	nhal_wdog_rxonoff = rdtsc_oai();	
	while (nhal_rx_ix->roReg.hwStatus != rxst_off)
	{
		usleep(10);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_rxonoff)/cpuf/1000000.0) < NHAL_WATCHDOG_ONOFF, "virtual RX on/off timeout!!\n");
	}



	nhal_rxInstance.preamble_length = preamble_length;
	nhal_rxInstance.mode = nhal_rx_ix->woReg.mode;
	nhal_rxInstance.detect_window = detect_window;
	nhal_rxInstance.detect_offset = detect_offset;
	memcpy(&(nhal_rxInstance.preambleSeq[0]), preamble_seq, preamble_length*sizeof(int));

	nhal_rfInstance.status = nhal_rxOn;
	
	return 0;
	
}




int NHAL_RXcmd_off(void)
{
	//validity check (state)
	if (nhal_rxInstance.status == nhal_rxNull ||
		nhal_rxInstance.status == nhal_rxOff ||
		nhal_rxInstance.status == nhal_rxOff_going)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept on command : RX is already on! (%i)\n", nhal_rxInstance.status);
		return -1;
	}

	nhal_rxInstance.status = nhal_rxOff_going;
	LOG_I(PHY, "[HALcmd] RX off\n");

	//1. wait for HW status to be on ---------------------------------------------
	nhal_wdog_rxonoff = rdtsc_oai();
	//wait for status
	while (nhal_rx_ix->roReg.hwStatus != rxst_on)
	{
		usleep(10);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_rxonoff)/cpuf/1000000.0) < NHAL_WATCHDOG_RXONOFF, "virtual RX on/off timeout!!\n");
	}
	

	//2. set the register to be off!
	nhal_rx_ix->rwReg.onoff = 0;

	//3. wait for HW status to be off ---------------------------------------------
	nhal_wdog_rxonoff = rdtsc_oai();
	//wait for status
	while (nhal_rx_ix->roReg.hwStatus != rxst_off)
	{
		usleep(10);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_rxonoff)/cpuf/1000000.0) < NHAL_WATCHDOG_RXONOFF, "virtual RX on/off timeout!!\n");
	}

	NHAL_initRxAbstract();
	nhal_rxInstance.status = nhal_rxOff;
	
	return 0;
}

