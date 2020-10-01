#include "NHAL_tx.h"
#include "vThread_tx.h"
#include "rt_wrapper.h"


#define NHAL_WATCHDOG_TXONOFF		100
#define NHAL_WATCHDOG_TXINIT		5000


static pthread_t nhal_vThread_tx;
static pthread_attr_t nhal_vthreadattr_tx;
static ix_halRx_t*nhal_tx_ix;

long long nhal_wdog_txonoff;





typedef enum {
	nhal_txNull=0,
	nhal_txOff,
	nhal_txOn_going,
	nhal_txOn,
	nhal_txOff_going
} nhal_txStatus_e;






struct {	
	nhal_txStatus_e status;
	uint8_t slotDelay;
	txHwMode_e mode;
	uint32_t preamble_length;
	int preambleSeq[VTX_NB_MAX_PREAMBLE_SEQ];
	uint32_t length[VHW_NB_SAMPLEBANK];
	void* txData[VHW_NB_SAMPLEBANK];

	uint8_t sch_completed;
	uint8_t newIndicator;
	
} nhal_txInstance;



static void NHAL_initTxAbstract(void)
{
	nhal_txInstance.mode = txst_null;
	nhal_txInstance.slotDelay = 0;
	nhal_txInstance.mode = txMode_oneshot;
	for (int i=0;i<VHW_NB_SAMPLEBANK;i++)_
	{
		nhal_txInstance.length[i] = 0;
		nhal_txInstance.txData[i] = NULL;
	}
	nhal_txInstance.sch_completed = 0;
	nhal_txInstance.newIndicator = 0;
	nhal_txInstance.preamble_length = 0;
	memset(nhal_txInstance.preambleSeq, 0, VTX_NB_MAX_PREAMBLE_SEQ*sizeof(int));
}



int NHAL_TXcmd_init(void)
{
	nhal_tx_ix = (ix_halTx_t*)malloc(sizeof(ix_halTx_t));
	if (nhal_tx_ix == NULL)
	{
		LOG_E(PHY, "[ERROR] Failed to make an instance for virtual H/W SRCH interface\n");
		return -1;
	}
	
	pthread_attr_init (&nhal_vthreadattr_tx);
	pthread_attr_setstacksize(&nhal_vthreadattr_tx, 8192);//5*PTHREAD_STACK_MIN);

	//TX vHW initial setting ---------------------------------------------------
	//set as default value

	
	nhal_tx_ix->woReg.mode = txMode_oneshot;
	for (int i=0;i<VHW_NB_SAMPLEBANK;i++)
	{
		nhal_tx_ix->woReg.length[i] = 0;
		nhal_tx_ix->woReg.txData[i] = NULL;
	}
	
	nhal_tx_ix->woReg.newIndicator = 0;
	nhal_tx_ix->rwReg.onoff = 0;
	nhal_tx_ix->woReg.slotDelay = 0;

	pthread_cond_init(&(nhal_tx_ix->woReg.cond_woHal), NULL);
	pthread_mutex_init(&(nhal_tx_ix->woReg.mutex_woHal), NULL);
	pthread_cond_init(&(nhal_tx_ix->roReg.cond_roHal), NULL);
	pthread_mutex_init(&(nhal_tx_ix->roReg.mutex_roHal), NULL);
	pthread_cond_init(&(nhal_tx_ix->rwReg.cond_rwHal), NULL);
	pthread_mutex_init(&(nhal_tx_ix->rwReg.mutex_rwHal), NULL);
	
	LOG_I(PHY,"Intializing TX virtual thread... \n");
	
	AssertFatal(0 == pthread_create(&nhal_vThread_tx,
								&nhal_vthreadattr_tx,
								vTX_mainThread,
								(void*)nhal_tx_ix), "");

	
	nhal_wdog_txonoff = rdtsc_oai();
	//wait for status
	while (nhal_tx_ix->roReg.hwStatus != txst_off)
	{
		usleep(100);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_txonoff)/cpuf/1000000.0) < NHAL_WATCHDOG_TXINIT, "virtual TX on off timeout!! (hwStatus : %i)\n", nhal_tx_ix->roReg.hwStatus);
	}

	return 0;
}











int NHAL_TXcmd_on(nhal_txMode_e txmode, 
						uint8_t slotDelay,
						uint32_t preamble_length,
						)
{
	//validity check (state)
	if (nhal_txInstance.status == nhal_txNull ||
		nhal_txInstance.status == nhal_txOn ||
		nhal_txInstance.status == nhal_txOn_going)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept on command : TX is already on! (%i)\n", nhal_txInstance.status);
		return -1;
	}
	

	

	nhal_txInstance.status = nhal_txOn_going;	
	LOG_I(PHY, "[HALcmd] TX on (rxmode : %i, preamble length : %i, detect window : %i, detect offset %i\n",
		rxmode, preamble_length, detect_window, detect_offset);


	//1. wait for HW status to be off ---------------------------------------------
	nhal_wdog_txonoff = rdtsc_oai();
	//wait for status
	while (nhal_tx_ix->roReg.hwStatus != txst_off)
	{
		usleep(10);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_txonoff)/cpuf/1000000.0) < NHAL_WATCHDOG_TXONOFF, "virtual TX initialization timeout!! (hwStatus : %i)\n", nhal_tx_ix->roReg.hwStatus);
	}

	//2. RX vHW configuration ---------------------------------------------------

	//TX H/W configuration
	if (txmode == nhal_txMode_oneshot)
	{
		nhal_tx_ix->woReg.mode = txMode_oneshot;
	}
	else
	{
		nhal_tx_ix->woReg.mode = txMode_repeat;
	}
	nhal_tx_ix->woReg.slotDelay = slotDelay;
	
	nhal_tx_ix->rwReg.onoff = 1;
	pthread_cond_signal(&(nhal_tx_ix->rwReg.cond_rwHal));
	nhal_wdog_txonoff = rdtsc_oai();
	while (nhal_tx_ix->roReg.hwStatus != txst_off)
	{
		usleep(10);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_txonoff)/cpuf/1000000.0) < NHAL_WATCHDOG_TXONOFF, "virtual TX on/off timeout!!\n");
	}



	nhal_txInstance.mode = nhal_tx_ix->woReg.mode;
	nhal_txInstance.slot = detect_window;
	nhal_txInstance.detect_offset = detect_offset;
	memcpy(&(nhal_rxInstance.preambleSeq[0]), preamble_seq, preamble_length*sizeof(int));

	nhal_rxInstance.status = nhal_rxOn;
	
	return 0;
	
}




int NHAL_RXcmd_off(void)
{
	//validity check (state)
	if (nhal_rxInstance.status == nhal_rxNull ||
		nhal_rxInstance.status == nhal_rxOff ||
		nhal_rxInstance.status == nhal_rxOff_going)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept on command : RX is already off! (%i)\n", nhal_rxInstance.status);
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



int NHAL_RXcmd_waitForIrq(void)
{
	int offset;

	AssertFatal ( 0== pthread_mutex_lock(&(nhal_rx_ix->rwReg.mutex_rwHal)), "");
	while (nhal_rx_ix->rwReg.rxIrq == 0)
		pthread_cond_wait( &(nhal_rx_ix->rwReg.cond_rwHal), &(nhal_rx_ix->rwReg.mutex_rwHal) ); // the thread waits here most of the time
	AssertFatal ( 0== pthread_mutex_unlock(&(nhal_rx_ix->rwReg.mutex_rwHal)), "");

	
	AssertFatal ( 0== pthread_mutex_lock(&(nhal_rx_ix->roReg.mutex_roHal)), "");
	offset = nhal_rx_ix->roReg.tOffset;
	AssertFatal ( 0== pthread_mutex_unlock(&(nhal_rx_ix->roReg.mutex_roHal)), "");

	return offset;
}

int NHAL_RXcmd_getSlotNr(void)
{
	uint8_t slot_nr;
	uint8_t rxIrq;

	AssertFatal ( 0== pthread_mutex_lock(&(nhal_rx_ix->rwReg.mutex_rwHal)), "");
	rxIrq = nhal_rx_ix->rwReg.rxIrq;
	AssertFatal ( 0== pthread_mutex_unlock(&(nhal_rx_ix->rwReg.mutex_rwHal)), "");

	if (rxIrq == 0)
	{
		LOG_E(PHY, "[NHAL RX] WARNING! getting slot number which is invalid (irq is 0)\n");
		return -1;
	}
	
	AssertFatal ( 0== pthread_mutex_lock(&(nhal_rx_ix->roReg.mutex_roHal)), "");
	slot_nr = nhal_rx_ix->roReg.slot_nr;
	AssertFatal ( 0== pthread_mutex_unlock(&(nhal_rx_ix->roReg.mutex_roHal)), "");

	return slot_nr;
}





int NHAL_RXcmd_clrIrq(void)
{	
	AssertFatal ( 0== pthread_mutex_lock(&(nhal_rx_ix->rwReg.mutex_rwHal)), "");
	nhal_rx_ix->rwReg.rxIrq = 0;
	AssertFatal ( 0== pthread_mutex_unlock(&(nhal_rx_ix->rwReg.mutex_rwHal)), "");

	return 0;
}


