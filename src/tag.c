#include <stdint.h>
#include <string.h>

#include <inc/hw_types.h>

#include <driverlib/adi.h>
#include <driverlib/aon_batmon.h>
#include <driverlib/aon_event.h>
#include <driverlib/aon_ioc.h>
#include <driverlib/aon_rtc.h>
#include <driverlib/aon_wuc.h>
#include <driverlib/ddi.h>
#include <driverlib/event.h>
#include <driverlib/prcm.h>
#include <driverlib/gpio.h>
#include <driverlib/ioc.h>
#include <driverlib/cpu.h>
#include <driverlib/pwr_ctrl.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/rfc.h>
#include <driverlib/osc.h>
#include <driverlib/timer.h>
#include <driverlib/aux_timer.h>

#include <inc/hw_rfc_dbell.h>

#define wait1us(x)      CPUdelay(8*x)
#define wait1ms(x)      CPUdelay(8000*x)
#define wait1s(x)       CPUdelay(8000000*x)
#define directCmd(x)    (x << 16) | 1
// RF
#define CMD_RSSI            0x0403
#define CMD_TRIGGER         0x0404
#define CMD_START_RAT       0x0405
#define CMD_PING            0x0406

#define CMD_NOP             0x0801
#define CMD_RADIO_SETUP     0x0802
#define CMD_FS              0x0803

#define CMD_BLE_ADV_NC      0x1805

#define BLE_ADV_TYPE_DEVINFO      0x01
#define BLE_ADV_TYPE_NAME         0x09
#define BLE_ADV_TYPE_TX_POWER_LVL 0x0A
#define BLE_ADV_TYPE_MANUFACTURER 0xFF

#define ADV_NC_OPTIMIZED

// Overrides for CMD_RADIO_SETUP
uint32_t pOverrides[] = {
#ifndef ADV_NC_OPTIMIZED
    0x00001007,  // Remove if RFE patch is not applied.
    0x00456088, // Adjust AGC reference level
#endif
    0x00354038, // Synth: Set RTRIM (POTAILRESTRIM) to 5
    0x4001402D, // Synth: Correct CKVD latency setting (address)
    0x00608402, // Synth: Correct CKVD latency setting (value)
    0x4001405D, // Synth: Set ANADIV DIV_BIAS_MODE to PG1 (address)
    0x1801F800, // Synth: Set ANADIV DIV_BIAS_MODE to PG1 (value)
    0x000784A3, // Synth: Set FREF = 3.43 MHz (24 MHz / 7)
    0xA47E0583, // Synth: Set loop bandwidth after lock to 80 kHz (K2)
    0xEAE00603, // Synth: Set loop bandwidth after lock to 80 kHz (K3, LSB)
    0x00010623, // Synth: Set loop bandwidth after lock to 80 kHz (K3, MSB)
    0x013800C3, // Use enhanced BLE shape
    /* Spur fix */
    0x02010403, // Synth: Use 24 MHz XOSC as synth clock, enable phase error discard feature
    0x40014035, // Synth: Set phase error error discard count to 1 (default 2) to get faster settling in TX (address)
    0x177F0408, // Synth: Set phase error error discard count to 1 (default 2) to get faster settling in TX (value)
    0x38000463, // Synth: Modify phase error discard threshold factor and offset
    /* Increase tone length due to spur fix */
    0x036052AC, // Add 6 us to tone in front of packet
    0x01AD02A3, // Compensate for 6 us added to tone in front of packet
    0x01680263, // Compensate for 6 us added to tone in front of packet
    0xFFFFFFFF,   // End of override list
};


typedef struct pDataEntryQ_t {
    uint32_t pCurrEntry;
    uint32_t pLastEntry;
} pDataEntryQ;

typedef struct rf_radioOpCmd_t {
    uint16_t commandNo;
    uint16_t status;
    uint32_t pNextOp;
    uint32_t startTime;
    uint8_t startTrigger;
    uint8_t condition;
} rf_radioOpCmd;

typedef struct rf_commandRadioSetup_t {
    uint16_t commandNo;
    uint16_t status;
    uint32_t pNextOp;
    uint32_t startTime;
    uint8_t startTrigger;
    uint8_t condition;
    uint8_t mode;
    uint8_t ioDivider;
    uint16_t config;
    uint16_t txPower;
    uint32_t pRegOverride;
} rf_commandRadioSetup;

typedef struct rf_commandFrequencySynthesizer_t {
    uint16_t commandNo;
    uint16_t status;
    uint32_t pNextOp;
    uint32_t startTime;
    uint8_t startTrigger;
    uint8_t condition;
    uint16_t frequency;
    uint16_t fractFreq;
    uint8_t synthConf;
    uint8_t __dummy0;
    uint32_t __dummy1;
} rf_commandFrequencySynthesizer;

typedef struct rf_bleAdvPar_t {
    uint32_t pRxQ;
    uint8_t rxConfig;
    uint8_t advConfig;
    uint8_t advLen;
    uint8_t scanRspLen;
    uint32_t pAdvData;
    uint32_t pScanRspData;
    uint32_t pDeviceAddress;
    uint32_t pWhiteList;
    uint16_t __dummy0;
    uint8_t __dummy1;
    uint8_t endTrigger;
    uint32_t endTime;
} rf_bleAdvPar;

typedef struct rf_bleAdvOutput_t {
    uint16_t nTxAdvInd;
    uint8_t nTxScanRsp;
    uint8_t nRxScanReq;
    uint8_t nRxConnectReq;
    uint8_t __dummy0;
    uint16_t nRxNok;
    uint16_t nRxIgnored;
    uint8_t nRxBufFull;
    int8_t lastRssi;
    uint32_t timeStamp;
} rf_bleAdvOutput;

typedef struct rf_bleCommand_t {
    uint16_t commandNo;
    uint16_t status;
    uint32_t pNextOp;
    uint32_t startTime;
    uint8_t startTrigger;
    uint8_t condition;
    uint8_t channel;
    uint8_t whitening;
    uint32_t pParams;
    uint32_t pOutput;
} rf_bleCommand;

typedef struct rf_CMD_BLE_ADV_NC_t {
    uint16_t commandNo;
    uint16_t status;
    uint32_t pNextOp;
    uint32_t startTime;
    struct {
        uint8_t triggerType:4;
        uint8_t bEnaCmd:1;
        uint8_t triggerNo:2;
        uint8_t pastTrig:1;
    } startTrigger;
    struct {
        uint8_t rule:4;
        uint8_t nSkip:4;
    } condition;
    uint8_t channel;
    struct {
        uint8_t init:7;
        uint8_t bOverride:1;
    } whitening;
    uint32_t pParams;
    uint32_t pOutput;
} rf_CMD_BLE_ADV_NC;

static uint32_t ui32AuxClocks = AUX_WUC_ADI_CLOCK | AUX_WUC_OSCCTRL_CLOCK |
                                AUX_WUC_TDCIF_CLOCK | AUX_WUC_SOC_CLOCK |
                                AUX_WUC_TIMER_CLOCK | AUX_WUC_AIODIO0_CLOCK |
                                AUX_WUC_AIODIO1_CLOCK | AUX_WUC_SMPH_CLOCK |
                                AUX_WUC_TDC_CLOCK | AUX_WUC_ADC_CLOCK |
                                AUX_WUC_REF_CLOCK;

void ble_setup(void);
void ble_advertise(void);
void pwr_setup(void);
void pwr_down(void);
void led_setup(void);
void led_toggle(void);
void pin_setup(void);
void pin_irq (void);
void tmr_setup(void);

static uint8_t bleAdvPayload[] = {
    2, BLE_ADV_TYPE_DEVINFO, 0x06,
    // 2, BLE_ADV_TYPE_TX_POWER_LVL, 0,
    8, BLE_ADV_TYPE_NAME, '1', '2', '3', '4', 'S', '0', '5'
};
static uint8_t devAddress[] = {0xEE,0xEE,0xEE,0xEE,0xEE,0xEE};
#define adv_interval 3


int main (void) {
    pwr_setup();
    led_setup();
    led_toggle();
    ble_setup();
    ble_advertise();
    led_toggle();
    pin_setup();
    tmr_setup();
    while(1) {
        pwr_down();
    }
    return 0;
}

void tmr_setup() {
    AONEventMcuWakeUpSet(AON_EVENT_MCU_WU3, AON_EVENT_RTC2);
    AONRTCCombinedEventConfig(AON_RTC_CH2);
    AONRTCModeCh1Set(AON_RTC_MODE_CH2_CONTINUOUS);
    AONEventRtcSet(AON_EVENT_AUXWUSEL_WU0_EV_RTC_CH2);
    AONRTCCompareValueSet(AON_RTC_CH2, adv_interval*0x10000);
    AONRTCChannelEnable(AON_RTC_CH2);
    AONRTCEnable();
    IntEnable(INT_AON_RTC);
    SysCtrlAonSync();
    IntMasterEnable();
}

void AONRTCIntHandler(void) {
    SysCtrlAonUpdate();
    pwr_setup();
    led_setup();
    led_toggle();
    ble_setup();
    ble_advertise();
    led_toggle();
    AONRTCEventClear(AON_RTC_CH2);
    AONRTCReset();
}

void pin_setup() {
#ifdef SensorTag
    IOCPortConfigureSet(IOID_0, IOC_PORT_MCU_PORT_EVENT0, IOC_INPUT_ENABLE | IOC_IOPULL_UP
                        | IOC_INT_ENABLE | IOC_WAKE_ON_LOW | IOC_FALLING_EDGE | IOC_IOMODE_NORMAL
                        | IOC_HYST_ENABLE);
    GPIODirModeSet(1 << IOID_0, GPIO_DIR_MODE_IN);
#else
    IOCPortConfigureSet(IOID_10, IOC_PORT_MCU_PORT_EVENT0, IOC_INPUT_ENABLE | IOC_IOPULL_UP
                        | IOC_INT_ENABLE | IOC_WAKE_ON_LOW | IOC_FALLING_EDGE | IOC_IOMODE_NORMAL
                        | IOC_HYST_ENABLE);
    GPIODirModeSet(1 << IOID_10, GPIO_DIR_MODE_IN);
#endif // SensorTag

    IOCIntRegister(pin_irq);
}

void pin_irq(void) {
    SysCtrlAonUpdate();
    pwr_setup();
    led_setup();
    led_toggle();
    ble_setup();
    ble_advertise();
    led_toggle();
#ifdef SensorTag
    IOCIntClear(IOID_0);
#else
    IOCIntClear(IOID_10);
#endif // SensorTag


}

void led_setup() {
#ifdef SensorTag
    IOCPinTypeGpioOutput(IOID_10);
    IOCPinTypeGpioOutput(IOID_15);
    GPIOPinWrite(1 << IOID_10, 0); // RED
    GPIOPinWrite(1 << IOID_15, 0); // GREEN
#else // SensorTag
    IOCPinTypeGpioOutput(IOID_14);
    GPIOPinWrite(1 << IOID_14, 1 << IOID_14); // BLUE
#endif // SensorTag
}

void led_toggle() {
#ifdef SensorTag
    GPIOPinToggle(1 << IOID_15);
#else // SensorTag
    GPIOPinToggle(1 << IOID_14);
#endif // SensorTag
}

void pwr_setup() {
    HWREG(PRCM_BASE + PRCM_O_RFCMODESEL) = PRCM_RFCMODESEL_CURR_MODE1;
    // SysCtrlPowerEverything();
    AONWUCAuxWakeupEvent(AONWUC_AUX_WAKEUP);
    while(!(AONWUCPowerStatus() & AONWUC_AUX_POWER_ON));
    AUXWUCClockEnable(ui32AuxClocks);
    while(AUXWUCClockStatus(ui32AuxClocks) != AUX_WUC_CLOCK_READY);
    OSCClockSourceSet(OSC_SRC_CLK_MF | OSC_SRC_CLK_HF, OSC_XOSC_HF);
    OSCClockSourceSet(OSC_SRC_CLK_LF, OSC_XOSC_LF);
    if(OSCClockSourceGet(OSC_SRC_CLK_HF) != OSC_XOSC_HF) {
        OSCHfSourceSwitch();
    }
    PRCMPowerDomainOn(PRCM_DOMAIN_RFCORE | PRCM_DOMAIN_SERIAL |
                      PRCM_DOMAIN_PERIPH);
    while(PRCMPowerDomainStatus(PRCM_DOMAIN_RFCORE | PRCM_DOMAIN_SERIAL |
                                PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON);
    PRCMClockConfigureSet(PRCM_DOMAIN_SYSBUS | PRCM_DOMAIN_CPU |
                          PRCM_DOMAIN_PERIPH | PRCM_DOMAIN_SERIAL |
                          PRCM_DOMAIN_TIMER, PRCM_CLOCK_DIV_1);
    PRCMDomainEnable(PRCM_DOMAIN_RFCORE | PRCM_DOMAIN_VIMS);
    HWREG(RFC_PWR_NONBUF_BASE + RFC_PWR_O_PWMCLKEN) = 0x7FF;
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
    PRCMPeripheralSleepEnable(PRCM_PERIPH_GPIO);
    PRCMPeripheralDeepSleepEnable(PRCM_PERIPH_GPIO);
    PRCMPeripheralRunEnable(PRCM_PERIPH_TIMER0);
    PRCMPeripheralSleepEnable(PRCM_PERIPH_TIMER0);
    PRCMPeripheralDeepSleepEnable(PRCM_PERIPH_TIMER0);
    PRCMLoadSet();
    while(!PRCMLoadGet());
}

void pwr_down() {
    AUXWUCClockDisable(ui32AuxClocks);
    while(AUXWUCClockStatus(ui32AuxClocks) != AUX_WUC_CLOCK_OFF);
    HWREG(PRCM_BASE+PRCM_O_PDCTL1VIMS) &= ~PRCM_PDCTL1VIMS_ON;
    HWREG( PRCM_BASE + PRCM_O_RAMRETEN ) |= PRCM_RAMRETEN_VIMS_M;
    AONWUCMcuSRamConfig(MCU_RAM0_RETENTION | MCU_RAM1_RETENTION |
                        MCU_RAM2_RETENTION | MCU_RAM3_RETENTION);
    PRCMPowerDomainOff(PRCM_DOMAIN_CPU | PRCM_DOMAIN_RFCORE | PRCM_DOMAIN_SERIAL |
                       PRCM_DOMAIN_PERIPH);
    while(PRCMPowerDomainStatus(PRCM_DOMAIN_CPU | PRCM_DOMAIN_RFCORE | PRCM_DOMAIN_SERIAL |
                                PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_OFF);
    OSCInterfaceEnable();
    OSCClockSourceSet(OSC_SRC_CLK_LF,OSC_XOSC_LF);
    OSCInterfaceDisable();
    AONWUCMcuPowerDownConfig(AONWUC_CLOCK_SRC_LF);
    AONWUCAuxPowerDownConfig(AONWUC_CLOCK_SRC_LF);
    AONWUCRechargeCtrlConfigSet(true, 34, 2500, 5000);
    AUXWUCPowerCtrl(AUX_WUC_POWER_DOWN);
    while(AONWUCPowerStatus() & AONWUC_AUX_POWER_ON);
    PowerCtrlSourceSet(PWRCTRL_PWRSRC_ULDO);
    SysCtrlAonSync();
    PRCMDeepSleep();
}

void ble_setup() {
    rf_commandRadioSetup SET;
    SET.commandNo = CMD_RADIO_SETUP;
    SET.startTime = 0;
    SET.startTrigger = 0;
    SET.condition = 1;
    SET.pNextOp = 0;
    SET.status = 0;
    SET.mode = 0;
    SET.ioDivider = 0;
    SET.config = 0;
#ifdef SensorTag
    SET.txPower = 0x9330; // 0x9330 0x3161
#else
    SET.txPower = 0x3161; // 0x9330 0x3161
#endif // SensorTag
    SET.pRegOverride = (uint32_t) pOverrides;
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = (uint32_t) &SET;
    while(!HWREGBITW(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG,0));
    HWREGBITW(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG,0) = 0;
    while(SET.status != 0x0002);
    while((SET.status & 0x0F00) != 0x0400);
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = directCmd(CMD_START_RAT);
    while(!HWREGBITW(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG,0));
    HWREGBITW(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG,0) = 0;

}

void ble_advertise() {
    rf_bleAdvPar params;
    params.pRxQ = 0;
    params.rxConfig = 0;
    params.advConfig = 0;
    params.advLen = sizeof(bleAdvPayload);// 0x0B
    params.scanRspLen = 0;
    params.pAdvData = (uint32_t) &bleAdvPayload;
    params.pScanRspData = 0;
    params.pDeviceAddress = (uint32_t) &devAddress;
    params.pWhiteList = 0;
    params.endTrigger = 1;
    params.endTime = 0;
    rf_bleCommand CMD;
    CMD.commandNo = CMD_BLE_ADV_NC;
    CMD.status = 0;
    CMD.condition = 1;
    CMD.pNextOp = 0;
    CMD.startTime = 0;
    CMD.startTrigger = 0;
    CMD.pOutput = 0;//(uint32_t)&out;
    CMD.pParams = (uint32_t)&params;
/////////////////////////////////////////////////////////////////
    CMD.channel = 0x66; // 0x66 0x7E 0xB4
    CMD.whitening = 0xE5; // 0xE5 0xE6 0xE7
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = (uint32_t) &CMD;
    while(!HWREGBITW(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG,0));
    HWREGBITW(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG,0) = 0;
    while(CMD.status != 0x0002);
    while((CMD.status & 0xFF00) != 0x1400);
///////////////////////////////////////////////////////////////
    CMD.channel = 0x7E; // 0x66 0x7E 0xB4
    CMD.whitening = 0xE6; // 0xE5 0xE6 0xE7
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = (uint32_t) &CMD;
    while(!HWREGBITW(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG,0));
    HWREGBITW(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG,0) = 0;
    while(CMD.status != 0x0002);
    while((CMD.status & 0xFF00) != 0x1400);
///////////////////////////////////////////////////////////////
    CMD.channel = 0xB4; // 0x66 0x7E 0xB4
    CMD.whitening = 0xE7; // 0xE5 0xE6 0xE7
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = (uint32_t) &CMD;
    while(!HWREGBITW(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG,0));
    HWREGBITW(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG,0) = 0;
    while(CMD.status != 0x0002);
    while((CMD.status & 0xFF00) != 0x1400);
}
