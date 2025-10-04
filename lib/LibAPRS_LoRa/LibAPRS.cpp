#include <RadioLib.h>
#include "Arduino.h"
#include "AX25.h"
#include "LibAPRSesp.h"
#include "ais.h"
#include <WiFiClient.h>
#include "RadioHal.hpp"

ICACHE_RAM_ATTR IRadioHal *radioHal;
#ifdef RF2
ICACHE_RAM_ATTR IRadioHal *radioHal1;
#endif

extern float rssi;
extern float snr;
extern float freqErr;
extern bool afskSync;
extern statusType status;

// float _freq;

AX25Ctx AX25;
extern void aprs_msg_callback(struct AX25Msg *msg);

uint8_t chip_type = RF_SX1262;

extern Configuration config;

extern WiFiClient aprsClient;

// Radio::Radio()
#if CONFIG_IDF_TARGET_ESP32C3
SPIClass spi(FSPI);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
SPIClass spi(HSPI);
#else
SPIClass spi(VSPI);
#endif

#define countof(a) sizeof(a) / sizeof(a[0])

TaskHandle_t fifoTaskHandle;
TaskHandle_t fifoTaskHandle1;

unsigned long custom_preamble = 350UL;
unsigned long custom_tail = 50UL;

AX25Call src;
AX25Call dst;
AX25Call path1;
AX25Call path2;

char CALL[7] = "NOCALL";
int CALL_SSID = 0;
char DST[7] = "APE32L";
int DST_SSID = 0;
char PATH1[7] = "WIDE1";
int PATH1_SSID = 1;
char PATH2[7] = "WIDE2";
int PATH2_SSID = 2;

AX25Call path[8];

// Location packet assembly fields
char latitude[9];
char longtitude[10];
char symbolTable = '/';
char symbol = 'n';

uint8_t power = 10;
uint8_t height = 10;
uint8_t gain = 10;
uint8_t directivity = 10;
/////////////////////////

// Message packet assembly fields
char message_recip[7];
int message_recip_ssid = -1;

int message_seq = 0;
char lastMessage[67];
size_t lastMessageLen;
bool message_autoAck = false;
/////////////////////////

// (2^31 / 180) / 380926 semicircles per Base 91 unit
static unsigned long b91val[4] = {23601572L, 259358L, 2851L, 32L};
// Constants for converting lat/lon to semicircles
static long valtable[] = {1193046471L, 119304647L, 11930465L, 1988411L, 198841L, 19884L, 1988L, 199L, 20L, 2L};

bool received = false;
bool eInterrupt = true;
bool noisyInterrupt = false;

unsigned long rxTimeout = 0;

#define MAX_RFBUFF 255

uint8_t *rxBuff = NULL;
int rxBuffLen = 0;
bool flagGetFifo = false;
uint8_t *txBufPtr;
size_t txBuf_len;
int remLength;
bool flagAddFifo = false;

#ifdef RF2
bool received1 = false;
bool eInterrupt1 = true;
bool noisyInterrupt1 = false;
uint8_t *rx1Buff = NULL;
int rx1BuffLen = 0;
bool flagGetFifo1 = false;
bool flagAddFifo1 = false;
uint8_t *txBufPtr1;
size_t txBuf_len1;
int remLength1;
unsigned long rxTimeout1 = 0;
bool fifoLock1 = false;
bool fifoTxLock1 = false;
#endif

bool fifoLock = false;
bool fifoTxLock = false;

float fskRSSI = -130;

// uint32_t LFSR_ORDER[]={0xFFFFFFFF,0x1FFFF,0x1FFFF01,0xFFFF010E,0xFF010E91,0x10E916F,0xE916F5F,0x916F5F43,0x6F5F435A,0x5F435A95,0x435A9505,0x5A95051B,0x95051BD2,0x51BD231};

ICACHE_RAM_ATTR void fifoAdd(void)
{
    // log_d("Interrupt DIO1");
    // portENTER_CRITICAL_ISR(&fifoMux); // ISR start
    // while(flagAddFifo) delay(1);
    if (ax25_stateTx && remLength > 0)
        flagAddFifo = true;
    // portEXIT_CRITICAL_ISR(&fifoMux); // ISR end
    // log_d("TX: %u",millis());
    // if (fifoTxLock == false && remLength>0)
    // {
    //     if (ax25_stateTx)
    //     {
    //         fifoTxLock = true;
    //         radioHal->fifoAdd(txBuff, txBuf_len, &remLength);
    //         //vTaskDelay(pdTICKS_TO_MS(1));
    //         //log_d("remLength: %i",remLength);
    //     }
    //     fifoTxLock = false;
    // }
}

ICACHE_RAM_ATTR void fifoGet(void)
{
    if (!ax25_stateTx)
    {
        flagGetFifo = true;
        // log_d("RX: %u",millis());
    }
}

ICACHE_RAM_ATTR void setFlag()
{
    // log_d("Interrupt DIO0 milis=%u", millis());
    if (received || !eInterrupt)
        noisyInterrupt = true;

    if (!eInterrupt)
        return;

    received = true;
}

ICACHE_RAM_ATTR void enableInterrupt()
{
    eInterrupt = true;
}

ICACHE_RAM_ATTR void disableInterrupt()
{
    eInterrupt = false;
}

void radioSleep()
{
    radioHal->sleep();
}

void startRx()
{
    received = false;
    flagGetFifo = false;

    // if (config.rf_type == RF_SX1272 || config.rf_type == RF_SX1273 || config.rf_type == RF_SX1276 || config.rf_type == RF_SX1278 || config.rf_type == RF_SX1279)
    //{
    if (rxBuff == NULL)
    {
        rxBuff = (uint8_t *)calloc(MAX_RFBUFF, sizeof(uint8_t));
        memset(rxBuff, 0, MAX_RFBUFF);
        rxBuffLen = 0;
    }
    else
    {
        memset(rxBuff, 0, MAX_RFBUFF);
        rxBuffLen = 0;
    }
    //}

    if (config.rf_mode == RF_MODE_G3RUH)
    {
        // uint8_t syncWord[] = {0x45,0x13,0xf2,0xb7}; //VX8-DR Delay 100mS,Last byte[0xec,0xef,0xb9,0x8d,0x45,0x13,0xf2,0xb7,0xd3,0x27,0xda,0xef,0x42]
        // uint8_t syncWord[] = {0x3f,0xed}; //FTM350 Delay 100mS,Last byte[0xf5,0xd4,0xd9,0x59,0x7,0xc2,0x1,0x3f,0xed,0x9f,0xd1,0xcc,0xa,0xd8,0x56,0x17,0xb4,0x8e]
        // uint8_t syncWord[] = {0x4, 0x1, 0x43};
        uint8_t syncWord[] = {0xd9};
        radioHal->setSyncWord(syncWord, sizeof(syncWord));
        rxTimeout = millis() + 60000;
        // if (config.rf_type == RF_SX1272 || config.rf_type == RF_SX1273 || config.rf_type == RF_SX1276 || config.rf_type == RF_SX1278 || config.rf_type == RF_SX1279)
        //{

        // rxTimeout = millis() + 10000;
        //      radioHal->setFifoFullAction(fifoGet);
        //      radioHal->fixedPacketLengthMode(0);
        //  }else if (config.rf_type == RF_SX1261 || config.rf_type == RF_SX1262 || config.rf_type == RF_SX1268 || config.rf_type == RF_SX126x)
        //  {
        //      radioHal->fixedPacketLengthMode(RADIOLIB_SX126X_MAX_PACKET_LENGTH);
        //}
    }
    else
    {
        rxTimeout = millis() + 900000;
    }
    // else if (config.rf_mode == RF_MODE_AIS)
    //{
    //  uint8_t syncWord[] = {0xcc, 0xcc, 0xcc};
    //  radioHal->setSyncWord(syncWord, 3);
    //      radioHal->setCRC(0);
    //      //radioHal->fixedPacketLengthMode(RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK); // payload[168bit:21Byte]+CRC[2Byte]+Flag7E[1Byte]
    //      if (config.rf_type == RF_SX1261 || config.rf_type == RF_SX1262 || config.rf_type == RF_SX1268 || config.rf_type == RF_SX126x)
    //          radioHal->fixedPacketLengthMode(RADIOLIB_SX126X_MAX_PACKET_LENGTH);
    //      else
    //          radioHal->fixedPacketLengthMode(RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK);
    //}

    enableInterrupt();
    // put module back to listen mode
    radioHal->startReceive();
    // we're ready to receive more packets,
    // enable interrupt service routine
    // radioHal->setDio0Action(setFlag);
}

void radioRecvStatus()
{
    if (config.rf_mode == RF_MODE_LoRa)
    {
        rssi = radioHal->getRSSI(true, false);
        snr = radioHal->getSNR();
        freqErr = radioHal->getFrequencyError();
        if ((config.rf_type == RF_SX1261) || (config.rf_type == RF_SX1262) || (config.rf_type == RF_SX1268) || (config.rf_type == RF_SX126x))
        {
            // SX126x RSSI is not reliable, so we estimate it
            if((int)rssi == -127){
                float noiseFloor = -174 + 10*log10(config.rf_bw*1000.0) + 6; // dBm, BW=125kHz, NF=6dB
                rssi = noiseFloor+snr;
            }
        }
        // print RSSI (Received Signal Strength Indicator)
        log_d("[LoRa] RSSI:%.1f dBm\tSNR:%.1f dBm\tFreqErr:%.1f Hz", rssi, snr, freqErr);
    }
    else
    {
        rssi = radioHal->getRSSI(false, true);
        if (rssi > fskRSSI)
            fskRSSI = rssi;
        // log_d("[GFSK] RSSI:%.0f dBm", rssi);
    }
    afskSync = true;
}

#ifdef RF2
ICACHE_RAM_ATTR void fifoAdd1(void)
{
    if (ax25_stateTx && remLength > 0)
        flagAddFifo1 = true;
}

ICACHE_RAM_ATTR void fifoGet1(void)
{
    if (!ax25_stateTx)
    {
        flagGetFifo1 = true;
        // log_d("RX: %u",millis());
    }
}

ICACHE_RAM_ATTR void setFlag1()
{
    log_d("RF2 Interrupt DIO0 milis=%u", millis());
    if (received1 || !eInterrupt1)
        noisyInterrupt1 = true;

    if (!eInterrupt1)
        return;

    received1 = true;
}

ICACHE_RAM_ATTR void enableInterrupt1()
{
    eInterrupt1 = true;
}

ICACHE_RAM_ATTR void disableInterrupt1()
{
    eInterrupt1 = false;
}

void radioSleep1()
{
    radioHal1->sleep();
}

void startRx1()
{
    received1 = false;
    flagGetFifo1 = false;

    if (rx1Buff == NULL)
    {
        rx1Buff = (uint8_t *)calloc(MAX_RFBUFF, sizeof(uint8_t));
        memset(rx1Buff, 0, MAX_RFBUFF);
        rx1BuffLen = 0;
    }
    else
    {
        memset(rx1Buff, 0, MAX_RFBUFF);
        rx1BuffLen = 0;
    }

    if (config.rf1_mode == RF_MODE_G3RUH)
    {
        uint8_t syncWord[] = {0xd9};
        radioHal1->setSyncWord(syncWord, sizeof(syncWord));
        rxTimeout1 = millis() + 60000;
    }
    else
    {
        rxTimeout1 = millis() + 900000;
    }

    enableInterrupt1();
    // put module back to listen mode
    radioHal1->startReceive();
}

void radioRecvStatus1()
{
    if (config.rf1_mode == RF_MODE_LoRa)
    {
        rssi = radioHal1->getRSSI(true, false);
        snr = radioHal1->getSNR();
        freqErr = radioHal1->getFrequencyError();
        // print RSSI (Received Signal Strength Indicator)
        log_d("[LoRa1] RSSI:%.0f dBm\tSNR:%.0f dBm\tFreqErr:%.0f Hz", rssi, snr, freqErr);
    }
    else
    {
        rssi = radioHal1->getRSSI(false, true);
        if (rssi > fskRSSI)
            fskRSSI = rssi;
        // log_d("[GFSK] RSSI:%.0f dBm", rssi);
    }
    afskSync = true;
}

void taskADDFifo1(void *pvParameters)
{
    for (;;)
    {
        if (ax25_stateTx)
        {
            if (flagAddFifo1 && remLength > 0)
            {

                // portENTER_CRITICAL_ISR(&fifoMux); // ISR start
                // flagAddFifo=false;
                // portEXIT_CRITICAL_ISR(&fifoMux); // ISR end
                // log_d("TX:%u remLen:%i",millis(),remLength);
                radioHal1->fifoAdd(txBufPtr, txBuf_len, &remLength);
                flagAddFifo = false;
                continue;
            }
            vTaskDelay(pdTICKS_TO_MS(1));
        }
        else
        {
            if (flagGetFifo1)
            {
                // fifoLock = true;
                if (rx1Buff == NULL)
                {
                    vTaskDelay(pdTICKS_TO_MS(10));
                    continue;
                }
                if (rx1BuffLen == 0)
                    radioRecvStatus();
                if (radioHal1->fifoGet(rx1Buff, MAX_RFBUFF, &rx1BuffLen))
                {
                    received1 = true;
                    // fifoLock = false;
                    // radioRecvStatus();
                }
                flagGetFifo1 = false;
                continue;
            }
            vTaskDelay(pdTICKS_TO_MS(1));
        }
    }
}
#endif

// void APRS_sendTNC2Pkt(String raw)
void APRS_sendTNC2Pkt(uint8_t *raw, size_t length)
{
    if (config.rf_ax25)
    {
        ax25frame *frame = (ax25frame *)calloc(1, sizeof(ax25frame));
        if (frame)
        {
            if (ax25_encode(*frame, (char *)raw, length))
            {
                size_t max_len = length + 10;
                uint8_t *hdlc = (uint8_t *)calloc(max_len, sizeof(uint8_t));
                if (hdlc)
                {
                    memset(hdlc, 0, max_len);
                    int len = hdlcFrame(hdlc, max_len, &AX25, frame);
                    // for(int i=0;i<len;i++) log_d("[%d]0x%0x",i,frame_ax25[i]);
                    ax25_sendBuf(hdlc, len);
                    free(hdlc);
                }
            }
            free(frame);
            // Serial.println("ax25 send sussesed...");
        }
    }
    else
    {
        if (config.rf_mode == RF_MODE_LoRa)
        {
            uint8_t *lora = (uint8_t *)calloc(length + 3, sizeof(uint8_t));
            if (lora)
            {
                lora[0] = 0x3C;
                lora[1] = 0xff;
                lora[2] = 0x01;
                for (int i = 0; i < length; i++)
                    lora[i + 3] = raw[i];
                ax25_sendBuf(lora, length + 3);
                free(lora);
            }
        }
        else
        {
            ax25_sendBuf(raw, length);
        }
    }
}

int APRS_getTNC2Pkt(uint8_t *raw, String info)
{
    ax25frame frame;
    // Serial.println(raw);
    ax25_init(&AX25, aprs_msg_callback);
    ax25_encode(frame, (char *)info.c_str(), info.length());
    int sz = hdlcFrame(raw, 250, &AX25, &frame);
    return sz;
}

int APRS_getTNC2(String info)
{
    uint8_t raw[300];
    ax25frame frame;
    // Serial.println(raw);
    ax25_init(&AX25, aprs_msg_callback);
    ax25_encode(frame, (char *)info.c_str(), info.length());
    int sz = hdlcFrame(raw, 300, &AX25, &frame);
    if (sz > 0)
        ax25_decode(&AX25);
    return sz;
}

void APRS_setFreq(float freq)
{
    // radioHal->setFrequency(freq);
    // radioHal->startReceive();
}

bool APRS_init(Configuration *cfg)
{
    bool ret = true;
    int state = -1;
    // if (config.rf_en == false)
    //     return false;

    // if (config.rf_type == 0)
    //     return true;

    // if (config.rf_type < 0 || config.rf_type > 13)
    //     return false;

    // SX127x
    // setDio0Action->getIRQ  (PayloadReady,PacketSent)
    // setDio1Action->getGPIO  (FifoLeve,FifoEmpty,FifoFull)

    // SX126x,SX128x
    // setDio0Action->setDio1Action->getIRQ
    pinMode(config.rf_nss_gpio, OUTPUT);
    spi.begin(config.rf_sclk_gpio, config.rf_miso_gpio, config.rf_mosi_gpio);
    if (config.rf_en)
    {
        log_d("[SX12xx] Initializing .. ");
        if (cfg->rf_type == RF_SX1278)
        {
            log_d("Init chip SX1278");
            radioHal = new RadioHal<SX1278>(new Module(config.rf_nss_gpio, config.rf_dio0_gpio, config.rf_reset_gpio, config.rf_dio1_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf_type == RF_SX1272)
        {
            log_d("Init chip SX1272");
            radioHal = new RadioHal<SX1272>(new Module(config.rf_nss_gpio, config.rf_dio0_gpio, config.rf_reset_gpio, config.rf_dio1_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf_type == RF_SX1273)
        {
            log_d("Init chip SX1273");
            radioHal = new RadioHal<SX1273>(new Module(config.rf_nss_gpio, config.rf_dio0_gpio, config.rf_reset_gpio, config.rf_dio1_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf_type == RF_SX1276)
        {
            log_d("Init chip SX1276");
            radioHal = new RadioHal<SX1276>(new Module(config.rf_nss_gpio, config.rf_dio0_gpio, config.rf_reset_gpio, config.rf_dio1_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf_type == RF_SX1279)
        {
            log_d("Init chip SX1279");
            radioHal = new RadioHal<SX1279>(new Module(config.rf_nss_gpio, config.rf_dio0_gpio, config.rf_reset_gpio, config.rf_dio1_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf_type == RF_SX1268)
        {
            log_d("Init chip SX1268");
            radioHal = new RadioHal<SX1268>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf_type == RF_SX1261)
        {
            log_d("Init chip SX1262");
            radioHal = new RadioHal<SX1261>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf_type == RF_SX1262)
        {
            log_d("Init chip SX1262");
            radioHal = new RadioHal<SX1262>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf_type == RF_SX1280)
        {
            log_d("Init chip SX1280");
            radioHal = new RadioHal<SX1280>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf_type == RF_SX1281)
        {
            log_d("Init chip SX1281");
            radioHal = new RadioHal<SX1281>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf_type == RF_SX1282)
        {
            log_d("Init chip SX1282");
            radioHal = new RadioHal<SX1282>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        // else if (cfg->rf_type == RF_SX1231)
        // {
        //     log_d("Init chip SX1231");
        //     radioHal = new RadioHal<SX1231>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        // }
        // else if (cfg->rf_type == RF_SX1233)
        // {
        //     log_d("Init chip SX1233");
        //     radioHal = new RadioHal<SX1233>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        // }
        else
        {
            log_d("Init chip Unknow default SX1268");
            radioHal = new RadioHal<SX1268>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }

        if (radioHal == NULL)
            return false;

        if (cfg->rf_rx_gpio != -1 && config.rf_tx_gpio != -1)
        {
            radioHal->setRfSwitchPins(config.rf_rx_gpio, config.rf_tx_gpio);
            log_d("setRfSwitchPins(RxEn->GPIO %d, TxEn->GPIO %d)", config.rf_rx_gpio, config.rf_tx_gpio);
        }

        log_d("[%d] Begin... ", config.rf_type);
        if ((cfg->rf_type == RF_SX1231) || (cfg->rf_type == RF_SX1233) || (config.rf_mode == RF_MODE_G3RUH) || (config.rf_mode == RF_MODE_AIS) || (config.rf_mode == RF_MODE_GFSK))
        {
            // state = radioHal->beginFSK(config.rf_freq + (config.rf_freq_offset / 1000000.0), config.rf_baudrate, config.rf_bw, config.rf_bw, config.rf_power, config.rf_preamable, 0, 1.6);
            state = radioHal->beginFSK(config.rf_freq + (config.rf_freq_offset / 1000000.0), config.rf_br, config.rf_br * 0.25f, config.rf_bw, config.rf_power, config.rf_preamable, false, 1.6);
            if (state == RADIOLIB_ERR_NONE)
            {
                ret = true;
                log_d("FSK begin success!");
            }
            else
            {
                ret = false;
                log_d("failed, code %d", state);
            }
        }
        else
        {
            state = radioHal->begin(config.rf_freq + (config.rf_freq_offset / 1000000.0), config.rf_bw, config.rf_sf, config.rf_cr, config.rf_sync, config.rf_power, config.rf_preamable, 1, 1.6);
            if (state == RADIOLIB_ERR_NONE)
            {
                ret = true;
                log_d("[LoRa] Init success!");
            }
            else
            {
                ret = false;
                log_d("[LoRa] Init failed, code %d", state);
            }
        }

        if (config.rf_mode == RF_MODE_G3RUH || config.rf_mode == RF_MODE_GFSK)
        {
            if (config.rf_mode == RF_MODE_G3RUH)
            {
                uint8_t syncWord[] = {0xd9};
                radioHal->setSyncWord(syncWord, sizeof(syncWord));
                radioHal->setCRC(0);
                log_d("Set syncWord GFSK(G3RUH) success!");
                if (state != RADIOLIB_ERR_NONE)
                {
                    ret = false;
                    log_d("Unable to set configuration, code %i", state);
                }
            }
            else
            {
                uint8_t syncWordRX[] = {0x84, 0xB5, 0x12, 0xAD};
                radioHal->setSyncWord(syncWordRX, 4);
            }
            state = radioHal->setDataShaping(config.rf_shaping);
            state = radioHal->setEncoding(config.rf_encoding);
            if (config.rf_type == RF_SX1272 || config.rf_type == RF_SX1273 || config.rf_type == RF_SX1276 || config.rf_type == RF_SX1278 || config.rf_type == RF_SX1279)
            {

                if (rxBuff == NULL)
                {
                    rxBuff = (uint8_t *)calloc(MAX_RFBUFF, sizeof(uint8_t));
                    memset(rxBuff, 0, MAX_RFBUFF);
                    rxBuffLen = 0;
                }
                if (config.rf_dio1_gpio > -1)
                { // Use DIO1 pin connected for fifo interrupt
                    if (fifoTaskHandle == NULL)
                    {
                        xTaskCreatePinnedToCore(
                            taskADDFifo,     /* Function to implement the task */
                            "taskADDFifo",   /* Name of the task */
                            2048,            /* Stack size in words */
                            NULL,            /* Task input parameter */
                            5,               /* Priority of the task */
                            &fifoTaskHandle, /* Task handle. */
                            0);              /* Core where the task should run */
                    }
                    radioHal->setFifoFullAction(fifoGet);
                    radioHal->fixedPacketLengthMode(0);
                }
                else
                {
                    radioHal->fixedPacketLengthMode(RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK);
                    radioHal->setDio0Action(setFlag);
                }
            }
            else if (config.rf_type == RF_SX1261 || config.rf_type == RF_SX1262 || config.rf_type == RF_SX1268 || config.rf_type == RF_SX126x)
            {
                if (fifoTaskHandle != NULL)
                    vTaskDelete(fifoTaskHandle);
                radioHal->fixedPacketLengthMode(RADIOLIB_SX126X_MAX_PACKET_LENGTH);
                radioHal->setDio0Action(setFlag);
            }
        }
        else if (config.rf_mode == RF_MODE_AIS)
        {
            if (fifoTaskHandle != NULL)
                vTaskDelete(fifoTaskHandle);
            // uint8_t syncWord[] = {0xcc, 0xcc, 0xcc, 0xfe}; // Flag HDLC 0x7EAAAAAA, 33,cc,99,
            uint8_t syncWord[] = {0xcc, 0xcc};
            radioHal->setSyncWord(syncWord, 2);
            radioHal->setCRC(0);
            log_d("Set syncWord FSK AIS success!");
            if (state != RADIOLIB_ERR_NONE)
            {
                ret = false;
                log_d("Unable to set configuration, code %i", state);
            }
            state = radioHal->setDataShaping(config.rf_shaping);
            state = radioHal->setEncoding(config.rf_encoding);
            if (config.rf_type == RF_SX1261 || config.rf_type == RF_SX1262 || config.rf_type == RF_SX1268 || config.rf_type == RF_SX126x)
                radioHal->fixedPacketLengthMode(RADIOLIB_SX126X_MAX_PACKET_LENGTH);
            else
                radioHal->fixedPacketLengthMode(RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK);

            radioHal->setDio0Action(setFlag);
        }
        else
        {
            radioHal->setDio0Action(setFlag);
        }

        if (config.rf_type == RF_SX1272 || config.rf_type == RF_SX1273 || config.rf_type == RF_SX1276 || config.rf_type == RF_SX1278 || config.rf_type == RF_SX1279)
        {
            radioHal->setCurrentLimit(140);
        }
        else
        {
            radioHal->setCurrentLimit(120);
        }
        
        radioHal->setOutputPower(config.rf_power);
        startRx();
        radioHal->setRxBoostedGainMode(config.rf_rx_boost);

    }

    ax25_init(&AX25, aprs_msg_callback);
    return ret;
}

#ifdef RF2
bool APRS_init2(Configuration *cfg)
{
    bool ret = true;
    int state = -1;

    pinMode(config.rf1_nss_gpio, OUTPUT);
    spi.begin(config.rf_sclk_gpio, config.rf_miso_gpio, config.rf_mosi_gpio);
    if (config.rf1_en)
    {
        log_d("[SX12xx] Module 2 Initializing .. ");
        if (cfg->rf1_type == RF_SX1278)
        {
            log_d("Init chip1 SX1278");
            radioHal1 = new RadioHal<SX1278>(new Module(config.rf1_nss_gpio, config.rf1_dio0_gpio, config.rf1_reset_gpio, config.rf1_dio1_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf1_type == RF_SX1272)
        {
            log_d("Init chip1 SX1272");
            radioHal1 = new RadioHal<SX1272>(new Module(config.rf1_nss_gpio, config.rf1_dio0_gpio, config.rf1_reset_gpio, config.rf1_dio1_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf1_type == RF_SX1273)
        {
            log_d("Init chip SX1273");
            radioHal1 = new RadioHal<SX1273>(new Module(config.rf1_nss_gpio, config.rf1_dio0_gpio, config.rf1_reset_gpio, config.rf1_dio1_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf1_type == RF_SX1276)
        {
            log_d("Init chip1 SX1276");
            radioHal1 = new RadioHal<SX1276>(new Module(config.rf1_nss_gpio, config.rf1_dio0_gpio, config.rf1_reset_gpio, config.rf1_dio1_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf1_type == RF_SX1279)
        {
            log_d("Init chip1 SX1279");
            radioHal1 = new RadioHal<SX1279>(new Module(config.rf1_nss_gpio, config.rf1_dio0_gpio, config.rf1_reset_gpio, config.rf1_dio1_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf1_type == RF_SX1268)
        {
            log_d("Init chip1 SX1268");
            radioHal1 = new RadioHal<SX1268>(new Module(config.rf1_nss_gpio, config.rf1_dio1_gpio, config.rf1_reset_gpio, config.rf1_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf1_type == RF_SX1261)
        {
            log_d("Init chip1 SX1262");
            radioHal1 = new RadioHal<SX1261>(new Module(config.rf1_nss_gpio, config.rf1_dio1_gpio, config.rf1_reset_gpio, config.rf1_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf1_type == RF_SX1262)
        {
            log_d("Init chip SX1262");
            radioHal1 = new RadioHal<SX1262>(new Module(config.rf1_nss_gpio, config.rf1_dio1_gpio, config.rf1_reset_gpio, config.rf1_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf1_type == RF_SX1280)
        {
            log_d("Init chip1 SX1280");
            radioHal1 = new RadioHal<SX1280>(new Module(config.rf1_nss_gpio, config.rf1_dio1_gpio, config.rf1_reset_gpio, config.rf1_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf1_type == RF_SX1281)
        {
            log_d("Init chip1 SX1281");
            radioHal1 = new RadioHal<SX1281>(new Module(config.rf1_nss_gpio, config.rf1_dio1_gpio, config.rf1_reset_gpio, config.rf1_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        else if (cfg->rf1_type == RF_SX1282)
        {
            log_d("Init chip1 SX1282");
            radioHal1 = new RadioHal<SX1282>(new Module(config.rf1_nss_gpio, config.rf1_dio1_gpio, config.rf1_reset_gpio, config.rf1_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }
        // else if (cfg->rf1_type == RF_SX1231)
        // {
        //     log_d("Init chip SX1231");
        //     radioHal1 = new RadioHal<SX1231>(new Module(config.rf1_nss_gpio, config.rf1_dio1_gpio, config.rf1_reset_gpio, config.rf1_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        // }
        // else if (cfg->rf1_type == RF_SX1233)
        // {
        //     log_d("Init chip SX1233");
        //     radioHal1 = new RadioHal<SX1233>(new Module(config.rf1_nss_gpio, config.rf1_dio1_gpio, config.rf1_reset_gpio, config.rf1_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        // }
        else
        {
            log_d("Init chip Unknow default SX1268");
            radioHal1 = new RadioHal<SX1268>(new Module(config.rf1_nss_gpio, config.rf1_dio1_gpio, config.rf1_reset_gpio, config.rf1_dio0_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
        }

        if (radioHal1 == NULL)
            return false;

        if (cfg->rf1_rx_gpio != -1 && config.rf1_tx_gpio != -1)
        {
            radioHal1->setRfSwitchPins(config.rf1_rx_gpio, config.rf1_tx_gpio);
            log_d("setRfSwitchPins(RxEn->GPIO %d, TxEn->GPIO %d)", config.rf1_rx_gpio, config.rf1_tx_gpio);
        }

        log_d("[%d] Begin... ", config.rf1_type);
        if ((cfg->rf1_type == RF_SX1231) || (cfg->rf1_type == RF_SX1233) || (config.rf1_mode == RF_MODE_G3RUH) || (config.rf1_mode == RF_MODE_AIS) || (config.rf1_mode == RF_MODE_GFSK))
        {
            // state = radioHal1->beginFSK(config.rf1_freq + (config.rf1_freq_offset / 1000000.0), config.rf_baudrate, config.rf1_bw, config.rf1_bw, config.rf1_power, config.rf1_preamable, 0, 1.6);
            state = radioHal1->beginFSK(config.rf1_freq + (config.rf1_freq_offset / 1000000.0), config.rf1_br, config.rf1_br * 0.25f, config.rf1_bw, config.rf1_power, config.rf1_preamable, false, 1.6);
            if (state == RADIOLIB_ERR_NONE)
            {
                ret = true;
                log_d("FSK begin success!");
            }
            else
            {
                ret = false;
                log_d("failed, code %d", state);
            }
        }
        else
        {
            state = radioHal1->begin(config.rf1_freq + (config.rf1_freq_offset / 1000000.0), config.rf1_bw, config.rf1_sf, config.rf1_cr, config.rf1_sync, config.rf1_power, config.rf1_preamable, 1, 1.6);
            if (state == RADIOLIB_ERR_NONE)
            {
                ret = true;
                log_d("[LoRa 2] Init success!");
            }
            else
            {
                ret = false;
                log_d("[LoRa 2] Init failed, code %d", state);
            }
        }

        if (config.rf1_mode == RF_MODE_G3RUH || config.rf1_mode == RF_MODE_GFSK)
        {
            if (config.rf1_mode == RF_MODE_G3RUH)
            {
                uint8_t syncWord[] = {0xd9};
                radioHal1->setSyncWord(syncWord, sizeof(syncWord));
                radioHal1->setCRC(0);
                log_d("Set syncWord GFSK(G3RUH) success!");
                if (state != RADIOLIB_ERR_NONE)
                {
                    ret = false;
                    log_d("Unable to set configuration, code %i", state);
                }
            }
            else
            {
                uint8_t syncWordRX[] = {0x84, 0xB5, 0x12, 0xAD};
                radioHal1->setSyncWord(syncWordRX, 4);
            }
            state = radioHal1->setDataShaping(config.rf1_shaping);
            state = radioHal1->setEncoding(config.rf1_encoding);
            if (config.rf1_type == RF_SX1272 || config.rf1_type == RF_SX1273 || config.rf1_type == RF_SX1276 || config.rf1_type == RF_SX1278 || config.rf1_type == RF_SX1279)
            {

                if (rx1Buff == NULL)
                {
                    rx1Buff = (uint8_t *)calloc(MAX_RFBUFF, sizeof(uint8_t));
                    memset(rx1Buff, 0, MAX_RFBUFF);
                    rx1BuffLen = 0;
                }
                if (config.rf1_dio1_gpio > -1)
                { // Use DIO1 pin connected for fifo interrupt
                    if (fifoTaskHandle1 == NULL)
                    {
                        xTaskCreatePinnedToCore(
                            taskADDFifo1,     /* Function to implement the task */
                            "taskADDFifo1",   /* Name of the task */
                            2048,             /* Stack size in words */
                            NULL,             /* Task input parameter */
                            5,                /* Priority of the task */
                            &fifoTaskHandle1, /* Task handle. */
                            0);               /* Core where the task should run */
                    }
                    radioHal1->setFifoFullAction(fifoGet);
                    radioHal1->fixedPacketLengthMode(0);
                }
                else
                {
                    radioHal1->fixedPacketLengthMode(RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK);
                    radioHal1->setDio0Action(setFlag1);
                }
            }
            else if (config.rf1_type == RF_SX1261 || config.rf1_type == RF_SX1262 || config.rf1_type == RF_SX1268 || config.rf1_type == RF_SX126x)
            {
                if (fifoTaskHandle1 != NULL)
                    vTaskDelete(fifoTaskHandle1);
                radioHal1->fixedPacketLengthMode(RADIOLIB_SX126X_MAX_PACKET_LENGTH);
                radioHal1->setDio0Action(setFlag1);
            }
        }
        else if (config.rf1_mode == RF_MODE_AIS)
        {
            if (fifoTaskHandle1 != NULL)
                vTaskDelete(fifoTaskHandle1);
            // uint8_t syncWord[] = {0xcc, 0xcc, 0xcc, 0xfe}; // Flag HDLC 0x7EAAAAAA, 33,cc,99,
            uint8_t syncWord[] = {0xcc, 0xcc};
            radioHal1->setSyncWord(syncWord, 2);
            radioHal1->setCRC(0);
            log_d("Set syncWord FSK AIS success!");
            if (state != RADIOLIB_ERR_NONE)
            {
                ret = false;
                log_d("Unable to set configuration, code %i", state);
            }
            state = radioHal1->setDataShaping(config.rf1_shaping);
            state = radioHal1->setEncoding(config.rf1_encoding);
            if (config.rf1_type == RF_SX1261 || config.rf1_type == RF_SX1262 || config.rf1_type == RF_SX1268 || config.rf1_type == RF_SX126x)
                radioHal1->fixedPacketLengthMode(RADIOLIB_SX126X_MAX_PACKET_LENGTH);
            else
                radioHal1->fixedPacketLengthMode(RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK);

            radioHal1->setDio0Action(setFlag1);
        }
        else
        {
            radioHal1->setDio0Action(setFlag1);
        }

        if (config.rf1_type == RF_SX1272 || config.rf1_type == RF_SX1273 || config.rf1_type == RF_SX1276 || config.rf1_type == RF_SX1278 || config.rf1_type == RF_SX1279)
        {
            radioHal1->setCurrentLimit(140);
        }
        else
        {
            radioHal1->setCurrentLimit(120);
        }
        
        radioHal1->setOutputPower(config.rf1_power);
        startRx1();
        radioHal1->setRxBoostedGainMode(config.rf1_rx_boost);
        
    }

    ax25_init(&AX25, aprs_msg_callback);
    return ret;
}
#endif

static inline int G3RUHScramble(uint32_t in, uint32_t *state)
{
    int out;
    out = ((*state >> 16) ^ (*state >> 11) ^ in) & 1;
    *state = (*state << 1) | (out & 1);
    return (out);
}

static inline int G3RUHDescramble(uint32_t in, uint32_t *state)
{
    int out;
    out = ((*state >> 16) ^ (*state >> 11) ^ in) & 1;
    *state = (*state << 1) | (in & 1);
    return (out);
}

void lfsr_scramble(byte *inputArray, byte *outputArray, int length)
{
    // Polynomial for 17-bit LFSR: x^17 + x^12 + 1
    // uint32_t lfsr = 0x1FFFF; // Initial LFSR state (all 1s)
    // uint32_t lfsr = 0x5F435A95;
    uint32_t lfsr = 0xFFFFFFFF;

    for (int i = 0; i < length; i++)
    {
        byte scrambledByte = 0;
        // if(i<101) log_d("LFSR[%i:%2x]: %04X",i,inputArray[i],lfsr);
        //  Process each bit in the current byte
        for (int bit = 0; bit < 8; bit++)
        {
            // Extract the current bit of the input byte
            int inputBit = (inputArray[i] >> bit) & 0x01;

            byte scrambledBit = (byte)G3RUHScramble(inputBit, &lfsr) & 0x1;
            // Add the scrambled bit to the scrambled byte
            scrambledByte |= (scrambledBit << bit);
        }
        // Store the scrambled byte in the output array
        outputArray[i] = scrambledByte;
    }
}

void lfsr_descramble(byte *inputArray, byte *outputArray, int length)
{
    // Polynomial for 17-bit LFSR: x^17 + x^12 + 1
    uint32_t lfsr = 0xFFFFFFFF;
    for (int i = 0; i < length; i++)
    {
        byte scrambledByte = 0;
        // Process each bit in the current byte
        for (int bit = 0; bit < 8; bit++)
        {
            // Extract the current bit of the input byte
            int inputBit = (inputArray[i] >> bit) & 0x01;

            byte scrambledBit = (byte)G3RUHDescramble(inputBit, &lfsr) & 0x1;
            // Add the scrambled bit to the scrambled byte
            scrambledByte |= (scrambledBit << bit);
        }
        // Store the scrambled byte in the output array
        outputArray[i] = scrambledByte;
    }
}

// Non-Return-to-Zero Inverted (NRZI) encoding:
// 0 causes a state transition and 1 does not.
void NRZIEncode(uint8_t *input_bytes, uint8_t *output_bytes, size_t length)
{
    bool state = true;
    for (size_t i = 0; i < length; i++)
    {
        uint8_t input_byte = input_bytes[i];
        output_bytes[i] = 0;

        for (int j = 0; j < 8; j++)
        {
            bool current_bit = (input_byte >> j) & 1;
            if (!current_bit)
                state = !state;

            if (state)
            {
                output_bytes[i] |= (1 << j);
            }
        }
    }
}

// Non-Return-to-Zero Inverted (NRZI) decoding.
void NRZIDecode(uint8_t *input_bytes, uint8_t *output_bytes, size_t length)
{
    bool previous_bit = false; // Initial state
    for (size_t i = 0; i < length; i++)
    {
        uint8_t input_byte = input_bytes[i];
        output_bytes[i] = 0;

        for (int j = 0; j < 8; j++)
        {
            bool current_bit = (input_byte >> j) & 1;
            if (current_bit == previous_bit)
            {
                output_bytes[i] |= (1 << j);
            }
            previous_bit = current_bit;
        }
    }
}

void flipBit(uint8_t *input_bytes, uint8_t *output_bytes, size_t len)
{
    uint8_t bit = 0;
    for (size_t i = 0; i < len; i++)
    {
        uint8_t input_byte = input_bytes[i];
        uint8_t output_byte = 0;
        for (int j = 7; j >= 0; j--)
        {
            bool current_bit = (input_byte >> j) & 1;
            if (current_bit)
            {
                output_byte |= (0x80 >> j);
            }
        }
        output_bytes[i] = output_byte;
    }
}

void printHex(uint8_t *data, size_t len)
{
    String str = "HEX: [" + String(len) + "]";
    for (int i = 0; i < len; i++)
    {
        str += "0x" + String((unsigned char)data[i], HEX) + ",";
    }
    log_d("%s", str.c_str());
    str.clear();
}

// portMUX_TYPE DRAM_ATTR fifoMux = portMUX_INITIALIZER_UNLOCKED;
void taskADDFifo(void *pvParameters)
{
    for (;;)
    {
        if (ax25_stateTx)
        {
            if (flagAddFifo && remLength > 0)
            {

                // portENTER_CRITICAL_ISR(&fifoMux); // ISR start
                // flagAddFifo=false;
                // portEXIT_CRITICAL_ISR(&fifoMux); // ISR end
                // log_d("TX:%u remLen:%i",millis(),remLength);
                radioHal->fifoAdd(txBufPtr, txBuf_len, &remLength);
                flagAddFifo = false;
                continue;
            }
            vTaskDelay(pdTICKS_TO_MS(1));
        }
        else
        {
            if (flagGetFifo)
            {
                // fifoLock = true;
                if (rxBuff == NULL)
                {
                    vTaskDelay(pdTICKS_TO_MS(10));
                    continue;
                }
                if (rxBuffLen == 0)
                    radioRecvStatus();
                if (radioHal->fifoGet(rxBuff, MAX_RFBUFF, &rxBuffLen))
                {
                    received = true;
                    // fifoLock = false;
                    // radioRecvStatus();
                }
                flagGetFifo = false;
                continue;
            }
            vTaskDelay(pdTICKS_TO_MS(1));
        }
    }
}

extern FIFOBuffer rxFifo;
Hdlc hdlcFlag;

void parseBit(uint8_t *raw, size_t len)
{
    uint8_t bit = 0;
    fifo_flush(&rxFifo);
    hdlcFlag.bitIndex = 0;
    hdlcFlag.currentByte = 0;
    hdlcFlag.demodulatedBits = 0;
    hdlcFlag.receiving = false;
    for (size_t i = 0; i < len; i++)
    {
        uint8_t input_byte = raw[i];
        uint8_t output_byte = 0;
        for (int j = 0; j < 8; j++)
        {
            bool current_bit = (input_byte >> j) & 1;
            if (hdlcParse(&hdlcFlag, current_bit, &rxFifo))
            {
                if (fifo_isfull(&rxFifo))
                {
                    fifo_flush(&rxFifo);
                    log_d("FIFO IS FULL");
                }
                ax25_poll(&AX25);
            }
        }
    }
}

void APRS_getHDLC(uint8_t *raw, size_t len)
{
    AX25.frame_len = len;
    memcpy(AX25.buf, raw, len);
    ax25_decode(&AX25);
}

bool APRS_poll(void)
{
    bool ret = false;
    if (millis() > rxTimeout)
    {
        rxTimeout = millis() + 600000;
        APRS_init(&config);
    }
    // check if the flag is set
    if (config.rf_en)
    {
        if (received)
        {
            rxTimeout = millis() + 900000;
            // disable the interrupt service routine while
            // processing the data
            disableInterrupt();

            // LED_Status(0, 200, 0);

            received = false;
            uint8_t *byteArr = (uint8_t *)calloc(MAX_RFBUFF, sizeof(uint8_t));
            if (byteArr)
            {
                int numBytes = 0;
                int state = -1;
                if (config.rf_mode == RF_MODE_G3RUH)
                {
                    rxTimeout = millis() + 30000;
                    uint8_t *outputBuff = (uint8_t *)calloc(MAX_RFBUFF, sizeof(uint8_t));
                    state = RADIOLIB_ERR_NULL_POINTER;
                    if (outputBuff)
                    {
                        if ((config.rf_dio1_gpio < 0) || config.rf_type == RF_SX1261 || config.rf_type == RF_SX1262 || config.rf_type == RF_SX1268 || config.rf_type == RF_SX126x)
                        {
                            radioRecvStatus();
                            numBytes = radioHal->getPacketLength();
                            memset(byteArr, 0, MAX_RFBUFF);
                            state = radioHal->readData(&rxBuff[0], numBytes);
                            // printHex(rxBuff, numBytes);
                        }
                        else
                        {
                            numBytes = rxBuffLen;
                        }
                        if (numBytes > MAX_RFBUFF)
                            numBytes = MAX_RFBUFF;
                        uint8_t *buff = (uint8_t *)calloc(numBytes, sizeof(uint8_t));
                        if (buff)
                        {
                            flipBit(rxBuff, buff, numBytes);
                            LED_Status(0, 200, 0);
                            startRx();

                            // printHex(rxBuff, rxBuffLen);
                            // int s=0;
                            int scn = numBytes;
                            if (scn > 10)
                                scn = 10;
                            for (int s = 0; s < scn; s++)
                            {
                                memset(outputBuff, 0, MAX_RFBUFF);
                                lfsr_descramble(&buff[s], outputBuff, numBytes - s);
                                memset(byteArr, 0, MAX_RFBUFF);
                                NRZIDecode(outputBuff, byteArr, numBytes - s);
                                int num = numBytes - s;
                                size_t frame_len = 0;
                                int idx = hdlcDecodeAX25(outputBuff, frame_len, &byteArr[0], num);
                                if (frame_len > 10)
                                {
                                    // LED_Status(0, 200, 0);
                                    // printHex(buff, numBytes);
                                    // printHex(byteArr, num);
                                    rssi = fskRSSI;
                                    fskRSSI = -130;
                                    log_d("[GFSK] RSSI:%.0f dBm", rssi);
                                    ax25_init(&AX25, aprs_msg_callback);
                                    AX25.frame_len = frame_len;
                                    memcpy(AX25.buf, outputBuff, frame_len);
                                    ax25_decode(&AX25);
                                    log_d("[GFSK] Received packet! %d Byte IDX:%d", frame_len, s);
                                    received = false;
                                    break;
                                }
                            }
                            free(buff);
                        }
                        free(outputBuff);
                    }
                }
                else if (config.rf_mode == RF_MODE_AIS)
                {
                    rxTimeout = millis() + 300000;
                    numBytes = radioHal->getPacketLength();
                    if (numBytes > 0)
                    {
                        uint8_t *rawBuff = (uint8_t *)calloc(numBytes, sizeof(uint8_t));
                        if (rawBuff)
                        {
                            radioRecvStatus();
                            memset(rawBuff, 0, numBytes);
                            state = radioHal->readData(rawBuff, numBytes);
                            if (state == RADIOLIB_ERR_NONE)
                            {
                                LED_Status(0, 200, 0);
                                flipBit(rawBuff, byteArr, numBytes);
                                NRZIDecode(byteArr, rawBuff, numBytes);
                                // printHex(rawBuff, numBytes);
                                size_t frame_len = 0;
                                int idx = 0;
                                do
                                {
                                    frame_len = 0;
                                    idx = hdlcDecode(byteArr, frame_len, &rawBuff[idx], numBytes);
                                    if (frame_len > 2)
                                    {
                                        status.rxCount++;
                                        // rssi = fskRSSI;
                                        // fskRSSI = -130;
                                        frame_len -= 2; // remove FCS 2byte
                                        char nmea[256];
                                        ais_to_nmea((unsigned char *)byteArr, frame_len, nmea, sizeof(nmea));
                                        log_d("AIS NMEA: %s", nmea);
                                        String aisRaw = ais2aprs(nmea);
                                        if (aisRaw.length() > 0)
                                        {
                                            // log_d("%s",aisRaw.c_str());
                                            int size = APRS_getTNC2(aisRaw);
                                        }
                                    }
                                    numBytes -= idx;
                                } while (numBytes >= 10);
                            }
                            free(rawBuff);
                        }
                    }
                    startRx();
                }
                else if (config.rf_mode == RF_MODE_GFSK)
                {
                    uint8_t *outputBuff = (uint8_t *)calloc(MAX_RFBUFF, sizeof(uint8_t));
                    if (outputBuff)
                    {
                        if ((config.rf_dio1_gpio < 0) || config.rf_type == RF_SX1261 || config.rf_type == RF_SX1262 || config.rf_type == RF_SX1268 || config.rf_type == RF_SX126x)
                        {
                            radioRecvStatus();
                            numBytes = radioHal->getPacketLength();
                            memset(byteArr, 0, MAX_RFBUFF);
                            state = radioHal->readData(&rxBuff[0], numBytes);
                            // printHex(rxBuff, numBytes);
                        }
                        else
                        {
                            numBytes = rxBuffLen;
                        }
                        if (numBytes > MAX_RFBUFF)
                            numBytes = MAX_RFBUFF;
                        flipBit(rxBuff, byteArr, numBytes);
                        LED_Status(0, 200, 0);
                        startRx();
                        if (config.rf_ax25)
                        {
                            // printHex(rxBuff, rxBuffLen);
                            size_t frame_len = 0;
                            int idx = hdlcDecodeAX25(outputBuff, frame_len, &byteArr[0], numBytes);
                            if (frame_len > 10)
                            {
                                // LED_Status(0, 200, 0);
                                // printHex(buff, numBytes);
                                // printHex(byteArr, num);
                                rssi = fskRSSI;
                                fskRSSI = -130;
                                log_d("[GFSK] RSSI:%.0f dBm", rssi);
                                ax25_init(&AX25, aprs_msg_callback);
                                AX25.frame_len = frame_len;
                                memcpy(AX25.buf, outputBuff, frame_len);
                                ax25_decode(&AX25);
                                log_d("[GFSK AX.25] Received packet! %d Byte", frame_len);
                            }
                        }
                        else
                        {
                            String str = "";
                            for (int i = 0; i < numBytes; i++)
                            {
                                str += String((char)byteArr[i]);
                                // str.setCharAt(i,byteArr[i+3]);
                            }
                            int size = APRS_getTNC2(str);
                            log_d("[GFSK] Received packet! %d Byte", size);
                        }
                        free(outputBuff);
                    }
                }
                else if (config.rf_mode == RF_MODE_LoRa)
                {
                    LED_Status(0, 200, 0);
                    numBytes = radioHal->getPacketLength();
                    state = radioHal->readData(&byteArr[0], numBytes);
                    radioRecvStatus();
                    startRx();
                    if (state == RADIOLIB_ERR_NONE)
                    {
                        log_d("[LoRa] Received packet! %d Byte\n", numBytes);
                        // packet was successfully received
                        // log_d("[LoRa] Received packet! %d Byte\n", numBytes);
                        if (numBytes > 10)
                        {
                            // Check AX.25 protocol with HDLC 7E Flage
                            if (byteArr[0] == 0x7E)
                            {
                                // parseBit(byteArr, numBytes);
                                uint8_t *outputBuff = (uint8_t *)calloc(numBytes, sizeof(uint8_t));
                                if (outputBuff)
                                {
                                    size_t frame_len = 0;
                                    int idx = hdlcDecodeAX25(outputBuff, frame_len, &byteArr[0], numBytes);
                                    if (frame_len > 10)
                                    {
                                        ax25_init(&AX25, aprs_msg_callback);
                                        AX25.frame_len = frame_len;
                                        memcpy(AX25.buf, outputBuff, frame_len);
                                        ax25_decode(&AX25);
                                        log_d("[LoRa AX.25] Packet size %d Byte", frame_len);
                                    }
                                    free(outputBuff);
                                }
                            }
                            else if (byteArr[0] == '<' && byteArr[1] == 0xFF && byteArr[2] == 0x01)
                            {
                                // Get TNC2 Raw text
                                String str = "";
                                for (int i = 0; i < numBytes - 3; i++)
                                {
                                    str += String((char)byteArr[i + 3]);
                                    // str.setCharAt(i,byteArr[i+3]);
                                }
                                int size = APRS_getTNC2(str);
                                log_d("[LoRa TNC2] Packet size %d Byte", size);
                            }
                        }
                        ret = true;
                    }
                    else if (state == RADIOLIB_ERR_CRC_MISMATCH)
                    {
                        // packet was received, but is malformed
                        log_d("[LoRa] CRC error!");
                    }
                    else
                    {
                        // some other error occurred
                        log_d("[LoRa] Failed, code %d", state);
                        // Serial.println(state);
                    }
                }
                free(byteArr);
            }
            LED_Status(0, 0, 0);
        }
        else
        {
            if (ax25_stateTx)
            {
                disableInterrupt();
                LED_Status(200, 0, 0);
                // flagTx = true;
                // if (config.rf_type == RF_SX1272 || config.rf_type == RF_SX1273 || config.rf_type == RF_SX1276 || config.rf_type == RF_SX1278 || config.rf_type == RF_SX1279)
                // {
                //     radioHal->setCurrentLimit(240);
                // }
                // else
                // {
                //     radioHal->setCurrentLimit(140);
                // }

                int byteArrLen = 300;
                uint8_t *byteArr = (uint8_t *)calloc(byteArrLen, sizeof(uint8_t));
                memset(byteArr, 0, byteArrLen);
                if (byteArr)
                {
                    // Serial.print("TX HEX: ");
                    int i;
                    memset(byteArr, 0, byteArrLen);
                    for (i = 0; i < byteArrLen; i++)
                    {
                        if (config.rf_mode == RF_MODE_G3RUH)
                        {
                            if (i < 7)
                            {
                                byteArr[i] = 0x7E;
                                continue;
                            }
                        }

                        int c = tx_getchar();
                        if (c == -1)
                        {
                            break;
                        }
                        else
                        {
                            byteArr[i] = (uint8_t)c;
                        }

                        // Serial.printf("%0X ",byteArr[i]);
                    }

                    if (config.rf_mode == RF_MODE_G3RUH)
                    {
                        if (i > byteArrLen)
                            i = byteArrLen;
                        int sizeBuff = i;
                        uint8_t *rawBuff = (uint8_t *)calloc(sizeBuff, sizeof(uint8_t));
                        if (rawBuff)
                        {

                            memset(rawBuff, 0, sizeBuff);
                            NRZIEncode(byteArr, rawBuff, i);
                            memset(byteArr, 0, byteArrLen);
                            lfsr_scramble(rawBuff, byteArr, i);
                            free(rawBuff);

                            txBufPtr = (uint8_t *)calloc(i, sizeof(uint8_t));
                            if (txBufPtr)
                            {
                                memset(txBufPtr, 0, i);
                                flipBit(byteArr, txBufPtr, i); // Flip bit send LSB->MSB
                                radioHal->setCRC(0);
                                radioHal->fixedPacketLengthMode(i);
                                if (config.rf_type == RF_SX1272 || config.rf_type == RF_SX1273 || config.rf_type == RF_SX1276 || config.rf_type == RF_SX1278 || config.rf_type == RF_SX1279)
                                {
                                    if (i > RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK)
                                    {
                                        flagAddFifo = false;
                                        txBuf_len = i;
                                        remLength = i;
                                        fifoTxLock = false;
                                        radioHal->setFifoFullAction(fifoAdd);
                                        // radioHal->setFifoEmptyAction(fifoAdd);
                                        delay(10);
                                    }
                                }
                                // printHex(txBuff, i);
                                //  Send 7E flag after nrzi->scramble->flipLSB 0xc2,0x80,0x20,0x0,0x52,0xd6,0xdf,0xd5
                                // uint8_t syncWord[] = {0xab, 0xfb, 0x6b, 0x4a, 0x0, 0x4, 0x1, 0x43};
                                // uint8_t syncWordTX[] = {0x1, 0xe, 0x91, 0x6f, 0x5f, 0x43, 0x5a, 0x95};
                                uint8_t syncWordTX[] = {0xf5, 0xd4, 0xd9, 0x59, 0x7, 0xc2, 0x1, 0x3f};
                                radioHal->setSyncWord(syncWordTX, 8);
                                radioHal->transmit(txBufPtr, i);
                                delay(100);
                                free(txBufPtr);

                                syncWordTX[0] = 0xd9;
                                radioHal->setSyncWord(syncWordTX, 1);
                                if (config.rf_type == RF_SX1272 || config.rf_type == RF_SX1273 || config.rf_type == RF_SX1276 || config.rf_type == RF_SX1278 || config.rf_type == RF_SX1279)
                                {
                                    if (config.rf_dio1_gpio > -1)
                                    { // Use DIO1 for fifo interrupt
                                        radioHal->setFifoFullAction(fifoGet);
                                        radioHal->fixedPacketLengthMode(0);
                                    }
                                    else
                                    {
                                        radioHal->fixedPacketLengthMode(RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK);
                                        radioHal->setDio0Action(setFlag);
                                    }
                                }
                                else if (config.rf_type == RF_SX1261 || config.rf_type == RF_SX1262 || config.rf_type == RF_SX1268 || config.rf_type == RF_SX126x)
                                {
                                    radioHal->fixedPacketLengthMode(RADIOLIB_SX126X_MAX_PACKET_LENGTH);
                                    radioHal->setDio0Action(setFlag);
                                }
                                // log_d("FiFo Timer = %i,%i,%i",fifoTimer[1]-fifoTimer[0],fifoTimer[2]-fifoTimer[1],fifoTimer[3]-fifoTimer[2]);
                            }
                        }
                    }
                    else if (config.rf_mode == RF_MODE_GFSK)
                    {
                        txBufPtr = (uint8_t *)calloc(i, sizeof(uint8_t));
                        if (txBufPtr)
                        {
                            memset(txBufPtr, 0, i);
                            flipBit(byteArr, txBufPtr, i); // Flip bit send LSB->MSB
                            radioHal->fixedPacketLengthMode(i);
                            radioHal->setCRC(1);
                            if (config.rf_type == RF_SX1272 || config.rf_type == RF_SX1273 || config.rf_type == RF_SX1276 || config.rf_type == RF_SX1278 || config.rf_type == RF_SX1279)
                            {
                                if (i > RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK)
                                {
                                    flagAddFifo = false;
                                    txBuf_len = i;
                                    remLength = i;
                                    fifoTxLock = false;
                                    radioHal->setFifoFullAction(fifoAdd);
                                    delay(10);
                                }
                            }
                            uint8_t syncWordTX[] = {0x84, 0xB5, 0x12, 0xAD};
                            radioHal->setSyncWord(syncWordTX, 4);
                            radioHal->transmit(txBufPtr, i);
                            delay(100);
                            free(txBufPtr);

                            if (config.rf_type == RF_SX1272 || config.rf_type == RF_SX1273 || config.rf_type == RF_SX1276 || config.rf_type == RF_SX1278 || config.rf_type == RF_SX1279)
                            {
                                if (config.rf_dio1_gpio > -1)
                                { // Use DIO1 for fifo interrupt
                                    radioHal->setFifoFullAction(fifoGet);
                                    radioHal->fixedPacketLengthMode(0);
                                }
                                else
                                {
                                    radioHal->fixedPacketLengthMode(RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK);
                                    radioHal->setDio0Action(setFlag);
                                }
                            }
                            else if (config.rf_type == RF_SX1261 || config.rf_type == RF_SX1262 || config.rf_type == RF_SX1268 || config.rf_type == RF_SX126x)
                            {
                                radioHal->fixedPacketLengthMode(RADIOLIB_SX126X_MAX_PACKET_LENGTH);
                                radioHal->setDio0Action(setFlag);
                            }
                        }
                    }
                    else
                    {
                        radioHal->transmit(byteArr, i);
                    }
                    free(byteArr);
                    ret = true;
                }
                LED_Status(0, 0, 0);
                ax25_stateTx = false;
                startRx();
            }
            else
            {
                // ax25_poll(&AX25);
                //  size_t pkgLen = radioHal->getPacketLength();
                //  if(pkgLen>30) fifoGet();
                //  if(pkgLen>0) log_d("PKG Len %i Byte",pkgLen);
            }
        }
    }

#ifdef RF2
    if (config.rf1_en)
    {
        if (received1)
        {
            rxTimeout1 = millis() + 900000;
            // disable the interrupt service routine while
            // processing the data
            disableInterrupt1();

            log_d("RF2 Received");

            // LED_Status(0, 200, 0);

            received1 = false;
            uint8_t *byteArr = (uint8_t *)calloc(MAX_RFBUFF, sizeof(uint8_t));
            if (byteArr)
            {
                int numBytes = 0;
                int state = -1;
                if (config.rf1_mode == RF_MODE_G3RUH)
                {
                    rxTimeout1 = millis() + 30000;
                    uint8_t *outputBuff = (uint8_t *)calloc(MAX_RFBUFF, sizeof(uint8_t));
                    state = RADIOLIB_ERR_NULL_POINTER;
                    if (outputBuff)
                    {
                        if ((config.rf1_dio1_gpio < 0) || config.rf1_type == RF_SX1261 || config.rf_type == RF_SX1262 || config.rf1_type == RF_SX1268 || config.rf1_type == RF_SX126x)
                        {
                            radioRecvStatus1();
                            numBytes = radioHal->getPacketLength();
                            memset(byteArr, 0, MAX_RFBUFF);
                            state = radioHal1->readData(&rx1Buff[0], numBytes);
                            // printHex(rx1Buff, numBytes);
                        }
                        else
                        {
                            numBytes = rx1BuffLen;
                        }
                        if (numBytes > MAX_RFBUFF)
                            numBytes = MAX_RFBUFF;
                        uint8_t *buff = (uint8_t *)calloc(numBytes, sizeof(uint8_t));
                        if (buff)
                        {
                            flipBit(rx1Buff, buff, numBytes);
                            LED_Status(0, 200, 0);
                            startRx1();

                            // printHex(rx1Buff, rx1BuffLen);
                            // int s=0;
                            int scn = numBytes;
                            if (scn > 10)
                                scn = 10;
                            for (int s = 0; s < scn; s++)
                            {
                                memset(outputBuff, 0, MAX_RFBUFF);
                                lfsr_descramble(&buff[s], outputBuff, numBytes - s);
                                memset(byteArr, 0, MAX_RFBUFF);
                                NRZIDecode(outputBuff, byteArr, numBytes - s);
                                int num = numBytes - s;
                                size_t frame_len = 0;
                                int idx = hdlcDecodeAX25(outputBuff, frame_len, &byteArr[0], num);
                                if (frame_len > 10)
                                {
                                    // LED_Status(0, 200, 0);
                                    // printHex(buff, numBytes);
                                    // printHex(byteArr, num);
                                    rssi = fskRSSI;
                                    fskRSSI = -130;
                                    log_d("[GFSK] RSSI:%.0f dBm", rssi);
                                    ax25_init(&AX25, aprs_msg_callback);
                                    AX25.frame_len = frame_len;
                                    memcpy(AX25.buf, outputBuff, frame_len);
                                    ax25_decode(&AX25);
                                    log_d("[GFSK] Received packet! %d Byte IDX:%d", frame_len, s);
                                    received = false;
                                    break;
                                }
                            }
                            free(buff);
                        }
                        free(outputBuff);
                    }
                }
                else if (config.rf1_mode == RF_MODE_AIS)
                {
                    rxTimeout1 = millis() + 300000;
                    numBytes = radioHal1->getPacketLength();
                    if (numBytes > 0)
                    {
                        uint8_t *rawBuff = (uint8_t *)calloc(numBytes, sizeof(uint8_t));
                        if (rawBuff)
                        {
                            radioRecvStatus1();
                            memset(rawBuff, 0, numBytes);
                            state = radioHal1->readData(rawBuff, numBytes);
                            if (state == RADIOLIB_ERR_NONE)
                            {
                                LED_Status(0, 200, 0);
                                flipBit(rawBuff, byteArr, numBytes);
                                NRZIDecode(byteArr, rawBuff, numBytes);
                                // printHex(rawBuff, numBytes);
                                size_t frame_len = 0;
                                int idx = 0;
                                do
                                {
                                    frame_len = 0;
                                    idx = hdlcDecode(byteArr, frame_len, &rawBuff[idx], numBytes);
                                    if (frame_len > 2)
                                    {
                                        status.rxCount++;
                                        // rssi = fskRSSI;
                                        // fskRSSI = -130;
                                        frame_len -= 2; // remove FCS 2byte
                                        char nmea[256];
                                        ais_to_nmea((unsigned char *)byteArr, frame_len, nmea, sizeof(nmea));
                                        log_d("AIS NMEA: %s", nmea);
                                        String aisRaw = ais2aprs(nmea);
                                        if (aisRaw.length() > 0)
                                        {
                                            // log_d("%s",aisRaw.c_str());
                                            int size = APRS_getTNC2(aisRaw);
                                        }
                                    }
                                    numBytes -= idx;
                                } while (numBytes >= 10);
                            }
                            free(rawBuff);
                        }
                    }
                    startRx1();
                }
                else if (config.rf1_mode == RF_MODE_GFSK)
                {
                    uint8_t *outputBuff = (uint8_t *)calloc(MAX_RFBUFF, sizeof(uint8_t));
                    if (outputBuff)
                    {
                        if ((config.rf1_dio1_gpio < 0) || config.rf1_type == RF_SX1261 || config.rf1_type == RF_SX1262 || config.rf1_type == RF_SX1268 || config.rf1_type == RF_SX126x)
                        {
                            radioRecvStatus1();
                            numBytes = radioHal1->getPacketLength();
                            memset(byteArr, 0, MAX_RFBUFF);
                            state = radioHal1->readData(&rx1Buff[0], numBytes);
                            // printHex(rx1Buff, numBytes);
                        }
                        else
                        {
                            numBytes = rx1BuffLen;
                        }
                        if (numBytes > MAX_RFBUFF)
                            numBytes = MAX_RFBUFF;
                        flipBit(rx1Buff, byteArr, numBytes);
                        LED_Status(0, 200, 0);
                        startRx1();
                        if (config.rf1_ax25)
                        {
                            // printHex(rx1Buff, rxBuffLen);
                            size_t frame_len = 0;
                            int idx = hdlcDecodeAX25(outputBuff, frame_len, &byteArr[0], numBytes);
                            if (frame_len > 10)
                            {
                                // LED_Status(0, 200, 0);
                                // printHex(buff, numBytes);
                                // printHex(byteArr, num);
                                rssi = fskRSSI;
                                fskRSSI = -130;
                                log_d("[GFSK] RSSI:%.0f dBm", rssi);
                                ax25_init(&AX25, aprs_msg_callback);
                                AX25.frame_len = frame_len;
                                memcpy(AX25.buf, outputBuff, frame_len);
                                ax25_decode(&AX25);
                                log_d("[GFSK AX.25] Received packet! %d Byte", frame_len);
                            }
                        }
                        else
                        {
                            String str = "";
                            for (int i = 0; i < numBytes; i++)
                            {
                                str += String((char)byteArr[i]);
                                // str.setCharAt(i,byteArr[i+3]);
                            }
                            int size = APRS_getTNC2(str);
                            log_d("[GFSK] Received packet! %d Byte", size);
                        }
                        free(outputBuff);
                    }
                }
                else if (config.rf1_mode == RF_MODE_LoRa)
                {
                    LED_Status(0, 200, 0);
                    numBytes = radioHal1->getPacketLength();
                    state = radioHal1->readData(&byteArr[0], numBytes);
                    radioRecvStatus1();
                    startRx1();
                    if (state == RADIOLIB_ERR_NONE)
                    {
                        log_d("[LoRa1] Received packet! %d Byte\n", numBytes);
                        // packet was successfully received
                        // log_d("[LoRa] Received packet! %d Byte\n", numBytes);
                        if (numBytes > 10)
                        {
                            // Check AX.25 protocol with HDLC 7E Flage
                            if (byteArr[0] == 0x7E)
                            {
                                // parseBit(byteArr, numBytes);
                                uint8_t *outputBuff = (uint8_t *)calloc(numBytes, sizeof(uint8_t));
                                if (outputBuff)
                                {
                                    size_t frame_len = 0;
                                    int idx = hdlcDecodeAX25(outputBuff, frame_len, &byteArr[0], numBytes);
                                    if (frame_len > 10)
                                    {
                                        ax25_init(&AX25, aprs_msg_callback);
                                        AX25.frame_len = frame_len;
                                        memcpy(AX25.buf, outputBuff, frame_len);
                                        ax25_decode(&AX25);
                                        log_d("[LoRa1 AX.25] Packet size %d Byte", frame_len);
                                    }
                                    free(outputBuff);
                                }
                            }
                            else if (byteArr[0] == '<' && byteArr[1] == 0xFF && byteArr[2] == 0x01)
                            {
                                // Get TNC2 Raw text
                                String str = "";
                                for (int i = 0; i < numBytes - 3; i++)
                                {
                                    str += String((char)byteArr[i + 3]);
                                    // str.setCharAt(i,byteArr[i+3]);
                                }
                                int size = APRS_getTNC2(str);
                                log_d("[LoRa TNC2] Packet size %d Byte", size);
                            }
                        }
                        ret = true;
                    }
                    else if (state == RADIOLIB_ERR_CRC_MISMATCH)
                    {
                        // packet was received, but is malformed
                        log_d("[LoRa1] CRC error!");
                    }
                    else
                    {
                        // some other error occurred
                        log_d("[LoRa1] Failed, code %d", state);
                        // Serial.println(state);
                    }
                }
                free(byteArr);
            }
            LED_Status(0, 0, 0);
        }
    }
// else
// {
//     if (ax25_stateTx)
//     {
//         disableInterrupt1();
//         LED_Status(200, 0, 0);

//         int byteArrLen = 300;
//         uint8_t *byteArr = (uint8_t *)calloc(byteArrLen, sizeof(uint8_t));
//         memset(byteArr, 0, byteArrLen);
//         if (byteArr)
//         {
//             // Serial.print("TX HEX: ");
//             int i;
//             memset(byteArr, 0, byteArrLen);
//             for (i = 0; i < byteArrLen; i++)
//             {
//                 if (config.rf1_mode == RF_MODE_G3RUH)
//                 {
//                     if (i < 7)
//                     {
//                         byteArr[i] = 0x7E;
//                         continue;
//                     }
//                 }

//                 int c = tx_getchar();
//                 if (c == -1)
//                 {
//                     break;
//                 }
//                 else
//                 {
//                     byteArr[i] = (uint8_t)c;
//                 }

//                 // Serial.printf("%0X ",byteArr[i]);
//             }

//             if (config.rf1_mode == RF_MODE_G3RUH)
//             {
//                 if (i > byteArrLen)
//                     i = byteArrLen;
//                 int sizeBuff = i;
//                 uint8_t *rawBuff = (uint8_t *)calloc(sizeBuff, sizeof(uint8_t));
//                 if (rawBuff)
//                 {

//                     memset(rawBuff, 0, sizeBuff);
//                     NRZIEncode(byteArr, rawBuff, i);
//                     memset(byteArr, 0, byteArrLen);
//                     lfsr_scramble(rawBuff, byteArr, i);
//                     free(rawBuff);

//                     txBufPtr = (uint8_t *)calloc(i, sizeof(uint8_t));
//                     if (txBufPtr)
//                     {
//                         memset(txBufPtr, 0, i);
//                         flipBit(byteArr, txBufPtr, i); // Flip bit send LSB->MSB
//                         radioHal1->setCRC(0);
//                         radioHal1->fixedPacketLengthMode(i);
//                         if (config.rf1_type == RF_SX1272 || config.rf1_type == RF_SX1273 || config.rf1_type == RF_SX1276 || config.rf1_type == RF_SX1278 || config.rf_type == RF_SX1279)
//                         {
//                             if (i > RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK)
//                             {
//                                 flagAddFifo = false;
//                                 txBuf_len = i;
//                                 remLength = i;
//                                 fifoTxLock = false;
//                                 radioHal1->setFifoFullAction(fifoAdd);
//                                 // radioHal->setFifoEmptyAction(fifoAdd);
//                                 delay(10);
//                             }
//                         }
//                         // printHex(txBuff, i);
//                         uint8_t syncWordTX[] = {0xf5, 0xd4, 0xd9, 0x59, 0x7, 0xc2, 0x1, 0x3f};
//                         radioHal1->setSyncWord(syncWordTX, 8);
//                         radioHal1->transmit(txBufPtr, i);
//                         delay(100);
//                         free(txBufPtr);

//                         syncWordTX[0] = 0xd9;
//                         radioHal1->setSyncWord(syncWordTX, 1);
//                         if (config.rf1_type == RF_SX1272 || config.rf1_type == RF_SX1273 || config.rf1_type == RF_SX1276 || config.rf1_type == RF_SX1278 || config.rf1_type == RF_SX1279)
//                         {
//                             if (config.rf1_dio1_gpio > -1)
//                             { // Use DIO1 for fifo interrupt
//                                 radioHal1->setFifoFullAction(fifoGet);
//                                 radioHal1->fixedPacketLengthMode(0);
//                             }
//                             else
//                             {
//                                 radioHal1->fixedPacketLengthMode(RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK);
//                                 radioHal1->setDio0Action(setFlag);
//                             }
//                         }
//                         else if (config.rf_type == RF_SX1261 || config.rf1_type == RF_SX1262 || config.rf1_type == RF_SX1268 || config.rf1_type == RF_SX126x)
//                         {
//                             radioHal1->fixedPacketLengthMode(RADIOLIB_SX126X_MAX_PACKET_LENGTH);
//                             radioHal1->setDio0Action(setFlag);
//                         }
//                         // log_d("FiFo Timer = %i,%i,%i",fifoTimer[1]-fifoTimer[0],fifoTimer[2]-fifoTimer[1],fifoTimer[3]-fifoTimer[2]);
//                     }
//                 }
//             }
//             else if (config.rf1_mode == RF_MODE_GFSK)
//             {
//                 txBufPtr = (uint8_t *)calloc(i, sizeof(uint8_t));
//                 if (txBufPtr)
//                 {
//                     memset(txBufPtr, 0, i);
//                     flipBit(byteArr, txBufPtr, i); // Flip bit send LSB->MSB
//                     radioHal1->fixedPacketLengthMode(i);
//                     radioHal1->setCRC(1);
//                     if (config.rf1_type == RF_SX1272 || config.rf1_type == RF_SX1273 || config.rf1_type == RF_SX1276 || config.rf1_type == RF_SX1278 || config.rf1_type == RF_SX1279)
//                     {
//                         if (i > RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK)
//                         {
//                             flagAddFifo = false;
//                             txBuf_len = i;
//                             remLength = i;
//                             fifoTxLock = false;
//                             radioHal1->setFifoFullAction(fifoAdd);
//                             delay(10);
//                         }
//                     }
//                     uint8_t syncWordTX[] = {0x84, 0xB5, 0x12, 0xAD};
//                     radioHal1->setSyncWord(syncWordTX, 4);
//                     radioHal1->transmit(txBufPtr, i);
//                     delay(100);
//                     free(txBufPtr);

//                     if (config.rf1_type == RF_SX1272 || config.rf1_type == RF_SX1273 || config.rf1_type == RF_SX1276 || config.rf1_type == RF_SX1278 || config.rf1_type == RF_SX1279)
//                     {
//                         if (config.rf1_dio1_gpio > -1)
//                         { // Use DIO1 for fifo interrupt
//                             radioHal1->setFifoFullAction(fifoGet);
//                             radioHal1->fixedPacketLengthMode(0);
//                         }
//                         else
//                         {
//                             radioHal1->fixedPacketLengthMode(RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK);
//                             radioHal1->setDio0Action(setFlag);
//                         }
//                     }
//                     else if (config.rf1_type == RF_SX1261 || config.rf1_type == RF_SX1262 || config.rf1_type == RF_SX1268 || config.rf1_type == RF_SX126x)
//                     {
//                         radioHal1->fixedPacketLengthMode(RADIOLIB_SX126X_MAX_PACKET_LENGTH);
//                         radioHal1->setDio0Action(setFlag);
//                     }
//                 }
//             }
//             else
//             {
//                 radioHal1->transmit(byteArr, i);
//             }
//             free(byteArr);
//             ret = true;
//         }
//         LED_Status(0, 0, 0);
//         ax25_stateTx = false;
//         startRx1();
//     }
// }
#endif
    return ret;
}

// void APRS_setCallsign(char *call, int ssid)
// {
//     memset(CALL, 0, 7);
//     int i = 0;
//     while (i < 6 && call[i] != 0)
//     {
//         CALL[i] = call[i];
//         i++;
//     }
//     CALL_SSID = ssid;
// }

// void APRS_setDestination(char *call, int ssid)
// {
//     memset(DST, 0, 7);
//     int i = 0;
//     while (i < 6 && call[i] != 0)
//     {
//         DST[i] = call[i];
//         i++;
//     }
//     DST_SSID = ssid;
// }

// void APRS_setPath1(char *call, int ssid)
// {
//     memset(PATH1, 0, 7);
//     int i = 0;
//     while (i < 6 && call[i] != 0)
//     {
//         PATH1[i] = call[i];
//         i++;
//     }
//     PATH1_SSID = ssid;
// }

// void APRS_setPath2(char *call, int ssid)
// {
//     memset(PATH2, 0, 7);
//     int i = 0;
//     while (i < 6 && call[i] != 0)
//     {
//         PATH2[i] = call[i];
//         i++;
//     }
//     PATH2_SSID = ssid;
// }

// void APRS_setMessageDestination(char *call, int ssid)
// {
//     memset(message_recip, 0, 7);
//     int i = 0;
//     while (i < 6 && call[i] != 0)
//     {
//         message_recip[i] = call[i];
//         i++;
//     }
//     message_recip_ssid = ssid;
// }

// void APRS_setPreamble(unsigned long pre)
// {
//     custom_preamble = pre;
// }

// void APRS_setTail(unsigned long tail)
// {
//     custom_tail = tail;
// }

// void APRS_useAlternateSymbolTable(bool use)
// {
//     if (use)
//     {
//         symbolTable = '\\';
//     }
//     else
//     {
//         symbolTable = '/';
//     }
// }

// void APRS_setSymbol(char sym)
// {
//     symbol = sym;
// }

// void APRS_setLat(char *lat)
// {
//     memset(latitude, 0, 9);
//     int i = 0;
//     while (i < 8 && lat[i] != 0)
//     {
//         latitude[i] = lat[i];
//         i++;
//     }
// }

// void APRS_setLon(char *lon)
// {
//     memset(longtitude, 0, 10);
//     int i = 0;
//     while (i < 9 && lon[i] != 0)
//     {
//         longtitude[i] = lon[i];
//         i++;
//     }
// }

// void APRS_setPower(int s)
// {
//     if (s >= 0 && s < 10)
//     {
//         power = s;
//     }
// }

// void APRS_setHeight(int s)
// {
//     if (s >= 0 && s < 10)
//     {
//         height = s;
//     }
// }

// void APRS_setGain(int s)
// {
//     if (s >= 0 && s < 10)
//     {
//         gain = s;
//     }
// }

// void APRS_setDirectivity(int s)
// {
//     if (s >= 0 && s < 10)
//     {
//         directivity = s;
//     }
// }

// void APRS_printSettings()
// {
//     Serial.println(F("LibAPRS Settings:"));
//     Serial.print(F("Callsign:     "));
//     Serial.print(CALL);
//     Serial.print(F("-"));
//     Serial.println(CALL_SSID);
//     Serial.print(F("Destination:  "));
//     Serial.print(DST);
//     Serial.print(F("-"));
//     Serial.println(DST_SSID);
//     Serial.print(F("Path1:        "));
//     Serial.print(PATH1);
//     Serial.print(F("-"));
//     Serial.println(PATH1_SSID);
//     Serial.print(F("Path2:        "));
//     Serial.print(PATH2);
//     Serial.print(F("-"));
//     Serial.println(PATH2_SSID);
//     Serial.print(F("Message dst:  "));
//     if (message_recip[0] == 0)
//     {
//         Serial.println(F("N/A"));
//     }
//     else
//     {
//         Serial.print(message_recip);
//         Serial.print(F("-"));
//         Serial.println(message_recip_ssid);
//     }
//     Serial.print(F("TX Preamble:  "));
//     Serial.println(custom_preamble);
//     Serial.print(F("TX Tail:      "));
//     Serial.println(custom_tail);
//     Serial.print(F("Symbol table: "));
//     if (symbolTable == '/')
//     {
//         Serial.println(F("Normal"));
//     }
//     else
//     {
//         Serial.println(F("Alternate"));
//     }
//     Serial.print(F("Symbol:       "));
//     Serial.println(symbol);
//     Serial.print(F("Power:        "));
//     if (power < 10)
//     {
//         Serial.println(power);
//     }
//     else
//     {
//         Serial.println(F("N/A"));
//     }
//     Serial.print(F("Height:       "));
//     if (height < 10)
//     {
//         Serial.println(height);
//     }
//     else
//     {
//         Serial.println(F("N/A"));
//     }
//     Serial.print(F("Gain:         "));
//     if (gain < 10)
//     {
//         Serial.println(gain);
//     }
//     else
//     {
//         Serial.println(F("N/A"));
//     }
//     Serial.print(F("Directivity:  "));
//     if (directivity < 10)
//     {
//         Serial.println(directivity);
//     }
//     else
//     {
//         Serial.println(F("N/A"));
//     }
//     Serial.print(F("Latitude:     "));
//     if (latitude[0] != 0)
//     {
//         Serial.println(latitude);
//     }
//     else
//     {
//         Serial.println(F("N/A"));
//     }
//     Serial.print(F("Longtitude:   "));
//     if (longtitude[0] != 0)
//     {
//         Serial.println(longtitude);
//     }
//     else
//     {
//         Serial.println(F("N/A"));
//     }
// }

// void APRS_sendPkt(void *_buffer, size_t length)
// {

//     uint8_t *buffer = (uint8_t *)_buffer;

//     memcpy(dst.call, DST, 6);
//     dst.ssid = DST_SSID;

//     memcpy(src.call, CALL, 6);
//     src.ssid = CALL_SSID;

//     memcpy(path1.call, PATH1, 6);
//     path1.ssid = PATH1_SSID;

//     memcpy(path2.call, PATH2, 6);
//     path2.ssid = PATH2_SSID;

//     path[0] = dst;
//     path[1] = src;
//     path[2] = path1;
//     path[3] = path2;

//     ax25_sendVia(&AX25, path, countof(path), buffer, length);
// }

// // Dynamic RAM usage of this function is 30 bytes
// void APRS_sendLoc(void *_buffer, size_t length)
// {
//     size_t payloadLength = 20 + length;
//     bool usePHG = false;
//     if (power < 10 && height < 10 && gain < 10 && directivity < 9)
//     {
//         usePHG = true;
//         payloadLength += 7;
//     }
//     uint8_t *packet = (uint8_t *)calloc(payloadLength, sizeof(uint8_t));
//     uint8_t *ptr = packet;
//     packet[0] = '=';
//     packet[9] = symbolTable;
//     packet[19] = symbol;
//     ptr++;
//     memcpy(ptr, latitude, 8);
//     ptr += 9;
//     memcpy(ptr, longtitude, 9);
//     ptr += 10;
//     if (usePHG)
//     {
//         packet[20] = 'P';
//         packet[21] = 'H';
//         packet[22] = 'G';
//         packet[23] = power + 48;
//         packet[24] = height + 48;
//         packet[25] = gain + 48;
//         packet[26] = directivity + 48;
//         ptr += 7;
//     }
//     if (length > 0)
//     {
//         uint8_t *buffer = (uint8_t *)_buffer;
//         memcpy(ptr, buffer, length);
//     }

//     APRS_sendPkt(packet, payloadLength);
//     free(packet);
// }

// // Dynamic RAM usage of this function is 18 bytes
// void APRS_sendMsg(void *_buffer, size_t length)
// {
//     if (length > 67)
//         length = 67;
//     size_t payloadLength = 11 + length + 4;

//     uint8_t *packet = (uint8_t *)calloc(payloadLength, sizeof(uint8_t));
//     uint8_t *ptr = packet;
//     packet[0] = ':';
//     int callSize = 6;
//     int count = 0;
//     while (callSize--)
//     {
//         if (message_recip[count] != 0)
//         {
//             packet[1 + count] = message_recip[count];
//             count++;
//         }
//     }
//     if (message_recip_ssid != -1)
//     {
//         packet[1 + count] = '-';
//         count++;
//         if (message_recip_ssid < 10)
//         {
//             packet[1 + count] = message_recip_ssid + 48;
//             count++;
//         }
//         else
//         {
//             packet[1 + count] = 49;
//             count++;
//             packet[1 + count] = message_recip_ssid - 10 + 48;
//             count++;
//         }
//     }
//     while (count < 9)
//     {
//         packet[1 + count] = ' ';
//         count++;
//     }
//     packet[1 + count] = ':';
//     ptr += 11;
//     if (length > 0)
//     {
//         uint8_t *buffer = (uint8_t *)_buffer;
//         memcpy(ptr, buffer, length);
//         memcpy(lastMessage, buffer, length);
//         lastMessageLen = length;
//     }

//     message_seq++;
//     if (message_seq > 999)
//         message_seq = 0;

//     packet[11 + length] = '{';
//     int n = message_seq % 10;
//     int d = ((message_seq % 100) - n) / 10;
//     int h = (message_seq - d - n) / 100;

//     packet[12 + length] = h + 48;
//     packet[13 + length] = d + 48;
//     packet[14 + length] = n + 48;

//     APRS_sendPkt(packet, payloadLength);
//     free(packet);
// }

// void APRS_msgRetry()
// {
//     message_seq--;
//     APRS_sendMsg(lastMessage, lastMessageLen);
// }

int freeMemory()
{
    int free_memory = ESP.getFreeHeap();
    return free_memory;
}
