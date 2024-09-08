#include <RadioLib.h>
#include "Arduino.h"
#include "AX25.h"
#include "LibAPRSesp.h"

// PhysicalLayer *lora; // TODO: Remove this

ICACHE_RAM_ATTR IRadioHal *radioHal;

extern float rssi;
extern float snr;
extern float freqErr;
extern bool afskSync;

// float _freq;

AX25Ctx AX25;
extern void aprs_msg_callback(struct AX25Msg *msg);

uint8_t chip_type = RF_SX1262;

extern Configuration config;

// Radio::Radio()
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
SPIClass spi(FSPI);
#else
SPIClass spi(VSPI);
#endif

#define countof(a) sizeof(a) / sizeof(a[0])

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

ICACHE_RAM_ATTR void setFlag()
{
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
    // put module back to listen mode
    radioHal->startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt();
}

void radioRecvStatus()
{

    // rssi = radioHal->getRSSI(false, true);
    rssi = radioHal->getRSSI(true, false);
    snr = radioHal->getSNR();
    freqErr = radioHal->getFrequencyError();
    // print RSSI (Received Signal Strength Indicator)
    log_d("[LoRa] RSSI:%.0f dBm\tSNR:%.0f dBm\tFreqErr:%.0f Hz", rssi, snr, freqErr);
    // Serial.print(radioHal->getRSSI());
    // Serial.println(F(" dBm"));
    // dBm = (int16_t)rssi;
    afskSync = true;
}

void APRS_sendTNC2Pkt(String raw)
{
    if (raw.substring(0, 3) == "<\xff\x01")
    {
        radioHal->transmit((uint8_t *)raw.c_str(), raw.length());
        startRx();
    }
    else
    {
        char *str = (char *)calloc(raw.length(), sizeof(char));
        if (str)
        {
            ax25frame *frame = (ax25frame *)calloc(1, sizeof(ax25frame));
            if (frame)
            {

                raw.toCharArray(str, raw.length());
                if (ax25_encode(*frame, str, raw.length()))
                {
                    // Serial.println("ax25 send...");
                    ax25sendFrame(&AX25, frame);
                }
                free(frame);
                // Serial.println("ax25 send sussesed...");
            }
            free(str);
        }
    }
}

int APRS_getTNC2Pkt(uint8_t *raw, String info)
{
    ax25frame frame;
    // Serial.println(raw);
    ax25_init(&AX25, aprs_msg_callback);
    ax25_encode(frame, (char *)info.c_str(), info.length());
    int sz = ax25getFrame(raw, &AX25, &frame);
    return sz;
}

void APRS_setFreq(float freq)
{
    // radioHal->setFrequency(freq);
    // radioHal->startReceive();
}

void APRS_init(Configuration *cfg)
{
    int state = -1;
    //_freq = cfg->rf_freq;
    if (config.rf_type == 0 || config.rf_type > 24)
        return;

    spi.begin(config.rf_sclk_gpio, config.rf_miso_gpio, config.rf_mosi_gpio, config.rf_nss_gpio);
    log_d("[SX12xx] Initializing .. ");
    if (cfg->rf_type == RF_SX1278)
    {
        log_d("Init chip SX1278");
        radioHal = new RadioHal<SX1278>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_busy_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
    }
    else if (cfg->rf_type == RF_SX1272)
    {
        log_d("Init chip SX1272");
        radioHal = new RadioHal<SX1272>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_busy_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
    }
    else if (cfg->rf_type == RF_SX1273)
    {
        log_d("Init chip SX1273");
        radioHal = new RadioHal<SX1273>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_busy_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
    }
    else if (cfg->rf_type == RF_SX1276)
    {
        log_d("Init chip SX1276");
        radioHal = new RadioHal<SX1276>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_busy_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
    }
    else if (cfg->rf_type == RF_SX1279)
    {
        log_d("Init chip SX1279");
        radioHal = new RadioHal<SX1279>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_busy_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
    }
    else if (cfg->rf_type == RF_SX1268)
    {
        log_d("Init chip SX1268");
        radioHal = new RadioHal<SX1268>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_busy_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
    }
    else if (cfg->rf_type == RF_SX1261)
    {
        log_d("Init chip SX1262");
        radioHal = new RadioHal<SX1261>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_busy_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
    }
    else if (cfg->rf_type == RF_SX1262)
    {
        log_d("Init chip SX1262");
        radioHal = new RadioHal<SX1262>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_busy_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
    }
    else if (cfg->rf_type == RF_SX1280)
    {
        log_d("Init chip SX1280");
        radioHal = new RadioHal<SX1280>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_busy_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
    }
    else if (cfg->rf_type == RF_SX1281)
    {
        log_d("Init chip SX1281");
        radioHal = new RadioHal<SX1281>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_busy_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
    }
    else if (cfg->rf_type == RF_SX1282)
    {
        log_d("Init chip SX1282");
        radioHal = new RadioHal<SX1282>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_busy_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
    }
    // else if (cfg->rf_type == RF_SX1231)
    // {
    //     log_d("Init chip SX1231");
    //     radioHal = new RadioHal<SX1231>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_busy_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
    // }
    // else if (cfg->rf_type == RF_SX1233)
    // {
    //     log_d("Init chip SX1233");
    //     radioHal = new RadioHal<SX1233>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_busy_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
    // }
    else
    {
        log_d("Init chip Unknow default SX1268");
        radioHal = new RadioHal<SX1268>(new Module(config.rf_nss_gpio, config.rf_dio1_gpio, config.rf_reset_gpio, config.rf_busy_gpio, spi, SPISettings(2000000, MSBFIRST, SPI_MODE0)));
    }

    if (cfg->rf_rx_gpio != -1 && config.rf_tx_gpio != -1)
    {
        radioHal->setRfSwitchPins(config.rf_rx_gpio, config.rf_tx_gpio);
        log_d("setRfSwitchPins(RxEn->GPIO %d, TxEn->GPIO %d)", config.rf_rx_gpio, config.rf_tx_gpio);
    }

    log_d("[%d] Begin... ", config.rf_type);
    if ((cfg->rf_type == RF_SX1231) || (cfg->rf_type == RF_SX1233))
    {
        state = radioHal->beginFSK(config.rf_freq + (config.rf_freq_offset / 1000000.0), config.rf_baudrate, config.rf_bw, config.rf_bw, config.rf_power, config.rf_preamable, 0, 1.6);
    }
    else
    {
        state = radioHal->begin(config.rf_freq + (config.rf_freq_offset / 1000000.0), config.rf_bw, config.rf_sf, config.rf_cr, config.rf_sync, config.rf_power, config.rf_preamable, 1, 1.6);
    }
    if (state == RADIOLIB_ERR_NONE)
    {
        log_d("[LoRa] Init success!");
    }
    else
    {
        log_d("[LoRa] Init failed, code %d", state);
    }

    // radioHal->setsetPacketReceivedAction(setFlagRx);
    radioHal->setDio0Action(setFlag);

    state = radioHal->startReceive();

    if (state == RADIOLIB_ERR_NONE)
    {
        log_d("[LoRa] Start receive success!");
    }
    else
    {
        log_d("[LoRa] Start receive failed, code %d", state);
    }

    enableInterrupt();
    ax25_init(&AX25, aprs_msg_callback);
}

bool APRS_poll(void)
{
    bool ret = false;
    // check if the flag is set
    if (received)
    {        
        // disable the interrupt service routine while
        // processing the data
        disableInterrupt();

        LED_Status(0,200,0);

        received = false;
        uint8_t *byteArr = (uint8_t *)calloc(350, sizeof(uint8_t));
        if (byteArr)
        {
            int numBytes = 0;
            int state = -1;

            numBytes = radioHal->getPacketLength();
            memset(byteArr, 0, 350);
            state = radioHal->readData(&byteArr[0], numBytes);
            radioRecvStatus();
            log_d("[LoRa] Received packet! %d Byte\n", numBytes);
            if (state == RADIOLIB_ERR_NONE)
            {
                // packet was successfully received
                // log_d("[LoRa] Received packet! %d Byte\n", numBytes);
                if (numBytes > 0)
                {
                    if (byteArr[0] == 0x7E)
                    {
                        rx_Fifo_flush();
                        for (int i = 0; i < numBytes; i++)
                        {
                            rx_putchar((char)byteArr[i]);
                        }
                    }
                    else if (byteArr[0] == '<' && byteArr[1] == 0xFF && byteArr[2] == 0x01)
                    {
                        String str = "";
                        // str.getBytes(byteArr, numBytes - 3,3);
                        //  print data of the packet

                        for (int i = 0; i < numBytes - 3; i++)
                        {
                            str += String((char)byteArr[i + 3]);
                            // str.setCharAt(i,byteArr[i+3]);
                        }
                        Serial.print(F("[SX1278] Data:\t\t"));
                        Serial.println(str);
                        // Add RSSI in comment
                        memset(byteArr, 0, 350);
                        int size = APRS_getTNC2Pkt(byteArr, str);
                        rx_Fifo_flush();
                        for (int i = 0; i < size; i++)
                        {
                            rx_putchar((char)byteArr[i]);
                        }
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
            free(byteArr);
        }
        startRx();
        ax25_poll(&AX25);
        LED_Status(0,0,0);
    }
    else
    {
        if (ax25_stateTx)
        {            
            ax25_stateTx = false;
            LED_Status(200,0,0);
            // flagTx = true;
            uint8_t *byteArr = (uint8_t *)calloc(250, sizeof(uint8_t));
            if (byteArr)
            {
                // Serial.print("TX HEX: ");
                int i;
                memset(byteArr, 0, 250);
                for (i = 0; i < 250; i++)
                {
                    int c = tx_getchar();
                    if (c == -1)
                        break;
                    else
                        byteArr[i] = (uint8_t)c;

                    // Serial.printf("%0X ",byteArr[i]);
                }

                // transmissionState = radioHal->startTransmit(byteArr, i);
                radioHal->transmit(byteArr, i);
                // radioHal->setDio0Action(setFlag); // TODO: Check, is this needed?? include it inside startRX ??
                startRx();
                // flagTx = false;
                free(byteArr);
                ret = true;
            }
            LED_Status(0,0,0);
        }
        else
        {
            ax25_poll(&AX25);
        }
    }
    return ret;
}

void APRS_setCallsign(char *call, int ssid)
{
    memset(CALL, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0)
    {
        CALL[i] = call[i];
        i++;
    }
    CALL_SSID = ssid;
}

void APRS_setDestination(char *call, int ssid)
{
    memset(DST, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0)
    {
        DST[i] = call[i];
        i++;
    }
    DST_SSID = ssid;
}

void APRS_setPath1(char *call, int ssid)
{
    memset(PATH1, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0)
    {
        PATH1[i] = call[i];
        i++;
    }
    PATH1_SSID = ssid;
}

void APRS_setPath2(char *call, int ssid)
{
    memset(PATH2, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0)
    {
        PATH2[i] = call[i];
        i++;
    }
    PATH2_SSID = ssid;
}

void APRS_setMessageDestination(char *call, int ssid)
{
    memset(message_recip, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0)
    {
        message_recip[i] = call[i];
        i++;
    }
    message_recip_ssid = ssid;
}

void APRS_setPreamble(unsigned long pre)
{
    custom_preamble = pre;
}

void APRS_setTail(unsigned long tail)
{
    custom_tail = tail;
}

void APRS_useAlternateSymbolTable(bool use)
{
    if (use)
    {
        symbolTable = '\\';
    }
    else
    {
        symbolTable = '/';
    }
}

void APRS_setSymbol(char sym)
{
    symbol = sym;
}

void APRS_setLat(char *lat)
{
    memset(latitude, 0, 9);
    int i = 0;
    while (i < 8 && lat[i] != 0)
    {
        latitude[i] = lat[i];
        i++;
    }
}

void APRS_setLon(char *lon)
{
    memset(longtitude, 0, 10);
    int i = 0;
    while (i < 9 && lon[i] != 0)
    {
        longtitude[i] = lon[i];
        i++;
    }
}

void APRS_setPower(int s)
{
    if (s >= 0 && s < 10)
    {
        power = s;
    }
}

void APRS_setHeight(int s)
{
    if (s >= 0 && s < 10)
    {
        height = s;
    }
}

void APRS_setGain(int s)
{
    if (s >= 0 && s < 10)
    {
        gain = s;
    }
}

void APRS_setDirectivity(int s)
{
    if (s >= 0 && s < 10)
    {
        directivity = s;
    }
}

void APRS_printSettings()
{
    Serial.println(F("LibAPRS Settings:"));
    Serial.print(F("Callsign:     "));
    Serial.print(CALL);
    Serial.print(F("-"));
    Serial.println(CALL_SSID);
    Serial.print(F("Destination:  "));
    Serial.print(DST);
    Serial.print(F("-"));
    Serial.println(DST_SSID);
    Serial.print(F("Path1:        "));
    Serial.print(PATH1);
    Serial.print(F("-"));
    Serial.println(PATH1_SSID);
    Serial.print(F("Path2:        "));
    Serial.print(PATH2);
    Serial.print(F("-"));
    Serial.println(PATH2_SSID);
    Serial.print(F("Message dst:  "));
    if (message_recip[0] == 0)
    {
        Serial.println(F("N/A"));
    }
    else
    {
        Serial.print(message_recip);
        Serial.print(F("-"));
        Serial.println(message_recip_ssid);
    }
    Serial.print(F("TX Preamble:  "));
    Serial.println(custom_preamble);
    Serial.print(F("TX Tail:      "));
    Serial.println(custom_tail);
    Serial.print(F("Symbol table: "));
    if (symbolTable == '/')
    {
        Serial.println(F("Normal"));
    }
    else
    {
        Serial.println(F("Alternate"));
    }
    Serial.print(F("Symbol:       "));
    Serial.println(symbol);
    Serial.print(F("Power:        "));
    if (power < 10)
    {
        Serial.println(power);
    }
    else
    {
        Serial.println(F("N/A"));
    }
    Serial.print(F("Height:       "));
    if (height < 10)
    {
        Serial.println(height);
    }
    else
    {
        Serial.println(F("N/A"));
    }
    Serial.print(F("Gain:         "));
    if (gain < 10)
    {
        Serial.println(gain);
    }
    else
    {
        Serial.println(F("N/A"));
    }
    Serial.print(F("Directivity:  "));
    if (directivity < 10)
    {
        Serial.println(directivity);
    }
    else
    {
        Serial.println(F("N/A"));
    }
    Serial.print(F("Latitude:     "));
    if (latitude[0] != 0)
    {
        Serial.println(latitude);
    }
    else
    {
        Serial.println(F("N/A"));
    }
    Serial.print(F("Longtitude:   "));
    if (longtitude[0] != 0)
    {
        Serial.println(longtitude);
    }
    else
    {
        Serial.println(F("N/A"));
    }
}

void APRS_sendPkt(void *_buffer, size_t length)
{

    uint8_t *buffer = (uint8_t *)_buffer;

    memcpy(dst.call, DST, 6);
    dst.ssid = DST_SSID;

    memcpy(src.call, CALL, 6);
    src.ssid = CALL_SSID;

    memcpy(path1.call, PATH1, 6);
    path1.ssid = PATH1_SSID;

    memcpy(path2.call, PATH2, 6);
    path2.ssid = PATH2_SSID;

    path[0] = dst;
    path[1] = src;
    path[2] = path1;
    path[3] = path2;

    ax25_sendVia(&AX25, path, countof(path), buffer, length);
}

// Dynamic RAM usage of this function is 30 bytes
void APRS_sendLoc(void *_buffer, size_t length)
{
    size_t payloadLength = 20 + length;
    bool usePHG = false;
    if (power < 10 && height < 10 && gain < 10 && directivity < 9)
    {
        usePHG = true;
        payloadLength += 7;
    }
    uint8_t *packet = (uint8_t *)calloc(payloadLength, sizeof(uint8_t));
    uint8_t *ptr = packet;
    packet[0] = '=';
    packet[9] = symbolTable;
    packet[19] = symbol;
    ptr++;
    memcpy(ptr, latitude, 8);
    ptr += 9;
    memcpy(ptr, longtitude, 9);
    ptr += 10;
    if (usePHG)
    {
        packet[20] = 'P';
        packet[21] = 'H';
        packet[22] = 'G';
        packet[23] = power + 48;
        packet[24] = height + 48;
        packet[25] = gain + 48;
        packet[26] = directivity + 48;
        ptr += 7;
    }
    if (length > 0)
    {
        uint8_t *buffer = (uint8_t *)_buffer;
        memcpy(ptr, buffer, length);
    }

    APRS_sendPkt(packet, payloadLength);
    free(packet);
}

// Dynamic RAM usage of this function is 18 bytes
void APRS_sendMsg(void *_buffer, size_t length)
{
    if (length > 67)
        length = 67;
    size_t payloadLength = 11 + length + 4;

    uint8_t *packet = (uint8_t *)calloc(payloadLength, sizeof(uint8_t));
    uint8_t *ptr = packet;
    packet[0] = ':';
    int callSize = 6;
    int count = 0;
    while (callSize--)
    {
        if (message_recip[count] != 0)
        {
            packet[1 + count] = message_recip[count];
            count++;
        }
    }
    if (message_recip_ssid != -1)
    {
        packet[1 + count] = '-';
        count++;
        if (message_recip_ssid < 10)
        {
            packet[1 + count] = message_recip_ssid + 48;
            count++;
        }
        else
        {
            packet[1 + count] = 49;
            count++;
            packet[1 + count] = message_recip_ssid - 10 + 48;
            count++;
        }
    }
    while (count < 9)
    {
        packet[1 + count] = ' ';
        count++;
    }
    packet[1 + count] = ':';
    ptr += 11;
    if (length > 0)
    {
        uint8_t *buffer = (uint8_t *)_buffer;
        memcpy(ptr, buffer, length);
        memcpy(lastMessage, buffer, length);
        lastMessageLen = length;
    }

    message_seq++;
    if (message_seq > 999)
        message_seq = 0;

    packet[11 + length] = '{';
    int n = message_seq % 10;
    int d = ((message_seq % 100) - n) / 10;
    int h = (message_seq - d - n) / 100;

    packet[12 + length] = h + 48;
    packet[13 + length] = d + 48;
    packet[14 + length] = n + 48;

    APRS_sendPkt(packet, payloadLength);
    free(packet);
}

void APRS_msgRetry()
{
    message_seq--;
    APRS_sendMsg(lastMessage, lastMessageLen);
}

int freeMemory()
{
    int free_memory = ESP.getFreeHeap();
    return free_memory;
}
