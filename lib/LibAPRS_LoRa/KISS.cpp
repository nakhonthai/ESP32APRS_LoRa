#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "KISS.h"
#include "AX25.h"

extern size_t ctxbufflen;
size_t frame_len;
extern uint8_t *ctxbuffer;

static uint8_t serialBuffer[AX25_MAX_FRAME_LEN]; // Buffer for holding incoming serial data
// AX25Ctx testkiss;
extern AX25Ctx AX25;
extern void aprs_msg_callback(struct AX25Msg *msg);

bool IN_FRAME = false;
bool ESCAPE = false;

uint8_t command = CMD_UNKNOWN;
extern unsigned long custom_preamble;
extern unsigned long custom_tail;

unsigned long slotTime = 200;
uint8_t p = 63;

int kiss_wrapper(uint8_t *pkg)
{
    uint8_t *ptr = pkg;
    int size = 0;
    *ptr++ = FEND;
    *ptr++ = 0x00;
    for (unsigned i = 0; i < ctxbufflen; i++)
    {
        uint8_t b = ctxbuffer[i];
        if (b == FEND)
        {
            *ptr++ = FESC;
            *ptr++ = TFEND;
        }
        else if (b == FESC)
        {
            *ptr++ = FESC;
            *ptr++ = TFESC;
        }
        else
        {
            *ptr++ = b;
        }
    }
    *ptr++ = FEND;
    size = ptr - pkg;
    return size;
}
void kiss_serial(uint8_t sbyte)
{
    if (IN_FRAME && sbyte == FEND && command == CMD_DATA)
    {
        IN_FRAME = false;
        // ax25_sendRaw(&AX25, serialBuffer, frame_len);
        log_d("[KISS] Received packet! %d Byte", frame_len);
        ax25_init(&AX25, aprs_msg_callback);
        AX25.frame_len = frame_len;
        memcpy(AX25.buf, serialBuffer, frame_len);
        ax25_decode(&AX25);
    }
    else if (sbyte == FEND)
    {
        IN_FRAME = true;
        command = CMD_UNKNOWN;
        frame_len = 0;
    }
    else if (IN_FRAME && frame_len < AX25_MAX_FRAME_LEN)
    {
        // Have a look at the command byte first
        if (frame_len == 0 && command == CMD_UNKNOWN)
        {
            // MicroModem supports only one HDLC port, so we
            // strip off the port nibble of the command byte
            sbyte = sbyte & 0x0F;
            command = sbyte;
        }
        else if (command == CMD_DATA)
        {
            if (sbyte == FESC)
            {
                ESCAPE = true;
            }
            else
            {
                if (ESCAPE)
                {
                    if (sbyte == TFEND)
                        sbyte = FEND;
                    if (sbyte == TFESC)
                        sbyte = FESC;
                    ESCAPE = false;
                }
                serialBuffer[frame_len++] = sbyte;
            }
        }
        else if (command == CMD_TXDELAY)
        {
            custom_preamble = sbyte * 10UL;
        }
        else if (command == CMD_TXTAIL)
        {
            custom_tail = sbyte * 10;
        }
        else if (command == CMD_SLOTTIME)
        {
            slotTime = sbyte * 10;
        }
        else if (command == CMD_P)
        {
            p = sbyte;
        }
    }
}