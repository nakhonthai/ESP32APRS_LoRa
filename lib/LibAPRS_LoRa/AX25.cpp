// Based on work by Francesco Sacchi

#include "Arduino.h"
#include <string.h>
#include <stdlib.h>
#include <strings.h>
#include <ctype.h>
#include <stdio.h>
#include "AX25.h"
#include "HDLC.h"
#include "CRC-CCIT.h"

FIFOBuffer txFifo;   // FIFO for transmit data
uint16_t txBuf[350]; // Actial data storage for said FIFO

volatile bool sending; // Set when modem is sending

FIFOBuffer rxFifo;   // FIFO for received data
uint16_t rxBuf[350]; // Actual data storage for said FIFO

bool ax25_stateTx = false;

size_t ctxbufflen;
uint8_t *ctxbuffer;

#define typeof(x) __typeof__(x)
#define countof(a) sizeof(a) / sizeof(a[0])
#define MIN(a, b) (                          \
    {                                        \
        typeof(a) _a = (a);                  \
        typeof(b) _b = (b);                  \
        ((typeof(_a))((_a < _b) ? _a : _b)); \
    })
#define DECODE_CALL(buf, addr)                                     \
    for (unsigned i = 0; i < sizeof((addr)) - CALL_OVERSPACE; i++) \
    {                                                              \
        char c = (*(buf)++ >> 1);                                  \
        (addr)[i] = (c == ' ') ? '\x0' : c;                        \
    }
#define AX25_SET_REPEATED(msg, idx, val)   \
    do                                     \
    {                                      \
        if (val)                           \
        {                                  \
            (msg)->rpt_flags |= _BV(idx);  \
        }                                  \
        else                               \
        {                                  \
            (msg)->rpt_flags &= ~_BV(idx); \
        }                                  \
    } while (0)

extern int LibAPRS_vref;

static void ax25_putchar(AX25Ctx *ctx, uint8_t c);

/******************************************************************************/
void updcrc(uint16_t *crc, unsigned char rxbyte)
/*******************************************************************************
 * ABSTRACT:	This function calculates the crc of the incomming message.
 *
 * INPUT:		txbyte	The byte to transmit
 * OUTPUT:	None
 * RETURN:	None
 */
{
    unsigned char loop; // Generic loop variable
    uint16_t lsb_int;   // LSBit of incoming byte
    uint16_t xor_int;   // Used for the IF statement

    for (loop = 0; loop < 8; loop++) // Loop through all eight bits
    {
        lsb_int = rxbyte & 0x01;  // Set aside the least significant bit
        xor_int = *crc ^ lsb_int; // XOR lsb of CRC with the latest bit
        *crc >>= 1;               // Shift 16-bit CRC one bit to the right
        if (xor_int & 0x0001)     // If XOR result from above has lsb set
        {
            *crc ^= 0x8408; // XOR the crc with magic number
        }

        rxbyte >>= 1; // Shift the reference byte one bit right
    }

    return;

} // End ax25rxByte(unsigned char rxbyte)

int hdlc_flag_count = 0;
bool hdlc_flage_end = false;
bool sync_flage = false;
uint16_t hdlc_data_count = 0;
bool hdlcParse(Hdlc *hdlc, bool bit, FIFOBuffer *fifo)
{
    // Initialise a return value. We start with the
    // assumption that all is going to end well :)
    bool ret = true;

    // Bitshift our byte of demodulated bits to
    // the left by one bit, to make room for the
    // next incoming bit
    hdlc->demodulatedBits <<= 1;
    // And then put the newest bit from the
    // demodulator into the byte.
    hdlc->demodulatedBits |= bit ? 1 : 0;

    // Now we'll look at the last 8 received bits, and
    // check if we have received a HDLC flag (01111110)
    if (hdlc->demodulatedBits == HDLC_FLAG)
    {
        // If we have, check that our output buffer is
        // not full.
        if (!fifo_isfull(fifo))
        {
            // If it isn't, we'll push the HDLC_FLAG into
            // the buffer and indicate that we are now
            // receiving data. For bling we also turn
            // on the RX LED.
            if (++hdlc_flag_count > 3)
            {
                hdlc->receiving = true;
                fifo_flush(fifo);
                // LED_RX_ON();
                sync_flage = true;
                fifo_push(fifo, HDLC_FLAG);
            }
        }
        else
        {
            // If the buffer is full, we have a problem
            // and abort by setting the return value to
            // false and stopping the here.

            ret = false;
            hdlc->receiving = false;
            // LED_RX_OFF();
            hdlc_flag_count = 0;
            hdlc_flage_end = false;
        }

        // Everytime we receive a HDLC_FLAG, we reset the
        // storage for our current incoming byte and bit
        // position in that byte. This effectively
        // synchronises our parsing to  the start and end
        // of the received bytes.
        hdlc->currentByte = 0;
        hdlc->bitIndex = 0;
        return ret;
    }

    // Check if we have received a RESET flag (01111111)
    // In this comparison we also detect when no transmission
    // (or silence) is taking place, and the demodulator
    // returns an endless stream of zeroes. Due to the NRZ
    // coding, the actual bits send to this function will
    // be an endless stream of ones, which this AND operation
    // will also detect.
    if ((hdlc->demodulatedBits & HDLC_RESET) == HDLC_RESET)
    {
        // If we have, something probably went wrong at the
        // transmitting end, and we abort the reception.
        hdlc->receiving = false;
        // LED_RX_OFF();
        hdlc_flag_count = 0;
        hdlc_flage_end = false;
        fifo_flush(fifo);
        return ret;
    }

    // If we have not yet seen a HDLC_FLAG indicating that
    // a transmission is actually taking place, don't bother
    // with anything.
    if (!hdlc->receiving)
        return ret;

    sync_flage = false;
    // hdlc_flage_end = true;

    // First check if what we are seeing is a stuffed bit.
    // Since the different HDLC control characters like
    // HDLC_FLAG, HDLC_RESET and such could also occur in
    // a normal data stream, we employ a method known as
    // "bit stuffing". All control characters have more than
    // 5 ones in a row, so if the transmitting party detects
    // this sequence in the _data_ to be transmitted, it inserts
    // a zero to avoid the receiving party interpreting it as
    // a control character. Therefore, if we detect such a
    // "stuffed bit", we simply ignore it and wait for the
    // next bit to come in.
    //
    // We do the detection by applying an AND bit-mask to the
    // stream of demodulated bits. This mask is 00111111 (0x3f)
    // if the result of the operation is 00111110 (0x3e), we
    // have detected a stuffed bit.
    if ((hdlc->demodulatedBits & 0x3f) == 0x3e)
        return ret;

    // If we have an actual 1 bit, push this to the current byte
    // If it's a zero, we don't need to do anything, since the
    // bit is initialized to zero when we bitshifted earlier.
    if (hdlc->demodulatedBits & 0x01)
        hdlc->currentByte |= 0x80;

    // Increment the bitIndex and check if we have a complete byte
    if (++hdlc->bitIndex >= 8)
    {
        // If we have a HDLC control character, put a AX.25 escape
        // in the received data. We know we need to do this,
        // because at this point we must have already seen a HDLC
        // flag, meaning that this control character is the result
        // of a bitstuffed byte that is equal to said control
        // character, but is actually part of the data stream.
        // By inserting the escape character, we tell the protocol
        // layer that this is not an actual control character, but
        // data.
        if ((hdlc->currentByte == HDLC_FLAG ||
             hdlc->currentByte == HDLC_RESET ||
             hdlc->currentByte == AX25_ESC))
        {
            // We also need to check that our received data buffer
            // is not full before putting more data in
            if (!fifo_isfull(fifo))
            {
                fifo_push(fifo, AX25_ESC);
            }
            else
            {
                // If it is, abort and return false
                hdlc->receiving = false;
                // LED_RX_OFF();
                hdlc_flag_count = 0;
                ret = false;
                log_d("FIFO IS FULL");
            }
        }

        // Push the actual byte to the received data FIFO,
        // if it isn't full.
        if (!fifo_isfull(fifo))
        {
            fifo_push(fifo, hdlc->currentByte);
            // log_printf("|%0X",hdlc->currentByte);
            hdlc_flag_count = 0;
        }
        else
        {
            // If it is, well, you know by now!
            hdlc->receiving = false;
            // LED_RX_OFF();
            hdlc_flag_count = 0;
            ret = false;
            log_d("FIFO IS FULL");
        }

        // Wipe received byte and reset bit index to 0
        hdlc->currentByte = 0;
        hdlc->bitIndex = 0;
    }
    else
    {
        // We don't have a full byte yet, bitshift the byte
        // to make room for the next bit
        hdlc->currentByte >>= 1;
    }

    return ret;
}

bool hdlcParseBit(Hdlc *hdlc, bool bit, uint8_t *data, size_t &idx, bool ax25)
{

    bool ret = true;
    hdlc->demodulatedBits <<= 1;
    hdlc->demodulatedBits |= bit ? 1 : 0;
    if (hdlc->demodulatedBits == HDLC_FLAG)
    {
        if (idx < 10)
        {
            hdlc->receiving = true;
            idx = 0;
            sync_flage = true;
        }
        hdlc->currentByte = 0;
        hdlc->bitIndex = 0;
        return false;
    }
    if ((hdlc->demodulatedBits & HDLC_RESET) == HDLC_RESET)
    {
        hdlc->receiving = false;
        hdlc_flage_end = false;
        idx = 0;
        return false;
    }

    if (!hdlc->receiving)
        return false;

    sync_flage = false;
    // This mask is 00111111 (0x3f)
    //  if the result of the operation is 00111110 (0x3e), we
    //  have detected a stuffed bit.
    if ((hdlc->demodulatedBits & 0x3f) == 0x3e)
        return false;

    if (hdlc->demodulatedBits & 0x01)
        hdlc->currentByte |= 0x80;

    // Increment the bitIndex and check if we have a complete byte
    if (++hdlc->bitIndex >= 8)
    {
        if (ax25)
        {
            if ((hdlc->currentByte == HDLC_FLAG ||
                 hdlc->currentByte == HDLC_RESET ||
                 hdlc->currentByte == AX25_ESC))
            {
                data[idx++] = AX25_ESC;
            }
        }

        data[idx++] = hdlc->currentByte;

        // Wipe received byte and reset bit index to 0
        hdlc->currentByte = 0;
        hdlc->bitIndex = 0;
        return true;
    }
    else
    {
        hdlc->currentByte >>= 1;
    }

    return false;
}

int unbit_stuffing(uint8_t *outbuf, size_t outbuf_size, uint8_t *inbuf, size_t inbuf_size)
{
    int data_cnt = 0; // AX.25 packet start
    bool delete_zero = false;
    int count_ones = 0;
    uint8_t data = 0;
    int data_bits = 0;
    int buf_index = 0;
    uint16_t crc_in = CRC_CCIT_INIT_VAL;

    if (inbuf[0] == 0x7E)
        data_cnt = 1;

    while (data_cnt < inbuf_size)
    {
        uint8_t bitq = inbuf[data_cnt++];
        int bitq_bits = 8;
        while (bitq_bits-- > 0)
        {
            uint8_t bit = bitq & 1;
            bitq >>= 1;
            if (delete_zero)
            {
                if (bit == 0)
                { // bit stuffing bit
                    delete_zero = false;
                    continue;
                }
                if (data_bits == 6)
                { // may be end flag (7e)
                    crc_in = fcs_calc(outbuf, buf_index - 2);
                    // log_d("actual_fcs=%02X%02X  expected_fcs=%04X",outbuf[buf_index-1]<<8),outbuf[buf_index-2],crc_in);
                    if (crc_in == (((uint16_t)outbuf[buf_index - 1] << 8) | ((uint16_t)outbuf[buf_index - 2])))
                    {
                        // log_d("CRC=%0X",crc_in);
                        return buf_index;
                    }
                    return -1;
                }
                return -1; // error
            }
            if (bit)
            {
#define BIT_STUFFING_BITS 5
                if (++count_ones >= BIT_STUFFING_BITS)
                {
                    delete_zero = true;
                    count_ones = 0;
                }
            }
            else
            {

                count_ones = 0;
            }
            data |= bit << data_bits;
            if (++data_bits >= 8)
            {
                outbuf[buf_index++] = data;
                // crc_in=update_crc_ccit(data, crc_in);
                if (buf_index >= outbuf_size)
                {
                    return buf_index;
                }

                data = 0;
                data_bits = 0;
            }
        }
    }
    return -1; // AX.25 end flag not found
}

// Credit from code https://github.com/amedes/ESP32TNC/blob/master/main/fx25.c
int bit_stuffing(uint8_t *outbuf, size_t outbuf_size, uint8_t *inbuf, size_t inbuf_size)
{
    int bit_len = 0;
    int outbuf_index = 0;
    uint8_t data = 0;  // data to modem
    int data_bits = 0; // number of bits
    int count_ones = 0;
    bool insert_zero = false;
    bool do_bitstuffing = true;

    for (int i = 0; i < inbuf_size; i++)
    {
        uint8_t bitq = inbuf[i]; // bit queue
        int bitq_bits = 8;       // number of bits

        if ((i == 0) || (i == (inbuf_size - 1)))
            do_bitstuffing = false;
        else
            do_bitstuffing = true;

        // bit stuffing
        while (bitq_bits-- > 0)
        {
            int bit;
            if (insert_zero)
            {
                bit = 0;
                insert_zero = false;
            }
            else
            {
                bit = bitq & 1;
                bitq >>= 1;
                if (do_bitstuffing)
                {
                    if (bit)
                    { // "one"
#define BIT_STUFFING_BITS 5
                        if (++count_ones >= BIT_STUFFING_BITS)
                        { // insert zero
                            insert_zero = true;
                            bitq_bits++;
                            count_ones = 0;
                        }
                    }
                    else
                    { // "zero"
                        count_ones = 0;
                    }
                }
            }

            data |= bit << data_bits;
            if (++data_bits >= 8)
            { // filled all 8 bits
                if (outbuf_index < outbuf_size)
                    outbuf[outbuf_index++] = data;
                bit_len += data_bits;
                data = 0;
                data_bits = 0;
            }
        } // while (bitq_bits-- > 0)
    } // for (i = 0; ..

    if (data_bits > 0)
    { // there is data to be sent
        if (outbuf_index < outbuf_size)
        {
            outbuf[outbuf_index++] = data | (HDLC_FLAG << data_bits); // padding with AX.25 flag
        }
        bit_len += data_bits;
    }
    // return bit_len;
    return outbuf_index;
}

void tx_putchar(char c)
{
    // while (fifo_isfull_locked(&txFifo))
    //{
    /* Wait */
    // delay(10);
    //}
    fifo_push_locked(&txFifo, c);
}

int tx_getchar(void)
{
    if (fifo_isempty_locked(&txFifo))
    {
        return EOF;
    }
    else
    {
        return fifo_pop_locked(&txFifo);
    }
}

void rx_putchar(char c)
{
    // while (fifo_isfull_locked(&rxFifo))
    //{
    /* Wait */
    // delay(10);
    //}
    fifo_push_locked(&rxFifo, c);
}
int rx_getchar(void)
{
    if (fifo_isempty_locked(&rxFifo))
    {
        return EOF;
    }
    else
    {
        return fifo_pop_locked(&rxFifo);
    }
}

void rx_Fifo_flush()
{
    fifo_flush(&rxFifo);
}

void ax25_init(AX25Ctx *ctx, ax25_callback_t hook)
{
    fifo_init(&rxFifo, rxBuf, sizeof(rxBuf));
    fifo_init(&txFifo, txBuf, sizeof(txBuf));
    memset(ctx, 0, sizeof(*ctx));
    ctx->hook = hook;
    ctx->crc_in = ctx->crc_out = CRC_CCIT_INIT_VAL;
}

void ax25_decode(AX25Ctx *ctx)
{
    AX25Msg msg;
    ctxbufflen = ctx->frame_len - 2;
    ctxbuffer = ctx->buf;
    uint8_t *buf = ctx->buf;

    DECODE_CALL(buf, msg.dst.call);
    msg.dst.ssid = (*buf++ >> 1) & 0x0F;
    msg.dst.call[6] = 0;

    DECODE_CALL(buf, msg.src.call);
    msg.src.ssid = (*buf >> 1) & 0x0F;
    msg.src.call[6] = 0;

    for (msg.rpt_count = 0; !(*buf++ & 0x01) && (msg.rpt_count < countof(msg.rpt_list)); msg.rpt_count++)
    {
        DECODE_CALL(buf, msg.rpt_list[msg.rpt_count].call);
        msg.rpt_list[msg.rpt_count].ssid = (*buf >> 1) & 0x0F;
        AX25_SET_REPEATED(&msg, msg.rpt_count, (*buf & 0x80));
        msg.rpt_list[msg.rpt_count].call[6] = 0;
    }

    msg.ctrl = *buf++;
    if (msg.ctrl != AX25_CTRL_UI)
    {
        return;
    }

    msg.pid = *buf++;
    if (msg.pid != AX25_PID_NOLAYER3)
    {
        return;
    }

    memset(msg.info, 0, sizeof(msg.info));
    msg.len = ctx->frame_len - 2 - (buf - ctx->buf);
    memcpy(msg.info, buf, msg.len);
    // msg.info[msg.len]=0;
    // msg.info = buf;

    if (ctx->hook)
    {
        cli();
        ctx->hook(&msg);
        sei();
    }
}

void ax25_poll(AX25Ctx *ctx)
{
    int c;

    while ((c = rx_getchar()) != EOF)
    {
        // log_printf("%0X,",c);
        if (!ctx->escape && c == HDLC_FLAG)
        {
            // if (ctx->frame_len >= AX25_MIN_FRAME_LEN)
            // {
            if (ctx->crc_in == AX25_CRC_CORRECT)
            {
                // End Flag 7E
                ctx->sync = false;
                ctx->crc_in = CRC_CCIT_INIT_VAL;
                ctx->frame_len = 0;
                continue;
            }
            //}
            // Sync and Start Flag 7E
            ctx->sync = true;
            ctx->crc_in = CRC_CCIT_INIT_VAL;
            ctx->frame_len = 0;
            continue;
        }

        if (!ctx->escape && c == HDLC_RESET)
        {
            ctx->sync = false;
            continue;
        }

        if (!ctx->escape && c == AX25_ESC)
        {
            ctx->escape = true;
            continue;
        }

        if (ctx->sync)
        {
            if (ctx->frame_len < AX25_MAX_FRAME_LEN)
            {
                ctx->buf[ctx->frame_len++] = c;
                ctx->crc_in = update_crc_ccit(c, ctx->crc_in);

                if (ctx->crc_in == AX25_CRC_CORRECT)
                {
                    ax25_decode(ctx);
                    ctx->sync = false;
                    ctx->escape = true;
                    ctx->frame_len = 0;
                    memset(ctx->buf, 0, sizeof(ctx->buf));
                }
            }
            else
            {
                ctx->sync = false;
            }
        }
        ctx->escape = false;
    }
}

static void ax25_putchar(AX25Ctx *ctx, uint8_t c)
{
    if (c == HDLC_FLAG || c == HDLC_RESET || c == AX25_ESC)
    {
        tx_putchar(AX25_ESC);
        updcrc(&ctx->crc_out, AX25_ESC);
    }
    else
    {
        updcrc(&ctx->crc_out, c);
    }
    // ctx->crc_out = update_crc_ccit(c, ctx->crc_out);
    tx_putchar(c);
}

void ax25_sendRaw(AX25Ctx *ctx, void *_buf, size_t len)
{
    ctx->crc_out = CRC_CCIT_INIT_VAL;
    tx_putchar(HDLC_FLAG);
    const uint8_t *buf = (const uint8_t *)_buf;
    while (len--)
        ax25_putchar(ctx, *buf++);

    uint8_t crcl = (ctx->crc_out & 0xff) ^ 0xff;
    uint8_t crch = (ctx->crc_out >> 8) ^ 0xff;
    ax25_putchar(ctx, crcl);
    ax25_putchar(ctx, crch);

    tx_putchar(HDLC_FLAG);
    ax25_stateTx = true;
}

void ax25_sendBuf(uint8_t *buf, size_t len)
{
    fifo_flush(&txFifo);
    // tx_putchar(HDLC_FLAG);
    for (int i = 0; i < len; i++)
        tx_putchar(buf[i]);
    // const uint8_t *buf = (const uint8_t *)_buf;
    // while (len--)
    //     tx_putchar(*buf++);

    // tx_putchar(HDLC_FLAG);
    ax25_stateTx = true;
}

static void ax25_sendCall(AX25Ctx *ctx, const AX25Call *addr, bool last)
{
    unsigned len = MIN((sizeof(addr->call) - CALL_OVERSPACE), strlen(addr->call));

    for (unsigned i = 0; i < len; i++)
    {
        uint8_t c = addr->call[i];
        c = toupper(c);
        ax25_putchar(ctx, c << 1);
    }

    if (len < (sizeof(addr->call) - CALL_OVERSPACE))
    {
        for (unsigned i = 0; i < (sizeof(addr->call) - CALL_OVERSPACE) - len; i++)
        {
            ax25_putchar(ctx, ' ' << 1);
        }
    }

    uint8_t ssid = 0x60 | (addr->ssid << 1) | (last ? 0x01 : 0);
    ax25_putchar(ctx, ssid);
}

void ax25_sendVia(AX25Ctx *ctx, const AX25Call *path, size_t path_len, const void *_buf, size_t len)
{
    const uint8_t *buf = (const uint8_t *)_buf;

    ctx->crc_out = CRC_CCIT_INIT_VAL;
    tx_putchar(HDLC_FLAG);

    for (size_t i = 0; i < path_len; i++)
    {
        ax25_sendCall(ctx, &path[i], (i == path_len - 1));
    }

    ax25_putchar(ctx, AX25_CTRL_UI);
    ax25_putchar(ctx, AX25_PID_NOLAYER3);

    while (len--)
    {
        ax25_putchar(ctx, *buf++);
    }

    uint8_t crcl = (ctx->crc_out & 0xff) ^ 0xff;
    uint8_t crch = (ctx->crc_out >> 8) ^ 0xff;
    ax25_putchar(ctx, crcl);
    ax25_putchar(ctx, crch);

    tx_putchar(HDLC_FLAG);
    ax25_stateTx = true;
}

unsigned int strpos(char *txt, char chk)
{
    char *pch;
    unsigned int idx = 0;
    pch = strchr(txt, chk);
    idx = pch - txt;
    return idx;
}

void convPath(ax25header *hdr, char *txt, unsigned int size)
{
    unsigned int i, p, j;
    char num[5];
    hdr->ssid = 0;
    memset(hdr->addr, 0, 7);
    memset(&num[0], 0, sizeof(num));

    p = strpos(txt, '-');
    if (p > 0 && p < size)
    {
        for (i = 0; i < p; i++)
        { // Get CallSign/Path
            hdr->addr[i] = txt[i];
        }
        j = 0;
        for (i = p + 1; i < size; i++)
        { // get SSID
            //            if(txt[i]=='*') break;
            //            if(txt[i]==',') break;
            //            if(txt[i]==':') break;
            if (txt[i] < 0x30)
                break;
            if (txt[i] > 0x39)
                break;
            num[j++] = txt[i];
        }
        if (j > 0)
        {
            hdr->ssid = atoi(num);
        }
        hdr->ssid <<= 1;
    }
    else
    {
        for (i = 0; i < size; i++)
        { // Get CallSign/Path
            if (txt[i] == '*')
                break;
            if (txt[i] == ',')
                break;
            if (txt[i] == ':')
                break;
            hdr->addr[i] = txt[i];
        }

        hdr->ssid = 0;
    }
    p = strpos(txt, '*');
    if (p > 0 && p < size)
        hdr->ssid |= 0x80;
    hdr->ssid |= 0x60;
}

char ax25_encode(ax25frame &frame, char *txt, int size)
{
    char *token, *ptr;
    int i;
    unsigned int p, p2, p3;
    char j;
    ptr = (char *)&frame;
    memset(ptr, 0, sizeof(ax25frame)); // Clear frame
    p = strpos(txt, ':');
    if (p > 0 && p < size)
    {
        // printf("p{:}=%d\r\n",p);
        // Get String APRS
        memset(&frame.data, 0, sizeof(frame.data));
        for (i = 0; i < (size - p); i++)
        {
            frame.data[i] = txt[p + i + 1];
        }
        p2 = strpos(txt, '>');
        if (p2 > 0 && p2 < size)
        {
            // printf("p2{>}=%d\r\n",p2);
            convPath(&frame.header[1], &txt[0], p2); // Get callsign src
            j = strpos(txt, ',');
            if ((j < 1) || (j > p))
                j = p;
            convPath(&frame.header[0], &txt[p2 + 1], j - p2 - 1); // Get callsign dest
                                                                  // if(j<p){
            p3 = 0;
            for (i = j; i < size; i++)
            { // copy path to origin
                if (txt[i] == ':')
                {
                    for (; i < size; i++)
                        txt[p3++] = 0x00;
                    break;
                }
                txt[p3++] = txt[i];
            }
            // printf("Path:%s\r\n",txt);
            token = strtok(txt, ",");
            j = 0;
            while (token != NULL)
            {
                ptr = token;
                convPath(&frame.header[j + 2], ptr, strlen(ptr));
                token = strtok(NULL, ",");
                j++;
                if (j > 7)
                    break;
            }

            for (i = 0; i < 10; i++)
                frame.header[i].ssid &= 0xFE; // Clear All END Path
            // Fix END path
            for (i = 2; i < 10; i++)
            {
                if (frame.header[i].addr[0] == 0x00)
                {
                    frame.header[i - 1].ssid |= 0x01;
                    break;
                }
            }
            // }
            return 1;
        }
    }
    return 0;
}

// void ax25sendFrame(AX25Ctx *ctx, ax25frame *pkg)
// {
//     int i, j, c = 0;
//     uint8_t data = 0;
//     ctx->crc_out = CRC_CCIT_INIT_VAL;
//     fifo_flush(&txFifo);
//     tx_putchar(HDLC_FLAG);

//     for (i = 0; i < 10; i++)
//         pkg->header[i].ssid &= 0xFE; // Clear All END Path
//     // Fix END path
//     for (i = 1; i < 10; i++)
//     {
//         if (pkg->header[i].addr[0] == 0x00)
//         {
//             pkg->header[i - 1].ssid |= 0x01;
//             break;
//         }
//     }

//     for (i = 0; i < 10; i++)
//     {
//         if (pkg->header[i].addr[0] == 0)
//             break;
//         for (j = 0; j < 6; j++)
//         {
//             data = (uint8_t)pkg->header[i].addr[j];
//             if (data == 0)
//                 data = 0x20;
//             // putchar(data);
//             data <<= 1;
//             ax25_putchar(ctx, data);
//             c++;
//         }
//         ax25_putchar(ctx, (uint8_t)pkg->header[i].ssid);
//         if (pkg->header[i].ssid & 0x01)
//             break;
//     }

//     ax25_putchar(ctx, AX25_CTRL_UI);      // Control field - 0x03 is APRS UI-frame
//     ax25_putchar(ctx, AX25_PID_NOLAYER3); // Protocol ID - 0xF0 is no layer 3
//                                           // ax25sendString(&pkg->data[0]);

//     for (i = 0; i < strlen(pkg->data); i++)
//     {
//         ax25_putchar(ctx, (uint8_t)pkg->data[i]);
//     }

//     uint8_t crcl = (ctx->crc_out & 0xff) ^ 0xff;
//     uint8_t crch = (ctx->crc_out >> 8) ^ 0xff;
//     tx_putchar(crcl);
//     tx_putchar(crch);

//     tx_putchar(HDLC_FLAG);
//     ax25_stateTx = true;
//     return;
// }

uint8_t *ax25_putRaw(uint8_t *raw, AX25Ctx *ctx, uint8_t c)
{
    if (c == HDLC_FLAG || c == HDLC_RESET || c == AX25_ESC)
    {
        *raw++ = AX25_ESC;
    }
    ctx->crc_out = update_crc_ccit(c, ctx->crc_out);
    *raw++ = c;
    return raw;
}

int hdlcFrame(uint8_t *outbuf, size_t outbuf_len, AX25Ctx *ctx, ax25frame *pkg)
{
    int i, j, c = 0;
    int idx = 0;
    uint8_t data = 0;
    ctx->crc_out = CRC_CCIT_INIT_VAL;
    int raw_count = 0;
    uint8_t info[300];
    info[idx++] = HDLC_FLAG;

    for (i = 0; i < 10; i++)
        pkg->header[i].ssid &= 0xFE; // Clear All END Path
    // Fix END path
    for (i = 1; i < 10; i++)
    {
        if (pkg->header[i].addr[0] == 0x00)
        {
            pkg->header[i - 1].ssid |= 0x01;
            break;
        }
    }

    for (i = 0; i < 10; i++)
    {
        if (pkg->header[i].addr[0] == 0)
            break;
        for (j = 0; j < 6; j++)
        {
            data = (uint8_t)pkg->header[i].addr[j];
            if (data == 0)
                data = 0x20;
            // putchar(data);
            data <<= 1;
            ax25_putRaw(&info[idx++], ctx, data);
            c++;
        }
        ax25_putRaw(&info[idx++], ctx, (uint8_t)pkg->header[i].ssid);
        if (pkg->header[i].ssid & 0x01)
            break;
    }

    ax25_putRaw(&info[idx++], ctx, AX25_CTRL_UI);      // Control field - 0x03 is APRS UI-frame
    ax25_putRaw(&info[idx++], ctx, AX25_PID_NOLAYER3); // Protocol ID - 0xF0 is no layer 3

    for (i = 0; i < strlen(pkg->data); i++)
    {
        ax25_putRaw(&info[idx++], ctx, (uint8_t)pkg->data[i]);
    }

    uint8_t crcl = (ctx->crc_out & 0xff) ^ 0xff;
    uint8_t crch = (ctx->crc_out >> 8) ^ 0xff;
    ax25_putRaw(&info[idx++], ctx, crcl);
    ax25_putRaw(&info[idx++], ctx, crch);
    memcpy(ctx->buf, &info[1], idx - 1);
    ctx->frame_len = idx - 1;

    info[idx++] = HDLC_FLAG;
    int len = bit_stuffing(outbuf, outbuf_len, &info[0], idx);
    return len;
}

// int hdlc2ax25(uint8_t *inbuf, size_t inbuf_len, AX25Ctx *ctx)
// {
//     uint8_t outputBuff[300];
//     int len = unbit_stuffing(outputBuff, sizeof(outputBuff), inbuf, inbuf_len);
//                     if (len > 0)
//                     {

//                     }
// }

// int hdlcDecode(AX25Ctx *ctx,uint8_t *inbuf, size_t inbuf_len)
// {
//     int i, j, c = 0;
//     int idx = 0;
//     uint8_t data = 0;
//     ctx->crc_out = CRC_CCIT_INIT_VAL;
//     int raw_count = 0;
//     uint8_t info[300];

//     int len = unbit_stuffing(outbuf, outbuf_len, &info[0], idx);
//     return len;
// }

// int hdlcDecode(uint8_t *outbuf, size_t outbuf_len, AX25Ctx *ctx, ax25frame *pkg)
// {
//     int i, j, c = 0;
//     int idx = 0;
//     uint8_t data = 0;
//     ctx->crc_out = CRC_CCIT_INIT_VAL;
//     int raw_count = 0;
//     uint8_t info[300];

//     int len = unbit_stuffing(outbuf, outbuf_len, &info[0], idx);
//     return len;

//     for (i = 0; i < 10; i++)
//         pkg->header[i].ssid &= 0xFE; // Clear All END Path
//     // Fix END path
//     for (i = 1; i < 10; i++)
//     {
//         if (pkg->header[i].addr[0] == 0x00)
//         {
//             pkg->header[i - 1].ssid |= 0x01;
//             break;
//         }
//     }

//     for (i = 0; i < 10; i++)
//     {
//         if (pkg->header[i].addr[0] == 0)
//             break;
//         for (j = 0; j < 6; j++)
//         {
//             data = (uint8_t)pkg->header[i].addr[j];
//             if (data == 0)
//                 data = 0x20;
//             // putchar(data);
//             data <<= 1;
//             ax25_putRaw(&info[idx++], ctx, data);
//             c++;
//         }
//         ax25_putRaw(&info[idx++], ctx, (uint8_t)pkg->header[i].ssid);
//         if (pkg->header[i].ssid & 0x01)
//             break;
//     }

//     ax25_putRaw(&info[idx++], ctx, AX25_CTRL_UI);      // Control field - 0x03 is APRS UI-frame
//     ax25_putRaw(&info[idx++], ctx, AX25_PID_NOLAYER3); // Protocol ID - 0xF0 is no layer 3

//     for (i = 0; i < strlen(pkg->data); i++)
//     {
//         ax25_putRaw(&info[idx++], ctx, (uint8_t)pkg->data[i]);
//     }

//     uint8_t crcl = (ctx->crc_out & 0xff) ^ 0xff;
//     uint8_t crch = (ctx->crc_out >> 8) ^ 0xff;
//     ax25_putRaw(&info[idx++], ctx, crcl);
//     ax25_putRaw(&info[idx++], ctx, crch);

//     info[idx++] = HDLC_FLAG;
//     int len = bit_stuffing(outbuf, outbuf_len, &info[0], idx);
//     return len;
// }

// void hdlc_rec_bit (int chan, int subchan, int slice, int raw, int is_scrambled, int not_used_remove)
// {

// 	int dbit;			/* Data bit after undoing NRZI. */
// 					/* Should be only 0 or 1. */
// 	struct hdlc_state_s *H;

// 	assert (was_init == 1);

// 	assert (chan >= 0 && chan < MAX_CHANS);
// 	assert (subchan >= 0 && subchan < MAX_SUBCHANS);

// 	assert (slice >= 0 && slice < MAX_SLICERS);

// // -e option can be used to artificially introduce the desired
// // Bit Error Rate (BER) for testing.

// 	if (g_audio_p->recv_ber != 0) {
// 	  double r = (double)my_rand() / (double)MY_RAND_MAX;  // calculate as double to preserve all 31 bits.
// 	  if (g_audio_p->recv_ber > r) {

// // FIXME
// //text_color_set(DW_COLOR_DEBUG);
// //dw_printf ("hdlc_rec_bit randomly clobber bit, ber = %.6f\n", g_audio_p->recv_ber);

// 	    raw = ! raw;
// 	  }
// 	}

// // EAS does not use HDLC.

// 	if (g_audio_p->achan[chan].modem_type == MODEM_EAS) {
// 	  eas_rec_bit (chan, subchan, slice, raw, not_used_remove);
// 	  return;
// 	}

// /*
//  * Different state information for each channel / subchannel / slice.
//  */
// 	H = &hdlc_state[chan][subchan][slice];

// /*
//  * Using NRZI encoding,
//  *   A '0' bit is represented by an inversion since previous bit.
//  *   A '1' bit is represented by no change.
//  */

// 	if (is_scrambled) {
// 	  int descram;

// 	  descram = descramble(raw, &(H->lfsr));

// 	  dbit = (descram == H->prev_descram);
// 	  H->prev_descram = descram;
// 	  H->prev_raw = raw;
// 	}
// 	else {

// 	  dbit = (raw == H->prev_raw);

// 	  H->prev_raw = raw;
// 	}

// // After BER insertion, NRZI, and any descrambling, feed into FX.25 decoder as well.
// // Don't waste time on this if AIS.  EAS does not get this far.

// 	if (g_audio_p->achan[chan].modem_type != MODEM_AIS) {
// 	  fx25_rec_bit (chan, subchan, slice, dbit);
// 	  il2p_rec_bit (chan, subchan, slice, raw);	// Note: skip NRZI.
// 	}

// /*
//  * Octets are sent LSB first.
//  * Shift the most recent 8 bits thru the pattern detector.
//  */
// 	H->pat_det >>= 1;
// 	if (dbit) {
// 	  H->pat_det |= 0x80;
// 	}

// 	H->flag4_det >>= 1;
// 	if (dbit) {
// 	  H->flag4_det |= 0x80000000;
// 	}

// 	rrbb_append_bit (H->rrbb, raw);

// 	if (H->pat_det == 0x7e) {

// 	  rrbb_chop8 (H->rrbb);

// /*
//  * The special pattern 01111110 indicates beginning and ending of a frame.
//  * If we have an adequate number of whole octets, it is a candidate for
//  * further processing.
//  *
//  * It might look odd that olen is being tested for 7 instead of 0.
//  * This is because oacc would already have 7 bits from the special
//  * "flag" pattern before it is detected here.
//  */

// #if OLD_WAY

// #if TEST
// 	  text_color_set(DW_COLOR_DEBUG);
// 	  dw_printf ("\nfound flag, olen = %d, frame_len = %d\n", olen, frame_len);
// #endif
// 	  if (H->olen == 7 && H->frame_len >= MIN_FRAME_LEN) {

// 	    unsigned short actual_fcs, expected_fcs;

// #if TEST
// 	    int j;
// 	    dw_printf ("TRADITIONAL: frame len = %d\n", H->frame_len);
// 	    for (j=0; j<H->frame_len; j++) {
// 	      dw_printf ("  %02x", H->frame_buf[j]);
// 	    }
// 	    dw_printf ("\n");

// #endif
// 	    /* Check FCS, low byte first, and process... */

// 	    /* Alternatively, it is possible to include the two FCS bytes */
// 	    /* in the CRC calculation and look for a magic constant.  */
// 	    /* That would be easier in the case where the CRC is being */
// 	    /* accumulated along the way as the octets are received. */
// 	    /* I think making a second pass over it and comparing is */
// 	    /* easier to understand. */

// 	    actual_fcs = H->frame_buf[H->frame_len-2] | (H->frame_buf[H->frame_len-1] << 8);

// 	    expected_fcs = fcs_calc (H->frame_buf, H->frame_len - 2);

// 	    if (actual_fcs == expected_fcs) {
// 	      alevel_t alevel = demod_get_audio_level (chan, subchan);

// 	      multi_modem_process_rec_frame (chan, subchan, slice, H->frame_buf, H->frame_len - 2, alevel, RETRY_NONE, 0);   /* len-2 to remove FCS. */
// 	    }
// 	    else {

// #if TEST
// 	      dw_printf ("*** actual fcs = %04x, expected fcs = %04x ***\n", actual_fcs, expected_fcs);
// #endif

// 	    }

// 	  }

// #else

// /*
//  * New way - Decode the raw bits in later step.
//  */

// #if TEST
// 	  text_color_set(DW_COLOR_DEBUG);
// 	  dw_printf ("\nfound flag, channel %d.%d, %d bits in frame\n", chan, subchan, rrbb_get_len(H->rrbb) - 1);
// #endif
// 	  if (rrbb_get_len(H->rrbb) >= MIN_FRAME_LEN * 8) {

// 	    alevel_t alevel = demod_get_audio_level (chan, subchan);

// 	    rrbb_set_audio_level (H->rrbb, alevel);
// 	    hdlc_rec2_block (H->rrbb);
// 	    	/* Now owned by someone else who will free it. */

// 	    H->rrbb = rrbb_new (chan, subchan, slice, is_scrambled, H->lfsr, H->prev_descram); /* Allocate a new one. */
// 	  }
// 	  else {
// 	    rrbb_clear (H->rrbb, is_scrambled, H->lfsr, H->prev_descram);
// 	  }

// 	  H->olen = 0;		/* Allow accumulation of octets. */
// 	  H->frame_len = 0;

// 	  rrbb_append_bit (H->rrbb, H->prev_raw); /* Last bit of flag.  Needed to get first data bit. */
// 						/* Now that we are saving other initial state information, */
// 						/* it would be sensible to do the same for this instead */
// 						/* of lumping it in with the frame data bits. */
// #endif

// 	}

// //#define EXPERIMENT12B 1

// #if EXPERIMENT12B

// 	else if (H->pat_det == 0xff) {

// /*
//  * Valid data will never have seven 1 bits in a row.
//  *
//  *	11111110
//  *
//  * This indicates loss of signal.
//  * But we will let it slip thru because it might diminish
//  * our single bit fixup effort.   Instead give up on frame
//  * only when we see eight 1 bits in a row.
//  *
//  *	11111111
//  *
//  * What is the impact?  No difference.
//  *
//  *  Before:	atest -P E -F 1 ../02_Track_2.wav	= 1003
//  *  After:	atest -P E -F 1 ../02_Track_2.wav	= 1003
//  */

// #else
// 	else if (H->pat_det == 0xfe) {

// /*
//  * Valid data will never have 7 one bits in a row.
//  *
//  *	11111110
//  *
//  * This indicates loss of signal.
//  */

// #endif

// 	  H->olen = -1;		/* Stop accumulating octets. */
// 	  H->frame_len = 0;	/* Discard anything in progress. */

// 	  rrbb_clear (H->rrbb, is_scrambled, H->lfsr, H->prev_descram);

// 	}
// 	else if ( (H->pat_det & 0xfc) == 0x7c ) {

// /*
//  * If we have five '1' bits in a row, followed by a '0' bit,
//  *
//  *	0111110xx
//  *
//  * the current '0' bit should be discarded because it was added for
//  * "bit stuffing."
//  */
// 	  ;

// 	} else {

// /*
//  * In all other cases, accumulate bits into octets, and complete octets
//  * into the frame buffer.
//  */
// 	  if (H->olen >= 0) {

// 	    H->oacc >>= 1;
// 	    if (dbit) {
// 	      H->oacc |= 0x80;
// 	    }
// 	    H->olen++;

// 	    if (H->olen == 8) {
// 	      H->olen = 0;

// 	      if (H->frame_len < MAX_FRAME_LEN) {
// 		H->frame_buf[H->frame_len] = H->oacc;
// 		H->frame_len++;
// 	      }
// 	    }
// 	  }
// 	}
// }

int hdlcDecode(uint8_t *frame_buf, size_t &frame_len, uint8_t *raw, size_t len)
{
    Hdlc hdlcFlag;
    uint8_t bit = 0;
    size_t idx = 0;
    size_t i = 0;
    uint16_t crc_in = CRC_CCIT_INIT_VAL;

    hdlcFlag.bitIndex = 0;
    hdlcFlag.currentByte = 0;
    hdlcFlag.demodulatedBits = 0;
    hdlcFlag.receiving = false;
    for (i = 0; i < len; i++)
    {
        uint8_t input_byte = raw[i];
        uint8_t output_byte = 0;
        for (int j = 0; j < 8; j++)
        {
            bool current_bit = (input_byte >> j) & 1;
            if (hdlcParseBit(&hdlcFlag, current_bit, frame_buf, idx, false))
            {
                if (idx > 0)
                {
                    // log_d("RAW[%d]:%0X FCS=%02X = %02X",idx-1,frame_buf[idx-1],actual_fcs,expected_fcs);
                    crc_in = update_crc_ccit(frame_buf[idx - 1], crc_in);
                    if (crc_in == 0xF0B8)
                    {
                        uint16_t actual_fcs = frame_buf[idx - 2] | (frame_buf[idx - 1] << 8);
                        uint16_t expected_fcs = fcs_calc(frame_buf, idx - 2);
                        if(expected_fcs == actual_fcs){
                            frame_len = idx;
                            return i;
                        }
                    }
                }
            }
        }
    }
    frame_len = 0;
    return i;
}

int hdlcDecodeAX25(uint8_t *frame_buf, size_t &frame_len, uint8_t *raw, size_t len)
{
    Hdlc hdlcFlag;
    uint8_t bit = 0;
    size_t idx = 0;
    size_t i = 0;
    uint16_t crc_in = CRC_CCIT_INIT_VAL;

    hdlcFlag.bitIndex = 0;
    hdlcFlag.currentByte = 0;
    hdlcFlag.demodulatedBits = 0;
    hdlcFlag.receiving = false;
    for (i = 0; i < len; i++)
    {
        uint8_t input_byte = raw[i];
        uint8_t output_byte = 0;
        for (int j = 0; j < 8; j++)
        {
            bool current_bit = (input_byte >> j) & 1;
            if (hdlcParseBit(&hdlcFlag, current_bit, frame_buf, idx, true))
            {
                if (idx > 0)
                {
                    // uint16_t actual_fcs = frame_buf[idx - 2] | (frame_buf[idx - 1] << 8);
                    // uint16_t expected_fcs = fcs_calc(frame_buf, idx - 2);
                    //  log_d("RAW[%d]:%0X FCS=%02X = %02X",idx-1,frame_buf[idx-1],actual_fcs,expected_fcs);
                    crc_in = update_crc_ccit(frame_buf[idx - 1], crc_in);
                    if (crc_in == 0xF0B8)
                    {
                        uint16_t actual_fcs = frame_buf[idx - 2] | (frame_buf[idx - 1] << 8);
                        uint16_t expected_fcs = fcs_calc(frame_buf, idx - 2);
                        if(expected_fcs == actual_fcs){
                            frame_len = idx;
                            return i;
                        }
                    }
                }
            }
        }
    }
    frame_len = 0;
    return i;
}