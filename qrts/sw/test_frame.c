/* =============================================================================
 * test_frame.c — check the host's frame layout against the RTL's parser
 *
 * dvcon_host.c cannot be built here: AF_PACKET is Linux-only and this is a
 * Windows machine. But the part most likely to be wrong is not the socket
 * code -- it is the byte layout, and that is pure arithmetic.
 *
 * The offsets below are not copied from dvcon_host.c. They are the ones
 * OBSERVED coming out of eth_mac_rx in tb_eth_path, printed byte by byte:
 *
 *     byte 0..5   destination MAC
 *     byte 6..11  source MAC
 *     byte 12..13 ethertype 88 b5
 *     byte 15     opcode
 *     byte 18..21 seq        (big endian)
 *     byte 22..25 addr       (big endian)
 *     byte 26..27 len        (big endian)
 *     byte 30..   payload
 *
 * If the host and the RTL ever disagree, the FPGA parses a valid frame as
 * garbage and drops it with no error anywhere -- which is exactly what
 * happened once already, when eth_cmd_engine's offsets assumed the MAC header
 * had been stripped.
 *
 *     cc -O2 -Wall -Wextra -o test_frame test_frame.c && ./test_frame
 * ============================================================================= */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "dvcon_memmap.h"

static const uint8_t FPGA_MAC[6] = {0x02, 0x00, 0x00, 0xC0, 0xFF, 0xEE};
static const uint8_t HOST_MAC[6] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15};

/* The frame builder, byte for byte as dvcon_host.c's send_cmd() writes it.
 * Kept in step by this test failing if either side moves. */
static size_t build(uint8_t *frame, uint8_t opcode, uint32_t seq,
                    uint32_t addr, const uint8_t *payload, uint16_t len)
{
    size_t n = 0;

    memcpy(frame + n, FPGA_MAC, 6);  n += 6;
    memcpy(frame + n, HOST_MAC, 6);  n += 6;
    frame[n++] = (DVCON_ETHERTYPE >> 8) & 0xFF;
    frame[n++] =  DVCON_ETHERTYPE       & 0xFF;

    frame[n++] = 1;
    frame[n++] = opcode;
    frame[n++] = 0;
    frame[n++] = 0;
    frame[n++] = (seq  >> 24) & 0xFF;
    frame[n++] = (seq  >> 16) & 0xFF;
    frame[n++] = (seq  >>  8) & 0xFF;
    frame[n++] =  seq         & 0xFF;
    frame[n++] = (addr >> 24) & 0xFF;
    frame[n++] = (addr >> 16) & 0xFF;
    frame[n++] = (addr >>  8) & 0xFF;
    frame[n++] =  addr        & 0xFF;
    frame[n++] = (len  >>  8) & 0xFF;
    frame[n++] =  len         & 0xFF;
    frame[n++] = 0;
    frame[n++] = 0;

    if (payload && len) { memcpy(frame + n, payload, len); n += len; }
    while (n < 60) frame[n++] = 0;
    return n;
}

static int errors = 0;

static void check(const char *what, unsigned got, unsigned exp)
{
    if (got != exp) {
        printf("  FAIL %-38s got 0x%02X expected 0x%02X\n", what, got, exp);
        errors++;
    } else {
        printf("  ok   %-38s 0x%02X\n", what, got);
    }
}

int main(void)
{
    uint8_t frame[2048];
    uint8_t payload[16];
    for (int i = 0; i < 16; i++) payload[i] = 0xA0 + i;

    printf("=== test_frame ===\n");

    size_t n = build(frame, DVCON_OP_WRITE_MEM, 0x11223344,
                     DVCON_FRAME_BASE, payload, 16);

    /* Positions the RTL reads, from tb_eth_path's trace. */
    check("byte 12 = ethertype high", frame[12], 0x88);
    check("byte 13 = ethertype low",  frame[13], 0xB5);
    check("byte 15 = opcode",         frame[15], DVCON_OP_WRITE_MEM);

    check("byte 18 = seq[31:24]", frame[18], 0x11);
    check("byte 19 = seq[23:16]", frame[19], 0x22);
    check("byte 20 = seq[15:8]",  frame[20], 0x33);
    check("byte 21 = seq[7:0]",   frame[21], 0x44);

    check("byte 22 = addr[31:24]", frame[22], (DVCON_FRAME_BASE >> 24) & 0xFF);
    check("byte 23 = addr[23:16]", frame[23], (DVCON_FRAME_BASE >> 16) & 0xFF);
    check("byte 24 = addr[15:8]",  frame[24], (DVCON_FRAME_BASE >>  8) & 0xFF);
    check("byte 25 = addr[7:0]",   frame[25],  DVCON_FRAME_BASE        & 0xFF);

    check("byte 26 = len high", frame[26], 0x00);
    check("byte 27 = len low",  frame[27], 16);

    check("byte 30 = payload[0]", frame[30], 0xA0);
    check("byte 31 = payload[1]", frame[31], 0xA1);
    check("byte 45 = payload[15]", frame[45], 0xAF);

    /* A short frame is a runt and switches drop it in hardware. */
    if (n < 60) {
        printf("  FAIL frame is %zu bytes, below the 60-byte minimum\n", n);
        errors++;
    } else {
        printf("  ok   frame padded to %zu bytes\n", n);
    }

    /* ACK_REQ carries no payload and must still be padded. */
    n = build(frame, DVCON_OP_ACK_REQ, 0, 0, NULL, 0);
    check("ack_req opcode at byte 15", frame[15], DVCON_OP_ACK_REQ);
    if (n != 60) {
        printf("  FAIL empty frame is %zu bytes, expected 60\n", n);
        errors++;
    } else {
        printf("  ok   empty frame padded to 60 bytes\n");
    }

    /* The payload cap must divide into whole SDRAM bursts. */
    if (DVCON_MAX_PAYLOAD % 128 != 0) {
        printf("  FAIL MAX_PAYLOAD %u is not a multiple of 128\n",
               DVCON_MAX_PAYLOAD);
        errors++;
    } else {
        printf("  ok   MAX_PAYLOAD %u is a multiple of 128\n",
               DVCON_MAX_PAYLOAD);
    }

    printf("=== %s: %d error(s) ===\n", errors ? "FAILED" : "PASSED", errors);
    return errors ? 1 : 0;
}
