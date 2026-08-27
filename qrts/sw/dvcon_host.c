/* =============================================================================
 * dvcon_host.c — host side of the DE2-115 accelerator
 *
 * Bulk data goes over raw Ethernet; control goes over JTAG. This program owns
 * the Ethernet half:
 *
 *     load    send a model blob or an image into SDRAM
 *     ident   check the link by round-tripping an ACK_REQ
 *
 * The JTAG half is a quartus_stp script (tools/dvcon_jtag.tcl), because the
 * USB-Blaster is reached through Quartus's own TAP driver rather than a socket.
 *
 * ---------------------------------------------------------------------------
 * WHY RAW L2 AND NOT UDP
 * ---------------------------------------------------------------------------
 * There is no IP stack on the FPGA, and adding one would be a large amount of
 * hardware for no benefit on a directly attached cable. Raw L2 with a private
 * ethertype means the FPGA parses one fixed 16-byte header and nothing else.
 *
 * The cost is that this program needs CAP_NET_RAW (or root) and a specific
 * interface -- it cannot be routed. That is the correct trade for a board on
 * the end of a cable.
 *
 * ---------------------------------------------------------------------------
 * RELIABILITY
 * ---------------------------------------------------------------------------
 * Raw L2 has no retransmission, and a 2.8 MB blob is ~2000 frames. A model
 * with a hole in it does not fail loudly -- it produces plausible, wrong
 * detections. So every 64 frames the host asks for the received bitmap and
 * resends whatever is missing, repeating until the window is complete.
 *
 * Build:  cc -O2 -Wall -o dvcon_host dvcon_host.c
 * Run:    sudo ./dvcon_host eth0 load 0x00000000 yolo26n.bin
 * ============================================================================= */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <net/if.h>
#include <netinet/in.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <poll.h>

#include "dvcon_memmap.h"

/* The board's MAC, matching dvcon_top's MAC_ADDR parameter. Locally
 * administered, so it cannot collide with a real vendor assignment. */
static const uint8_t FPGA_MAC[6] = {0x02, 0x00, 0x00, 0xC0, 0xFF, 0xEE};

struct link {
    int      fd;
    int      ifindex;
    uint8_t  src_mac[6];
};

static void die(const char *what)
{
    fprintf(stderr, "%s: %s\n", what, strerror(errno));
    exit(1);
}

static int link_open(struct link *lk, const char *ifname)
{
    struct ifreq ifr;

    lk->fd = socket(AF_PACKET, SOCK_RAW, htons(DVCON_ETHERTYPE));
    if (lk->fd < 0) {
        if (errno == EPERM)
            fprintf(stderr, "need CAP_NET_RAW: run with sudo\n");
        die("socket(AF_PACKET)");
    }

    memset(&ifr, 0, sizeof ifr);
    snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname);
    if (ioctl(lk->fd, SIOCGIFINDEX, &ifr) < 0) die("SIOCGIFINDEX");
    lk->ifindex = ifr.ifr_ifindex;

    if (ioctl(lk->fd, SIOCGIFHWADDR, &ifr) < 0) die("SIOCGIFHWADDR");
    memcpy(lk->src_mac, ifr.ifr_hwaddr.sa_data, 6);

    return 0;
}

/* One command frame. `payload` may be NULL for a command with no body. */
static int send_cmd(struct link *lk, uint8_t opcode, uint32_t seq,
                    uint32_t addr, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[14 + DVCON_HDR_BYTES + DVCON_MAX_PAYLOAD];
    struct sockaddr_ll sa;
    size_t n = 0;

    if (len > DVCON_MAX_PAYLOAD) {
        fprintf(stderr, "payload %u exceeds %u\n", len, DVCON_MAX_PAYLOAD);
        return -1;
    }

    memcpy(frame + n, FPGA_MAC, 6);      n += 6;
    memcpy(frame + n, lk->src_mac, 6);   n += 6;
    frame[n++] = (DVCON_ETHERTYPE >> 8) & 0xFF;
    frame[n++] =  DVCON_ETHERTYPE       & 0xFF;

    /* Header, big endian -- the FPGA reads it a byte at a time. */
    frame[n++] = 1;            /* ver    */
    frame[n++] = opcode;
    frame[n++] = 0;            /* flags  */
    frame[n++] = 0;            /* rsv    */
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
    frame[n++] = 0;            /* rsv2 */
    frame[n++] = 0;

    if (payload && len) { memcpy(frame + n, payload, len); n += len; }

    /* Pad to the 60-byte minimum; the NIC appends the FCS. A short frame is a
     * runt and switches drop it silently. */
    while (n < 60) frame[n++] = 0;

    memset(&sa, 0, sizeof sa);
    sa.sll_family   = AF_PACKET;
    sa.sll_ifindex  = lk->ifindex;
    sa.sll_halen    = 6;
    memcpy(sa.sll_addr, FPGA_MAC, 6);

    if (sendto(lk->fd, frame, n, 0, (struct sockaddr *)&sa, sizeof sa) < 0)
        die("sendto");
    return 0;
}

/* Ask for the window bitmap. Returns 0 and fills *bitmap, or -1 on timeout. */
static int fetch_bitmap(struct link *lk, uint32_t seq, uint64_t *bitmap,
                        int timeout_ms)
{
    uint8_t buf[2048];
    struct pollfd pfd = { .fd = lk->fd, .events = POLLIN };

    if (send_cmd(lk, DVCON_OP_ACK_REQ, seq, 0, NULL, 0) < 0) return -1;

    for (;;) {
        int r = poll(&pfd, 1, timeout_ms);
        if (r <= 0) return -1;                 /* timeout or error */

        ssize_t got = recv(lk->fd, buf, sizeof buf, 0);
        if (got < 14 + 8) continue;

        /* Only frames from the board, carrying our ethertype. */
        if (memcmp(buf + 6, FPGA_MAC, 6) != 0) continue;

        uint64_t bm = 0;
        for (int i = 0; i < 8; i++) bm = (bm << 8) | buf[14 + i];
        *bitmap = bm;
        return 0;
    }
}

/* Send a buffer to SDRAM, retransmitting whatever the board did not receive. */
static int load_region(struct link *lk, uint32_t base,
                       const uint8_t *data, size_t size)
{
    const size_t chunk = DVCON_MAX_PAYLOAD;
    size_t nframes = (size + chunk - 1) / chunk;
    size_t sent = 0;
    uint32_t seq = 0;

    printf("  %zu bytes -> 0x%08X  (%zu frames)\n", size, base, nframes);

    while (sent < nframes) {
        size_t win = nframes - sent;
        if (win > DVCON_ACK_WINDOW) win = DVCON_ACK_WINDOW;

        uint64_t want = (win == 64) ? ~0ULL : ((1ULL << win) - 1);
        /* Which frames of this window the board already holds. Carried ACROSS
         * attempts: on a retry only the missing ones are resent, so a single
         * lost frame costs one frame of bandwidth rather than sixty-four. */
        uint64_t have = 0;

        for (int attempt = 0; attempt < 8; attempt++) {
            for (size_t i = 0; i < win; i++) {
                if (have & (1ULL << i)) continue;
                size_t off = (sent + i) * chunk;
                size_t len = (off + chunk <= size) ? chunk : size - off;
                send_cmd(lk, DVCON_OP_WRITE_MEM, seq + (uint32_t)i,
                         base + (uint32_t)off, data + off, (uint16_t)len);
            }

            uint64_t got = 0;
            if (fetch_bitmap(lk, seq, &got, 500) < 0) {
                fprintf(stderr, "  no ACK from the board (attempt %d)\n",
                        attempt + 1);
                continue;
            }
            have |= got;
            if ((have & want) == want) goto window_done;

            fprintf(stderr, "  window at frame %zu: %d missing, resending\n",
                    sent, __builtin_popcountll(want & ~have));
        }

        fprintf(stderr, "  giving up on the window at frame %zu\n", sent);
        return -1;

    window_done:
        sent += win;
        seq  += DVCON_ACK_WINDOW;   /* next window: seq[31:6] advances */
        printf("\r  %zu/%zu frames", sent, nframes);
        fflush(stdout);
    }

    printf("\r  %zu/%zu frames -- complete\n", nframes, nframes);
    return 0;
}

static uint8_t *read_file(const char *path, size_t *size_out)
{
    struct stat st;
    int fd = open(path, O_RDONLY);
    if (fd < 0) die(path);
    if (fstat(fd, &st) < 0) die("fstat");

    uint8_t *buf = malloc((size_t)st.st_size);
    if (!buf) die("malloc");

    size_t done = 0;
    while (done < (size_t)st.st_size) {
        ssize_t r = read(fd, buf + done, (size_t)st.st_size - done);
        if (r <= 0) die("read");
        done += (size_t)r;
    }
    close(fd);

    *size_out = done;
    return buf;
}

static void usage(void)
{
    fprintf(stderr,
        "usage:\n"
        "  dvcon_host <iface> load  <addr> <file>   send a file into SDRAM\n"
        "  dvcon_host <iface> model <file>          load at MODEL_BASE\n"
        "  dvcon_host <iface> frame <file>          load at FRAME_BASE\n"
        "  dvcon_host <iface> ident                 check the link\n"
        "\n"
        "Control (START, status, boxes) is over JTAG -- see dvcon_jtag.tcl.\n");
    exit(2);
}

int main(int argc, char **argv)
{
    struct link lk;

    if (argc < 3) usage();
    link_open(&lk, argv[1]);

    const char *cmd = argv[2];

    if (!strcmp(cmd, "ident")) {
        uint64_t bm;
        if (fetch_bitmap(&lk, 0, &bm, 1000) < 0) {
            fprintf(stderr, "no reply -- check the cable, the PHY link LED, "
                            "and that the bitstream is loaded\n");
            return 1;
        }
        printf("board responded, window bitmap = 0x%016llX\n",
               (unsigned long long)bm);
        return 0;
    }

    if (!strcmp(cmd, "load") && argc == 5) {
        uint32_t addr = (uint32_t)strtoul(argv[3], NULL, 0);
        size_t size;
        uint8_t *buf = read_file(argv[4], &size);
        int rc = load_region(&lk, addr, buf, size);
        free(buf);
        return rc ? 1 : 0;
    }

    if (!strcmp(cmd, "model") && argc == 4) {
        size_t size;
        uint8_t *buf = read_file(argv[3], &size);
        if (size > DVCON_MODEL_SIZE) {
            fprintf(stderr, "model is %zu bytes, region holds %u\n",
                    size, DVCON_MODEL_SIZE);
            return 1;
        }
        int rc = load_region(&lk, DVCON_MODEL_BASE, buf, size);
        free(buf);
        return rc ? 1 : 0;
    }

    if (!strcmp(cmd, "frame") && argc == 4) {
        size_t size;
        uint8_t *buf = read_file(argv[3], &size);
        if (size > DVCON_FRAME_SIZE) {
            fprintf(stderr, "frame is %zu bytes, region holds %u\n",
                    size, DVCON_FRAME_SIZE);
            return 1;
        }
        int rc = load_region(&lk, DVCON_FRAME_BASE, buf, size);
        free(buf);
        return rc ? 1 : 0;
    }

    usage();
}
