/* Last-state peek. 64 B header + N x 64 B frames. Magic SOMA in the header only.
   make && ./peek [path.soma.bin]
   Default: generate a 64+64 fixture in memory (body 6 ocean). No grokd required. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "../../include/ztp.h"

static int peek_bytes(const uint8_t *buf, uint64_t len) {
    if (!ztp_last_state_header_ok(buf, len)) {
        printf("  header_ok=0  (need magic SOMA, 64 B header + frames)\n");
        return 1;
    }
    ZtpLastStateFrame last;
    memset(&last, 0, sizeof(last));
    if (!ztp_last_state_peek_last(buf, len, &last)) {
        printf("  peek_last=0\n");
        return 1;
    }
    uint16_t bid = ztp_last_state_body_id(buf);
    char magic[5];
    memcpy(magic, buf, 4);
    magic[4] = 0;
    printf("  magic=%s  body_id=%u  t=%.3f  pos0=%.3f  (named slot 0)\n",
           magic, (unsigned)bid, last.t, last.pos[0]);
    printf("  header %zu B  frame %zu B  file %llu B\n",
           sizeof(ZtpLastStateHeader), sizeof(ZtpLastStateFrame),
           (unsigned long long)len);
    return 0;
}

static int peek_path(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        printf("  cannot open %s\n", path);
        return 1;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return 1;
    }
    long n = ftell(f);
    if (n < 128 || n > (1 << 20)) {
        fclose(f);
        printf("  %s size %ld not a 64+64 peek (cap 1 MiB)\n", path, n);
        return 1;
    }
    rewind(f);
    uint8_t *buf = (uint8_t *)malloc((size_t)n);
    if (!buf) {
        fclose(f);
        return 1;
    }
    size_t nread = fread(buf, 1, (size_t)n, f);
    fclose(f);
    int rc = (nread == (size_t)n) ? peek_bytes(buf, (uint64_t)nread) : 1;
    free(buf);
    return rc;
}

static int peek_generated(void) {
    ZtpLastStateFrame fr = ztp_last_state_pack_ocean(
        1000, 100.0f, 1.006f, 12.0f, 600.0f, 600.0f, 4.0f, 11.0f, 100.0f,
        false, false);
    uint8_t out[128];
    memset(out, 0, sizeof(out));
    const char reserved[8] = "OCEAN001";
    uint64_t n = ztp_last_state_write(6, reserved, &fr, 1, out, sizeof(out));
    if (n != 128) {
        printf("  write fixture failed n=%llu\n", (unsigned long long)n);
        return 1;
    }
    printf("  fixture 64+64 in memory  body 6  (pass a .soma.bin to peek a file)\n");
    return peek_bytes(out, n);
}

int main(int argc, char **argv) {
    _Static_assert(sizeof(ZtpLastStateHeader) == 64, "ZtpLastStateHeader is 64 B");
    _Static_assert(sizeof(ZtpLastStateFrame) == 64, "ZtpLastStateFrame is 64 B");
    printf("ztp-runtime eval  ·  peek  ·  last-state  ·  64 B when the radio is dead\n");
    if (argc > 1) {
        printf("  file %s\n", argv[1]);
        return peek_path(argv[1]);
    }
    return peek_generated();
}
