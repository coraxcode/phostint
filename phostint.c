/*
 * PhosTint - safe X11/i3wm display tint and brightness controller
 *
 * Single-file C11 program. Uses only Xlib + XRandR for color control.
 * Optional hardware backlight control uses /sys/class/backlight only when the
 * current user already has write permission. It never invokes sudo, chmod,
 * DDC/CI, raw DRM/KMS, /dev/mem, or GPU registers.
 *
 * Safety design:
 *   - Captures the existing per-CRTC gamma ramps before modifying them.
 *   - Persists a private recovery copy (gamma ramps and original backlight
 *     values) in XDG_RUNTIME_DIR, or in a private 0700 directory under /tmp.
 *   - "normal" restores the exact captured ramps and any backlight values
 *     changed by this process, and reports honestly when a CRTC or a
 *     backlight device could not be restored.
 *   - SIGINT/SIGTERM/SIGHUP/SIGQUIT trigger restoration before exit.
 *   - If the X server dies, changed backlight values are still restored.
 *   - "emergency-reset" first tries a live daemon, then the recovery journal.
 *     If neither is usable it stops and explains: forcing a neutral ramp on
 *     every connected monitor would also discard an ICC calibration loaded by
 *     another tool, so it requires an explicit "--identity".
 *   - A per-display file lock guarantees a single daemon instance.
 *   - RandR events are monitored: monitor hotplug and mode changes re-sync
 *     the captured state and re-apply the current look automatically.
 *     Saved ramps are keyed by connector name, not only by CRTC id, so a
 *     monitor that moves between CRTCs keeps its own pristine capture.
 *   - The recovery file is parsed and validated completely before a single
 *     byte of it is applied, and is only trusted when it is a regular file
 *     owned by the current user (never followed through a symlink). It is
 *     deleted only after a fully successful restoration.
 *   - Only backlight devices this process actually changed are recorded for
 *     recovery, and only one device is driven per command.
 *   - Software brightness never reaches 0% (range 1..100) to reduce the risk
 *     of accidentally making the screen unusable.
 *
 * Important limitation:
 *   XRandR gamma ramps can scale each RGB channel independently but cannot mix
 *   channels. Therefore custom colors are tints, not a true monochrome color
 *   matrix. "temp" uses a common blackbody approximation for the white point.
 *   Physical spectral output also varies by panel/backlight: a digital blue
 *   value of 0% does not guarantee zero physical blue-wavelength emission.
 *
 * Build:
 *   ./build.sh    (or)
 *   cc -std=c11 -O2 -Wall -Wextra -Wpedantic phostint.c \
 *      $(pkg-config --cflags --libs x11 xrandr xext) -lm -o phostint
 */

#define _POSIX_C_SOURCE 200809L

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>          /* XVisualInfo, XMatchVisualInfo, XGetVisualInfo */
#include <X11/extensions/Xrandr.h>
#include <X11/extensions/shape.h>
#ifdef HAVE_XRENDER
#include <X11/extensions/Xrender.h>   /* optional: authoritative alpha check */
#endif

#include <ctype.h>
#include <langinfo.h>
#include <locale.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

#ifndef NAME_MAX
#define NAME_MAX 255
#endif

#if defined(__GNUC__) || defined(__clang__)
#define PT_PRINTF(fmt_idx, arg_idx) __attribute__((format(printf, fmt_idx, arg_idx)))
#else
#define PT_PRINTF(fmt_idx, arg_idx)
#endif

#define APP_NAME "PhosTint"
#define APP_VERSION "1.5.1"
#define MAX_SAVED_CRTCS 64
#define MAX_BACKLIGHTS 32
/*
 * The journal must be able to hold everything the program can hold at once:
 * every managed output plus every ramp still owed to a disconnected monitor.
 * Sizing it to the sum is what makes "never drop recovery data" achievable
 * instead of merely aspirational.
 */
#define MAX_JOURNAL_RAMPS (MAX_SAVED_CRTCS * 2)
#define MAX_JOURNAL_BACKLIGHTS (MAX_BACKLIGHTS * 2)
#define MAX_COMMAND 512
#define MAX_RESPONSE 2048
#define MAX_CLIENTS 8
#define CLIENT_TIMEOUT_MS 5000U
#define STATE_MAGIC "PHOST04"
#define STATE_VERSION 4U

/*
 * Noise overlay geometry and cadence. Every constant is drawn from the
 * golden ratio: 377x233 is a Fibonacci golden rectangle (377/233 = 1.618),
 * 13 animation frames and an 89 ms frame period are Fibonacci numbers, and
 * GOLDEN_HASH_K = floor(2^32 / phi) is Knuth's multiplicative hash constant.
 */
#define NOISE_TILE_W 377
#define NOISE_TILE_H 233
#define NOISE_FRAMES 13
#define NOISE_FRAME_MS 89U
#define GOLDEN_HASH_K 0x9E3779B9U

/* Re-assert the configured ramps once a minute (recovers from drivers that
 * silently reset gamma on suspend/resume or DPMS cycles). */
#define REASSERT_MS 60000U

/* Exit codes used by the CLI. */
enum {
    EXIT_OK = 0,
    EXIT_USAGE = 2,
    EXIT_RUNTIME = 3
};

#define OUTPUT_NAME_MAX 64

typedef struct {
    double r;
    double g;
    double b;
    double strength;
    double brightness;
    double blue_limit;    /* multiplicative ceiling on blue, composes with any mode */
    double kelvin;        /* last color temperature sent, 0 when not in use */
    char mode[64];
    int modified;
} ColorState;

/*
 * One managed CRTC. Identity is (crtc id + connector name): CRTC ids are
 * reassigned freely by the X server on hotplug, so the connector name is what
 * actually ties a pristine ramp to a physical monitor.
 */
typedef struct {
    RRCrtc id;
    char output[OUTPUT_NAME_MAX];
    uint64_t edid_hash;
    XRRCrtcGamma *original;
    ColorState state;
    int active;              /* refreshed by every table sync */
} SavedCrtc;

/*
 * Backlight class of a sysfs device. The numeric order is the *preference*
 * order, highest wins. Documentation/ABI/stable/sysfs-class-backlight is
 * explicit about it: "when multiple backlight interfaces are available for a
 * single device, firmware control should be preferred to platform control
 * should be preferred to raw control", because the firmware interface is the
 * one the hardware and the OS agree on.
 */
typedef enum {
    BL_TYPE_UNKNOWN = 0,
    BL_TYPE_RAW,
    BL_TYPE_PLATFORM,
    BL_TYPE_FIRMWARE
} BacklightType;

typedef struct {
    char name[NAME_MAX + 1];
    char brightness_path[PATH_MAX];
    long original;
    long maximum;
    BacklightType type;
    int writable;
    int changed;
} Backlight;

/*
 * Click-through ARGB overlay window used for the CRT noise effect. The 13
 * pre-rendered tiles live server-side; animating a frame only changes the
 * window background pixmap, so the per-frame client cost is three tiny X
 * requests regardless of screen size.
 */
typedef struct {
    int intensity;        /* 0 = disabled */
    Window win;
    Pixmap tiles[NOISE_FRAMES];
    int have_tiles;
    int frame;
    Visual *visual;
    Colormap colormap;
    int colormap_valid;
    uint64_t next_frame_ms;
} NoiseOverlay;

/*
 * Recovery journal, format 4.
 *
 * boot_hash ties the file to one boot: the /tmp fallback directory survives
 * reboots, and replaying ramps captured before a reboot would be wrong.
 * Ramps carry the connector name and an EDID hash so a monitor is recognized
 * by its physical identity rather than by a CRTC id the X server reassigns
 * at will.
 */
typedef struct {
    char magic[8];
    uint32_t version;
    uint32_t count;
    uint64_t boot_hash;
    uint32_t backlight_count;
    uint32_t reserved;
} StateHeader;

typedef struct {
    uint64_t crtc;
    uint64_t edid_hash;      /* 0 when the output exposes no readable EDID */
    uint32_t size;
    uint32_t reserved;
    char output[OUTPUT_NAME_MAX];
} StateCrtcHeader;

/* Fixed-size record so the state file layout never depends on NAME_MAX. */
typedef struct {
    char name[256];
    uint64_t original;
    uint64_t maximum;
} StateBacklight;

/*
 * Fully parsed recovery file. The loader validates every record before the
 * caller touches the hardware, so a truncated or corrupted file can never be
 * half-applied and then reported as a success.
 */
typedef struct {
    uint64_t crtc;
    uint64_t edid_hash;
    uint32_t size;
    char output[OUTPUT_NAME_MAX];
    unsigned short *red;
    unsigned short *green;
    unsigned short *blue;
    int applied;             /* set by recovery_apply when it reached hardware */
} StateRamp;

/* Outcome of one recovery attempt, so callers can decide honestly whether
 * the journal may be discarded. */
typedef struct {
    int ramps_restored;
    int ramps_unmatched;
    int backlights_restored;
    int backlights_failed;
    int x_errors;
} RecoveryResult;

typedef struct {
    StateRamp ramps[MAX_JOURNAL_RAMPS];
    size_t ramp_count;
    StateBacklight backlights[MAX_JOURNAL_BACKLIGHTS];
    int backlight_applied[MAX_JOURNAL_BACKLIGHTS];
    size_t backlight_count;
} StateFile;

/* One currently active output, as seen through RandR. */
typedef struct {
    RRCrtc crtc;
    RROutput output;
    char name[OUTPUT_NAME_MAX];
    uint64_t edid_hash;
    int gamma_size;
} LiveOutput;

typedef struct {
    Display *dpy;
    Window root;
    int randr_event_base;
    int lock_fd;
    SavedCrtc crtcs[MAX_SAVED_CRTCS];
    size_t crtc_count;
    Backlight backlights[MAX_BACKLIGHTS];
    size_t backlight_count;
    ColorState state;        /* global look; also the default for new outputs */
    int target;              /* index into crtcs[], or -1 for "every output" */
    /*
     * Ramps inherited from a previous crashed instance whose monitor is not
     * connected right now. They are carried forward in the journal and
     * re-applied the moment that monitor comes back, so an unclean exit with
     * an unplugged monitor never loses its pristine capture.
     */
    /*
     * Bumped by every change to the managed-output table. Comparing counts is
     * not enough: swapping one monitor for another leaves the count identical
     * while the pristine ramps behind it are completely different.
     */
    unsigned long table_generation;
    StateRamp pending[MAX_SAVED_CRTCS];
    size_t pending_count;
    /*
     * Backlight levels a previous instance still owes but whose device was
     * missing or refused the write. They stay in the journal and are retried
     * on every restore until they land.
     */
    StateBacklight pending_bl[MAX_BACKLIGHTS];
    size_t pending_bl_count;
    NoiseOverlay noise;
    uint64_t next_reassert_ms;
    /* 1 when the runtime directory is XDG_RUNTIME_DIR, which the system
     * clears between sessions; 0 for the /tmp fallback, which survives a
     * reboot and therefore needs the boot stamp to be trustworthy. */
    int runtime_is_volatile;
    char last_warning[192];  /* most recent non-fatal problem, shown by status */
    char sticky_warning[192];/* a condition the user must act on; never auto-cleared */
    char socket_path[PATH_MAX];
    char recovery_path[PATH_MAX];
    char lock_path[PATH_MAX];
} App;

static volatile sig_atomic_t g_stop_requested = 0;
static int g_x_error_count = 0;
static App *g_app = NULL;

static void on_signal(int signo)
{
    (void)signo;
    g_stop_requested = 1;
}

static int x_error_handler(Display *dpy, XErrorEvent *event)
{
    (void)dpy;
    (void)event;
    g_x_error_count++;
    return 0;
}

static double clamp_double(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static unsigned long hash_string(const char *s)
{
    unsigned long h = 5381UL;
    unsigned char c;
    while (s != NULL && *s != '\0') {
        c = (unsigned char)*s++;
        h = ((h << 5U) + h) ^ (unsigned long)c;
    }
    return h;
}

/* FNV-1a, 64-bit, with the standard offset basis and prime. */
static uint64_t fnv1a64(const void *data, size_t len)
{
    const unsigned char *p = (const unsigned char *)data;
    uint64_t h = 14695981039346656037ULL;
    size_t i;
    for (i = 0U; i < len; ++i) {
        h ^= (uint64_t)p[i];
        h *= 1099511628211ULL;
    }
    return h;
}

/*
 * Identify the running kernel instance. The /tmp fallback directory outlives
 * a reboot, and replaying gamma ramps captured before one would restore state
 * that no longer describes anything. boot_id is the canonical Linux answer;
 * /proc/stat's btime is the fallback. 0 means "unknown", and journals written
 * with 0 are refused rather than trusted.
 */
static uint64_t boot_hash(void)
{
    static uint64_t cached = 0U;
    static int done = 0;
    char buf[128];
    FILE *fp;

    if (done) return cached;
    done = 1;

    fp = fopen("/proc/sys/kernel/random/boot_id", "r");
    if (fp != NULL) {
        size_t n = fread(buf, 1U, sizeof(buf), fp);
        (void)fclose(fp);
        if (n >= 32U) {
            cached = fnv1a64(buf, n);
            return cached;
        }
    }
    fp = fopen("/proc/stat", "r");
    if (fp != NULL) {
        while (fgets(buf, sizeof(buf), fp) != NULL) {
            if (strncmp(buf, "btime ", 6U) == 0) {
                cached = fnv1a64(buf, strlen(buf));
                break;
            }
        }
        (void)fclose(fp);
    }
    return cached;
}

static uint64_t now_ms(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0U;
    return (uint64_t)ts.tv_sec * 1000U + (uint64_t)ts.tv_nsec / 1000000U;
}

/*
 * Golden-ratio integer hash (Knuth multiplicative hashing). The constant is
 * floor(2^32 / phi); successive multiplies walk the maximally uniform Weyl
 * sequence of the golden ratio, which is what gives the noise its even,
 * clump-free "film grain" distribution.
 */
static uint32_t golden_hash(uint32_t x)
{
    x ^= x >> 16;
    x *= GOLDEN_HASH_K;
    x ^= x >> 13;
    x *= GOLDEN_HASH_K;
    x ^= x >> 16;
    return x;
}

/*
 * Fill one noise tile with premultiplied ARGB grain. Pure function (no X),
 * deterministic per (intensity, seed), so it is unit-testable offline.
 * Tuning constants are powers of the golden ratio:
 *   density ceiling 0.382 = 1/phi^2, alpha ceiling 0.618 = 1/phi,
 *   response exponent 0.618 = 1/phi (softens the low end of the dial),
 *   dark grain depth 0.236 = 1/phi^3, bright sparkle floor 0.618 = 1/phi.
 */
static void noise_fill_buffer(uint32_t *px, int w, int h, int intensity, uint32_t seed)
{
    double level = clamp_double((double)intensity, 0.0, 100.0) / 100.0;
    double response = pow(level, 0.618);
    double density = 0.382 * response;
    double alpha_ceiling = 0.618 * response;
    int x, y;

    for (y = 0; y < h; ++y) {
        for (x = 0; x < w; ++x) {
            size_t idx = (size_t)y * (size_t)w + (size_t)x;
            uint32_t h1 = golden_hash((uint32_t)idx ^ seed);
            double u = (double)h1 * (1.0 / 4294967296.0);

            if (u >= density) {
                px[idx] = 0U;
                continue;
            }
            {
                uint32_t h2 = golden_hash(h1 + GOLDEN_HASH_K);
                double ua = (double)h2 * (1.0 / 4294967296.0);
                double uv = (double)golden_hash(h2 ^ seed) * (1.0 / 4294967296.0);
                double a = alpha_ceiling * (0.382 + 0.618 * ua);
                double v = ((h2 >> 31) & 1U) ? (0.618 + 0.382 * uv)  /* bright sparkle */
                                             : (0.236 * uv);          /* dark grain */
                uint32_t alpha = (uint32_t)llround(a * 255.0);
                uint32_t gray = (uint32_t)llround(v * a * 255.0);     /* premultiplied */
                px[idx] = (alpha << 24) | (gray << 16) | (gray << 8) | gray;
            }
        }
    }
}

static int path_join(char *out, size_t out_size, const char *a, const char *b)
{
    int n;
    if (out == NULL || out_size == 0U || a == NULL || b == NULL) return -1;
    n = snprintf(out, out_size, "%s/%s", a, b);
    if (n < 0 || (size_t)n >= out_size) return -1;
    return 0;
}

/*
 * Write exactly len bytes unless an unrecoverable error occurs.
 * Handles EINTR and short writes; callers never need to assume that a single
 * write(2) transfers the complete buffer.
 */
static int write_all(int fd, const void *buffer, size_t len)
{
    const unsigned char *p = (const unsigned char *)buffer;
    size_t remaining = len;

    if (buffer == NULL && len != 0U) {
        errno = EINVAL;
        return -1;
    }

    while (remaining > 0U) {
        ssize_t n = write(fd, p, remaining);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (n == 0) {
            errno = EIO;
            return -1;
        }
        p += (size_t)n;
        remaining -= (size_t)n;
    }
    return 0;
}

static void buf_append(char *buf, size_t size, size_t *used, const char *fmt, ...)
    PT_PRINTF(4, 5);

static void app_warn(App *app, const char *fmt, ...) PT_PRINTF(2, 3);

static void app_warn(App *app, const char *fmt, ...)
{
    va_list ap;
    if (app == NULL) return;
    va_start(ap, fmt);
    (void)vsnprintf(app->last_warning, sizeof(app->last_warning), fmt, ap);
    va_end(ap);
}

static void buf_append(char *buf, size_t size, size_t *used, const char *fmt, ...)
{
    va_list ap;
    int n;

    if (buf == NULL || used == NULL || *used >= size) return;
    va_start(ap, fmt);
    n = vsnprintf(buf + *used, size - *used, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if ((size_t)n >= size - *used) {
        *used = size;
    } else {
        *used += (size_t)n;
    }
}

/* Sets *volatile_dir to 1 when the chosen directory is cleared between
 * sessions (XDG_RUNTIME_DIR), 0 for the persistent /tmp fallback. */
static int get_runtime_base_ex(char *out, size_t out_size, int *volatile_dir)
{
    const char *dir = getenv("XDG_RUNTIME_DIR");
    struct stat st;
    int n;

    if (volatile_dir != NULL) *volatile_dir = 0;

    /*
     * XDG_RUNTIME_DIR is only trusted when it is a directory we own and that
     * nobody else can enter or write (the spec mandates 0700). Anything
     * looser falls through to the private directory below.
     */
    if (dir != NULL && dir[0] == '/' && stat(dir, &st) == 0 &&
        S_ISDIR(st.st_mode) && st.st_uid == getuid() &&
        (st.st_mode & (S_IRWXG | S_IRWXO)) == 0U) {
        n = snprintf(out, out_size, "%s", dir);
        if (n < 0 || (size_t)n >= out_size) return -1;
        if (volatile_dir != NULL) *volatile_dir = 1;
        return 0;
    }

    /*
     * Private owner-only fallback directory. State/socket names are
     * predictable, so they must never live directly in a shared
     * world-writable directory where another user could squat them.
     */
    n = snprintf(out, out_size, "/tmp/phostint-%lu", (unsigned long)getuid());
    if (n < 0 || (size_t)n >= out_size) return -1;
    if (mkdir(out, S_IRWXU) != 0 && errno != EEXIST) return -1;
    if (lstat(out, &st) != 0 || !S_ISDIR(st.st_mode) || st.st_uid != getuid() ||
        (st.st_mode & (S_IRWXG | S_IRWXO)) != 0U) {
        return -1;
    }
    return 0;
}

static int get_runtime_base(char *out, size_t out_size)
{
    return get_runtime_base_ex(out, out_size, NULL);
}

static int make_runtime_paths_ex(char *socket_path, size_t socket_size,
                                 char *recovery_path, size_t recovery_size,
                                 char *lock_path, size_t lock_size,
                                 int *volatile_dir)
{
    char base[PATH_MAX];
    char socket_name[128];
    char state_name[128];
    char lock_name[128];
    const char *display = getenv("DISPLAY");
    unsigned long h;
    int n1, n2, n3;

    if (get_runtime_base_ex(base, sizeof(base), volatile_dir) != 0) return -1;
    if (display == NULL || display[0] == '\0') display = ":0";
    h = hash_string(display) & 0xffffffffUL;

    n1 = snprintf(socket_name, sizeof(socket_name), "phostint-%lu-%08lx.sock",
                  (unsigned long)getuid(), h);
    n2 = snprintf(state_name, sizeof(state_name), "phostint-%lu-%08lx.state",
                  (unsigned long)getuid(), h);
    n3 = snprintf(lock_name, sizeof(lock_name), "phostint-%lu-%08lx.lock",
                  (unsigned long)getuid(), h);
    if (n1 < 0 || n2 < 0 || n3 < 0 ||
        (size_t)n1 >= sizeof(socket_name) ||
        (size_t)n2 >= sizeof(state_name) ||
        (size_t)n3 >= sizeof(lock_name)) return -1;

    if (path_join(socket_path, socket_size, base, socket_name) != 0) return -1;
    if (path_join(recovery_path, recovery_size, base, state_name) != 0) return -1;
    if (path_join(lock_path, lock_size, base, lock_name) != 0) return -1;
    return 0;
}


static int read_long_file(const char *path, long *value)
{
    FILE *fp;
    long v;
    if (path == NULL || value == NULL) return -1;
    fp = fopen(path, "r");
    if (fp == NULL) return -1;
    if (fscanf(fp, "%ld", &v) != 1) {
        fclose(fp);
        return -1;
    }
    if (fclose(fp) != 0) return -1;
    *value = v;
    return 0;
}

static int write_long_file(const char *path, long value)
{
    int fd;
    char buf[64];
    int n;

    if (path == NULL) return -1;
    fd = open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    n = snprintf(buf, sizeof(buf), "%ld\n", value);
    if (n < 0 || (size_t)n >= sizeof(buf)) {
        (void)close(fd);
        return -1;
    }
    if (write_all(fd, buf, (size_t)n) != 0) {
        (void)close(fd);
        return -1;
    }
    return close(fd) == 0 ? 0 : -1;
}

/*
 * Root of the backlight class. Only "phostint selftest" ever changes it, so
 * the daemon and the CLI can never be pointed somewhere else at runtime; the
 * seam exists purely so the write-ahead ordering and the device-preference
 * rules can be exercised against a temporary directory.
 */
static const char *g_backlight_root = "/sys/class/backlight";

static BacklightType read_backlight_type(const char *device_dir)
{
    char path[PATH_MAX];
    char buf[32];
    FILE *fp;
    size_t n;

    if (path_join(path, sizeof(path), device_dir, "type") != 0) return BL_TYPE_UNKNOWN;
    fp = fopen(path, "r");
    if (fp == NULL) return BL_TYPE_UNKNOWN;
    n = fread(buf, 1U, sizeof(buf) - 1U, fp);
    (void)fclose(fp);
    buf[n] = '\0';
    buf[strcspn(buf, "\r\n")] = '\0';
    if (strcmp(buf, "raw") == 0) return BL_TYPE_RAW;
    if (strcmp(buf, "platform") == 0) return BL_TYPE_PLATFORM;
    if (strcmp(buf, "firmware") == 0) return BL_TYPE_FIRMWARE;
    return BL_TYPE_UNKNOWN;
}

static const char *backlight_type_name(BacklightType t)
{
    switch (t) {
    case BL_TYPE_RAW: return "raw";
    case BL_TYPE_PLATFORM: return "platform";
    case BL_TYPE_FIRMWARE: return "firmware";
    default: return "unknown";
    }
}

static void discover_backlights(App *app)
{
    const char *base = g_backlight_root;
    DIR *dir;
    struct dirent *ent;

    app->backlight_count = 0U;
    dir = opendir(base);
    if (dir == NULL) return;

    while ((ent = readdir(dir)) != NULL && app->backlight_count < MAX_BACKLIGHTS) {
        Backlight *bl;
        char device_dir[PATH_MAX];
        char max_path[PATH_MAX];
        long current, maximum;
        int n;

        if (ent->d_name[0] == '.') continue;
        n = snprintf(device_dir, sizeof(device_dir), "%s/%s", base, ent->d_name);
        if (n < 0 || (size_t)n >= sizeof(device_dir)) continue;
        if (path_join(max_path, sizeof(max_path), device_dir, "max_brightness") != 0) continue;

        bl = &app->backlights[app->backlight_count];
        memset(bl, 0, sizeof(*bl));
        if (path_join(bl->brightness_path, sizeof(bl->brightness_path),
                      device_dir, "brightness") != 0) continue;
        if (read_long_file(max_path, &maximum) != 0 || maximum <= 0) continue;
        if (read_long_file(bl->brightness_path, &current) != 0 || current < 0) continue;

        (void)snprintf(bl->name, sizeof(bl->name), "%s", ent->d_name);
        bl->maximum = maximum;
        bl->original = current;
        bl->type = read_backlight_type(device_dir);
        bl->writable = (access(bl->brightness_path, W_OK) == 0) ? 1 : 0;
        bl->changed = 0;
        app->backlight_count++;
    }
    (void)closedir(dir);
}

/*
 * Pick the single device a bare "backlight" command drives. Writing every
 * writable device at once is wrong: a laptop often exposes both a firmware
 * device (acpi_video0) and a raw panel device (intel_backlight) for the same
 * physical panel, and driving both fights the firmware. Preference follows
 * the kernel's own advice (Documentation/ABI/stable/sysfs-class-backlight):
 * firmware > platform > raw > unknown.
 */
static Backlight *preferred_backlight(App *app)
{
    Backlight *best = NULL;
    size_t i;

    for (i = 0U; i < app->backlight_count; ++i) {
        Backlight *bl = &app->backlights[i];
        if (!bl->writable) continue;
        if (best == NULL || (int)bl->type > (int)best->type) best = bl;
    }
    return best;
}

static Backlight *find_backlight(App *app, const char *name)
{
    size_t i;
    for (i = 0U; i < app->backlight_count; ++i) {
        if (strcmp(app->backlights[i].name, name) == 0) return &app->backlights[i];
    }
    return NULL;
}

/*
 * Restore one backlight value described by a state-file record. The device
 * name is validated and the sysfs path is rebuilt locally, so a corrupted or
 * hostile state file can never steer writes outside /sys/class/backlight.
 */
/*
 * Returns 1 on success, 0 when the record is not applicable (bad name, device
 * gone, not writable, or a different scale than when it was recorded), -1
 * when the device exists and should have accepted the write but did not.
 */
static int restore_backlight_record(const StateBacklight *rec)
{
    char device_dir[PATH_MAX];
    char bright_path[PATH_MAX];
    char max_path[PATH_MAX];
    size_t len;
    long maximum = 0L;
    long target;
    int n;

    len = strnlen(rec->name, sizeof(rec->name));
    if (len == 0U || len >= sizeof(rec->name)) return 0;
    if (rec->name[0] == '.' || strchr(rec->name, '/') != NULL) return 0;
    if (rec->original > (uint64_t)LONG_MAX || rec->maximum > (uint64_t)LONG_MAX) return 0;

    n = snprintf(device_dir, sizeof(device_dir), "%s/%s", g_backlight_root, rec->name);
    if (n < 0 || (size_t)n >= sizeof(device_dir)) return 0;
    if (path_join(bright_path, sizeof(bright_path), device_dir, "brightness") != 0) return 0;
    if (path_join(max_path, sizeof(max_path), device_dir, "max_brightness") != 0) return 0;

    if (read_long_file(max_path, &maximum) != 0 || maximum <= 0) return 0;
    /*
     * The stored value is a raw level on the scale that existed when it was
     * captured. If the driver now reports a different max_brightness, this is
     * effectively a different device (or a re-probed one) and the number is
     * meaningless - writing it could darken the panel arbitrarily.
     */
    if ((uint64_t)maximum != rec->maximum) return 0;
    target = (long)rec->original;
    if (target < 0L) return 0;
    if (target > maximum) target = maximum;
    if (access(bright_path, W_OK) != 0) return 0;
    return write_long_file(bright_path, target) == 0 ? 1 : -1;
}

/* ==================== Noise overlay (golden-ratio CRT grain) ==================== */

/*
 * True per-pixel transparency needs a compositing manager (picom, xcompmgr,
 * a compositing WM). Without one, an ARGB window renders opaque garbage, so
 * the overlay must refuse to start - and must shut itself down if the
 * compositor disappears while the noise is active.
 */
static int compositor_present(Display *dpy)
{
    char name[32];
    Atom atom;
    (void)snprintf(name, sizeof(name), "_NET_WM_CM_S%d", DefaultScreen(dpy));
    atom = XInternAtom(dpy, name, False);
    return XGetSelectionOwner(dpy, atom) != None ? 1 : 0;
}

static void noise_destroy(App *app)
{
    NoiseOverlay *n = &app->noise;
    int f;

    if (n->have_tiles) {
        for (f = 0; f < NOISE_FRAMES; ++f) {
            if (n->tiles[f] != None) XFreePixmap(app->dpy, n->tiles[f]);
            n->tiles[f] = None;
        }
        n->have_tiles = 0;
    }
    if (n->win != None) {
        XDestroyWindow(app->dpy, n->win);
        n->win = None;
    }
    if (n->colormap_valid) {
        XFreeColormap(app->dpy, n->colormap);
        n->colormap_valid = 0;
    }
    n->visual = NULL;
    n->intensity = 0;
    n->frame = 0;
    XFlush(app->dpy);
}

static int noise_render_tiles(App *app, int intensity)
{
    NoiseOverlay *n = &app->noise;
    XImage *img;
    uint32_t *buffer;
    GC gc = NULL;
    char *img_data;
    int f, x, y;
    int ok = 0;

    buffer = malloc((size_t)NOISE_TILE_W * (size_t)NOISE_TILE_H * sizeof(uint32_t));
    img_data = malloc((size_t)NOISE_TILE_W * (size_t)NOISE_TILE_H * 4U);
    if (buffer == NULL || img_data == NULL) {
        free(buffer);
        free(img_data);
        return -1;
    }
    img = XCreateImage(app->dpy, n->visual, 32U, ZPixmap, 0, img_data,
                       NOISE_TILE_W, NOISE_TILE_H, 32, 0);
    if (img == NULL) {
        free(buffer);
        free(img_data);
        return -1;
    }

    for (f = 0; f < NOISE_FRAMES; ++f) {
        uint32_t seed = golden_hash((uint32_t)(f + 1) * GOLDEN_HASH_K);

        if (n->tiles[f] == None) {
            n->tiles[f] = XCreatePixmap(app->dpy, app->root,
                                        NOISE_TILE_W, NOISE_TILE_H, 32U);
            if (n->tiles[f] == None) goto out;
        }
        if (gc == NULL) {
            gc = XCreateGC(app->dpy, n->tiles[f], 0UL, NULL);
            if (gc == NULL) goto out;
        }
        noise_fill_buffer(buffer, NOISE_TILE_W, NOISE_TILE_H, intensity, seed);
        /* XPutPixel honours the server byte order, keeping this portable to
         * big-endian displays at a one-time cost of ~1M calls per regen. */
        for (y = 0; y < NOISE_TILE_H; ++y) {
            for (x = 0; x < NOISE_TILE_W; ++x) {
                XPutPixel(img, x, y,
                          buffer[(size_t)y * (size_t)NOISE_TILE_W + (size_t)x]);
            }
        }
        XPutImage(app->dpy, n->tiles[f], gc, img, 0, 0, 0, 0,
                  NOISE_TILE_W, NOISE_TILE_H);
    }
    n->have_tiles = 1;
    ok = 1;

out:
    if (gc != NULL) XFreeGC(app->dpy, gc);
    XDestroyImage(img); /* also frees img_data */
    free(buffer);
    if (!ok) {
        /* Release whatever this call already allocated: never leak pixmaps
         * on a partial failure, independently of what the caller does. */
        for (f = 0; f < NOISE_FRAMES; ++f) {
            if (n->tiles[f] != None) {
                XFreePixmap(app->dpy, n->tiles[f]);
                n->tiles[f] = None;
            }
        }
        n->have_tiles = 0;
    }
    return ok ? 0 : -1;
}

/*
 * Does this visual really carry an alpha channel? Depth 32 alone is not
 * proof. When the build has XRender (the authoritative source, since it is
 * the extension that defines picture formats) ask it directly; otherwise fall
 * back to checking that the RGB masks leave bits free for alpha.
 */
static int visual_has_alpha(Display *dpy, const XVisualInfo *vi)
{
#ifdef HAVE_XRENDER
    XRenderPictFormat *fmt = XRenderFindVisualFormat(dpy, vi->visual);
    if (fmt != NULL) {
        return (fmt->type == PictTypeDirect && fmt->direct.alphaMask != 0) ? 1 : 0;
    }
    /* No format for this visual: fall through to the mask heuristic. */
#else
    (void)dpy;
#endif
    {
        unsigned long rgb = vi->red_mask | vi->green_mask | vi->blue_mask;
        unsigned long alpha = 0xffffffffUL & ~rgb;
        return (vi->red_mask != 0UL && vi->green_mask != 0UL &&
                vi->blue_mask != 0UL && alpha != 0UL) ? 1 : 0;
    }
}

static int find_argb_visual(Display *dpy, int screen, XVisualInfo *out)
{
    XVisualInfo template;
    XVisualInfo *list;
    int count = 0;
    int i;
    int found = 0;

    memset(&template, 0, sizeof(template));
    template.screen = screen;
    template.depth = 32;
    template.class = TrueColor;
    list = XGetVisualInfo(dpy, VisualScreenMask | VisualDepthMask | VisualClassMask,
                          &template, &count);
    if (list == NULL) return 0;
    for (i = 0; i < count; ++i) {
        if (visual_has_alpha(dpy, &list[i])) {
            *out = list[i];
            found = 1;
            break;
        }
    }
    XFree(list);
    return found;
}

static int noise_set(App *app, int intensity, char *err, size_t err_size)
{
    NoiseOverlay *n = &app->noise;
    int shape_event = 0, shape_error = 0;

    if (err != NULL && err_size > 0U) err[0] = '\0';
    if (intensity <= 0) {
        noise_destroy(app);
        return 0;
    }

    if (!XShapeQueryExtension(app->dpy, &shape_event, &shape_error)) {
        (void)snprintf(err, err_size, "%s",
                       "the X server lacks the SHAPE extension needed for a click-through overlay");
        return -1;
    }
    if (!compositor_present(app->dpy)) {
        (void)snprintf(err, err_size, "%s",
                       "a compositing manager is required (e.g. run picom on i3wm); none detected");
        return -1;
    }

    if (n->win == None) {
        XVisualInfo vinfo;
        XSetWindowAttributes attrs;
        int screen = DefaultScreen(app->dpy);
        int width = DisplayWidth(app->dpy, screen);
        int height = DisplayHeight(app->dpy, screen);
        Atom shadow_atom;
        unsigned long shadow_off = 0UL;

        if (!find_argb_visual(app->dpy, screen, &vinfo)) {
            (void)snprintf(err, err_size, "%s",
                           "no 32-bit visual with a real alpha channel is available on this display");
            return -1;
        }
        n->visual = vinfo.visual;
        n->colormap = XCreateColormap(app->dpy, app->root, n->visual, AllocNone);
        n->colormap_valid = 1;

        memset(&attrs, 0, sizeof(attrs));
        attrs.override_redirect = True;
        attrs.colormap = n->colormap;
        attrs.border_pixel = 0UL;
        attrs.background_pixel = 0UL; /* alpha 0: fully transparent */
        n->win = XCreateWindow(app->dpy, app->root, 0, 0,
                               (unsigned int)width, (unsigned int)height, 0U,
                               32, InputOutput, n->visual,
                               CWOverrideRedirect | CWColormap | CWBorderPixel |
                               CWBackPixel,
                               &attrs);
        if (n->win == None) {
            noise_destroy(app);
            (void)snprintf(err, err_size, "%s", "could not create the overlay window");
            return -1;
        }
        /* Empty input shape: every click, scroll and key passes through. */
        XShapeCombineRectangles(app->dpy, n->win, ShapeInput, 0, 0,
                                NULL, 0, ShapeSet, Unsorted);
        XStoreName(app->dpy, n->win, "PhosTint noise overlay");
        shadow_atom = XInternAtom(app->dpy, "_COMPTON_SHADOW", False);
        XChangeProperty(app->dpy, n->win, shadow_atom, XA_CARDINAL, 32,
                        PropModeReplace, (unsigned char *)&shadow_off, 1);
    }

    if (noise_render_tiles(app, intensity) != 0) {
        noise_destroy(app);
        (void)snprintf(err, err_size, "%s", "could not render the noise tiles");
        return -1;
    }

    n->intensity = intensity;
    n->frame = 0;
    XSetWindowBackgroundPixmap(app->dpy, n->win, n->tiles[0]);
    XMapRaised(app->dpy, n->win);
    XClearWindow(app->dpy, n->win);
    XFlush(app->dpy);
    n->next_frame_ms = now_ms() + NOISE_FRAME_MS;
    return 0;
}

static void noise_frame_tick(App *app)
{
    NoiseOverlay *n = &app->noise;
    uint64_t now;

    if (n->intensity <= 0 || n->win == None || !n->have_tiles) return;

    n->frame = (n->frame + 1) % NOISE_FRAMES;
    /* Once per full cycle (~1.2 s), verify the compositor is still alive;
     * without it the ARGB overlay would render as an opaque black sheet. */
    if (n->frame == 0 && !compositor_present(app->dpy)) {
        noise_destroy(app);
        return;
    }
    XSetWindowBackgroundPixmap(app->dpy, n->win, n->tiles[n->frame]);
    XClearWindow(app->dpy, n->win);
    XRaiseWindow(app->dpy, n->win);
    XFlush(app->dpy);

    now = now_ms();
    n->next_frame_ms += NOISE_FRAME_MS;
    if (n->next_frame_ms < now) n->next_frame_ms = now + NOISE_FRAME_MS;
}

static void noise_resize(App *app)
{
    NoiseOverlay *n = &app->noise;
    int screen = DefaultScreen(app->dpy);

    if (n->win == None) return;
    XMoveResizeWindow(app->dpy, n->win, 0, 0,
                      (unsigned int)DisplayWidth(app->dpy, screen),
                      (unsigned int)DisplayHeight(app->dpy, screen));
    XClearWindow(app->dpy, n->win);
    XRaiseWindow(app->dpy, n->win);
    XFlush(app->dpy);
}

static void free_saved_crtcs(App *app)
{
    size_t i;
    for (i = 0U; i < app->crtc_count; ++i) {
        if (app->crtcs[i].original != NULL) {
            XRRFreeGamma(app->crtcs[i].original);
            app->crtcs[i].original = NULL;
        }
    }
    app->crtc_count = 0U;
}

static SavedCrtc *find_saved_crtc(App *app, RRCrtc id, size_t *index_out)
{
    size_t i;
    for (i = 0U; i < app->crtc_count; ++i) {
        if (app->crtcs[i].id == id) {
            if (index_out != NULL) *index_out = i;
            return &app->crtcs[i];
        }
    }
    return NULL;
}

static void remove_saved_crtc(App *app, size_t index)
{
    if (index >= app->crtc_count) return;
    app->table_generation++;
    if (app->crtcs[index].original != NULL) {
        XRRFreeGamma(app->crtcs[index].original);
    }
    memmove(&app->crtcs[index], &app->crtcs[index + 1U],
            (app->crtc_count - index - 1U) * sizeof(app->crtcs[0]));
    app->crtc_count--;
}

/*
 * Decide what to do with one active CRTC during a topology re-sync. Pure
 * function so the hotplug policy can be unit-tested without hardware:
 *
 *   KEEP     - the same connector is still on this CRTC with the same ramp
 *              size; its pristine capture stays untouched.
 *   TRANSFER - this connector was previously captured on a different CRTC
 *              (the X server re-assigned it): move the pristine ramp and the
 *              per-output look over instead of capturing the current - very
 *              possibly already tinted - ramp.
 *   CAPTURE  - genuinely new or incompatible: capture the ramp as-is.
 */
typedef enum {
    CRTC_KEEP = 0,
    CRTC_TRANSFER,
    CRTC_CAPTURE
} CrtcPlan;

static CrtcPlan crtc_plan(int have_saved, const char *saved_output, uint64_t saved_edid,
                          int saved_size,
                          const char *current_output, uint64_t current_edid,
                          int current_size,
                          int have_donor, int donor_size)
{
    if (have_saved && saved_size == current_size) {
        int same;
        /*
         * EDID decides whenever both sides have one: unplugging a monitor and
         * plugging a different one into the same port, on the same CRTC, with
         * the same ramp size is otherwise indistinguishable - and would hand
         * the new monitor the old monitor's pristine ramp.
         */
        if (saved_edid != 0U && current_edid != 0U) {
            same = (saved_edid == current_edid);
        } else {
            same = (saved_output != NULL && current_output != NULL &&
                    strcmp(saved_output, current_output) == 0);
        }
        if (same) return CRTC_KEEP;
    }
    if (have_donor && donor_size == current_size) return CRTC_TRANSFER;
    return CRTC_CAPTURE;
}

/*
 * Hash of a monitor's EDID. Connector names ("HDMI-1") describe a socket, not
 * a monitor: swap two screens between two ports and the names follow the
 * ports. The EDID (manufacturer, product, serial, mode block) is what
 * actually identifies the physical panel. Returns 0 when the driver exposes
 * no EDID, in which case the connector name remains the only key available.
 */
static uint64_t output_edid_hash(Display *dpy, RROutput output)
{
    Atom edid_atom;
    Atom actual_type = None;
    int actual_format = 0;
    unsigned long nitems = 0UL;
    unsigned long bytes_after = 0UL;
    unsigned char *prop = NULL;
    uint64_t hash = 0U;

    if (output == None) return 0U;
    edid_atom = XInternAtom(dpy, "EDID", True);
    if (edid_atom == None) return 0U;
    if (XRRGetOutputProperty(dpy, output, edid_atom, 0L, 512L, False, False,
                             AnyPropertyType, &actual_type, &actual_format,
                             &nitems, &bytes_after, &prop) != Success) {
        return 0U;
    }
    if (prop != NULL) {
        if (actual_format == 8 && nitems >= 128UL) hash = fnv1a64(prop, (size_t)nitems);
        XFree(prop);
    }
    return hash;
}

/*
 * Name and identity of everything a CRTC drives. In a clone/mirror setup one
 * CRTC feeds several outputs, so looking only at outputs[0] would call two
 * different configurations the same thing. The name joins the connectors
 * ("HDMI-1+DP-2") and the identity hash folds in every EDID.
 */
static void crtc_identity(Display *dpy, XRRScreenResources *res,
                          const XRRCrtcInfo *info, RRCrtc id,
                          char *out, size_t size, uint64_t *edid_out)
{
    uint64_t hashes[16];
    size_t nhash = 0U;
    int i;

    out[0] = '\0';
    if (edid_out != NULL) *edid_out = 0U;
    if (info != NULL) {
        for (i = 0; i < info->noutput; ++i) {
            XRROutputInfo *oi = XRRGetOutputInfo(dpy, res, info->outputs[i]);
            uint64_t h = output_edid_hash(dpy, info->outputs[i]);
            if (oi != NULL) {
                if (oi->name != NULL) {
                    size_t used = strlen(out);
                    (void)snprintf(out + used, size - used, "%s%s",
                                   used > 0U ? "+" : "", oi->name);
                }
                XRRFreeOutputInfo(oi);
            }
            if (h != 0U && nhash < sizeof(hashes) / sizeof(hashes[0])) hashes[nhash++] = h;
        }
    }
    if (out[0] == '\0') (void)snprintf(out, size, "crtc-%lu", (unsigned long)id);

    if (edid_out != NULL && nhash > 0U) {
        /*
         * Sort, then hash the sorted list: order-independent like XOR, but
         * without its fatal flaw - two identical monitors cloned on one CRTC
         * would XOR to exactly zero, i.e. "no identity at all".
         */
        size_t a, b;
        for (a = 1U; a < nhash; ++a) {
            uint64_t key = hashes[a];
            b = a;
            while (b > 0U && hashes[b - 1U] > key) {
                hashes[b] = hashes[b - 1U];
                b--;
            }
            hashes[b] = key;
        }
        *edid_out = fnv1a64(hashes, nhash * sizeof(hashes[0]));
    }
}

/*
 * Snapshot every active output: CRTC, connector name, EDID hash and gamma
 * size, in one pass. Every routine that touches the hardware works from this
 * snapshot, so the whole program shares one consistent view of the topology.
 */
static int enumerate_live_outputs(Display *dpy, Window root,
                                  LiveOutput *out, size_t max_out)
{
    XRRScreenResources *res;
    size_t n = 0U;
    int c;

    /*
     * Returns -1 when the topology could not be read at all. That must never
     * be reported as "zero active outputs": every caller then correctly
     * concludes there is nothing to restore and would declare success while
     * the screen is still tinted.
     */
    res = XRRGetScreenResourcesCurrent(dpy, root);
    if (res == NULL) return -1;
    for (c = 0; c < res->ncrtc && n < max_out; ++c) {
        XRRCrtcInfo *info = XRRGetCrtcInfo(dpy, res, res->crtcs[c]);
        int size;

        if (info == NULL) continue;
        if (info->mode == None || info->noutput <= 0) {
            XRRFreeCrtcInfo(info);
            continue;
        }
        size = XRRGetCrtcGammaSize(dpy, res->crtcs[c]);
        if (size <= 0 || size > 65536) {
            XRRFreeCrtcInfo(info);
            continue;
        }
        memset(&out[n], 0, sizeof(out[n]));
        out[n].crtc = res->crtcs[c];
        out[n].output = info->outputs[0];
        out[n].gamma_size = size;
        crtc_identity(dpy, res, info, res->crtcs[c],
                      out[n].name, sizeof(out[n].name), &out[n].edid_hash);
        XRRFreeCrtcInfo(info);
        n++;
    }
    XRRFreeScreenResources(res);
    return (int)n;
}

static const LiveOutput *live_find_crtc(const LiveOutput *live, size_t n, RRCrtc crtc)
{
    size_t i;
    for (i = 0U; i < n; ++i) {
        if (live[i].crtc == crtc) return &live[i];
    }
    return NULL;
}

/*
 * Does a journal record describe this live output? EDID is authoritative
 * when both sides have one; otherwise the connector name is the fallback.
 * A record with neither (an old-style entry) matches only by CRTC id.
 */
static int identity_matches(const char *rec_name, uint64_t rec_edid, uint64_t rec_crtc,
                            const LiveOutput *live)
{
    if (rec_edid != 0U && live->edid_hash != 0U) return rec_edid == live->edid_hash;
    if (rec_name != NULL && rec_name[0] != '\0' && live->name[0] != '\0') {
        return strcmp(rec_name, live->name) == 0;
    }
    return rec_crtc == (uint64_t)live->crtc;
}

/* Index of a saved entry for this connector whose CRTC is currently inactive
 * (a monitor the server just moved to a different CRTC), or -1. */
static int find_donor(App *app, const LiveOutput *live, const LiveOutput *all, size_t nlive)
{
    size_t i;
    for (i = 0U; i < app->crtc_count; ++i) {
        if (app->crtcs[i].id == live->crtc) continue;
        if (!identity_matches(app->crtcs[i].output, app->crtcs[i].edid_hash,
                              (uint64_t)app->crtcs[i].id, live)) {
            continue;
        }
        /* Only a saved entry whose own CRTC is no longer driving anything can
         * hand its capture over; otherwise two live outputs would share it. */
        if (live_find_crtc(all, nlive, app->crtcs[i].id) == NULL) return (int)i;
    }
    return -1;
}

/*
 * Take a pristine ramp inherited from a crashed instance for this output, if
 * one is waiting. Returns the index in app->pending, or -1.
 */
static int find_pending(App *app, const LiveOutput *live)
{
    size_t i;
    for (i = 0U; i < app->pending_count; ++i) {
        if (app->pending[i].size != (uint32_t)live->gamma_size) continue;
        if (identity_matches(app->pending[i].output, app->pending[i].edid_hash,
                             app->pending[i].crtc, live)) {
            return (int)i;
        }
    }
    return -1;
}

static void pending_remove(App *app, size_t index)
{
    if (index >= app->pending_count) return;
    free(app->pending[index].red);
    free(app->pending[index].green);
    free(app->pending[index].blue);
    memmove(&app->pending[index], &app->pending[index + 1U],
            (app->pending_count - index - 1U) * sizeof(app->pending[0]));
    app->pending_count--;
}

static void pending_clear(App *app)
{
    while (app->pending_count > 0U) pending_remove(app, app->pending_count - 1U);
}

/* Move ownership of a journal ramp into the pending list. */
static int pending_take(App *app, StateRamp *src)
{
    if (app->pending_count >= MAX_SAVED_CRTCS) return -1;
    app->pending[app->pending_count] = *src;
    app->pending[app->pending_count].applied = 0;
    app->pending_count++;
    src->red = NULL;
    src->green = NULL;
    src->blue = NULL;
    return 0;
}

/*
 * Re-synchronize the saved-CRTC table with the current RandR topology.
 * Entries whose CRTC vanished are dropped; monitors that moved keep their
 * pristine ramp and their per-output look; newly active CRTCs are adopted
 * with the global look as their default. Returns the number of managed
 * active CRTCs, or -1 if the topology could not be read.
 */
static int crtc_table_sync(App *app)
{
    LiveOutput live[MAX_SAVED_CRTCS];
    XRRScreenResources *res;
    size_t nlive;
    size_t i, l;
    int nlive_signed;
    int managed = 0;

    nlive_signed = enumerate_live_outputs(app->dpy, app->root, live, MAX_SAVED_CRTCS);
    if (nlive_signed < 0) return -1;
    nlive = (size_t)nlive_signed;

    /* Drop entries whose CRTC the server no longer advertises at all. */
    res = XRRGetScreenResourcesCurrent(app->dpy, app->root);
    if (res == NULL) return -1;
    i = 0U;
    while (i < app->crtc_count) {
        int exists = 0;
        int c;
        for (c = 0; c < res->ncrtc; ++c) {
            if (res->crtcs[c] == app->crtcs[i].id) {
                exists = 1;
                break;
            }
        }
        if (exists) ++i;
        else remove_saved_crtc(app, i);
    }
    XRRFreeScreenResources(res);

    for (i = 0U; i < app->crtc_count; ++i) app->crtcs[i].active = 0;

    for (l = 0U; l < nlive; ++l) {
        const LiveOutput *lo = &live[l];
        SavedCrtc *saved;
        size_t idx = 0U;
        int donor;
        int pending;
        CrtcPlan plan;
        XRRCrtcGamma *gamma;

        saved = find_saved_crtc(app, lo->crtc, &idx);
        donor = find_donor(app, lo, live, nlive);
        plan = crtc_plan(saved != NULL,
                         saved != NULL ? saved->output : NULL,
                         saved != NULL ? saved->edid_hash : 0U,
                         (saved != NULL && saved->original != NULL) ? saved->original->size : -1,
                         lo->name, lo->edid_hash, lo->gamma_size,
                         donor >= 0,
                         (donor >= 0 && app->crtcs[donor].original != NULL)
                             ? app->crtcs[donor].original->size : -1);

        if (plan == CRTC_KEEP) {
            saved->active = 1;
            saved->edid_hash = lo->edid_hash;
            /*
             * KEEP also covers "same panel, different port": the EDID proves
             * identity even though the connector changed. The stored name has
             * to follow, or status, 'output NAME ...' targeting and the
             * journal would all keep referring to a socket this monitor left.
             * A changed identity is a journal-worthy event.
             */
            if (strcmp(saved->output, lo->name) != 0) {
                (void)snprintf(saved->output, sizeof(saved->output), "%s", lo->name);
                app->table_generation++;
            }
            managed++;
            continue;
        }
        if (plan == CRTC_TRANSFER) {
            SavedCrtc moved = app->crtcs[donor];
            if (saved != NULL) {
                remove_saved_crtc(app, idx);
                if (donor > (int)idx) donor--;
            }
            app->crtcs[donor].original = NULL;   /* detach before freeing the slot */
            remove_saved_crtc(app, (size_t)donor);
            if (app->crtc_count >= MAX_SAVED_CRTCS) {
                XRRFreeGamma(moved.original);
                continue;
            }
            moved.id = lo->crtc;
            moved.edid_hash = lo->edid_hash;
            moved.active = 1;
            (void)snprintf(moved.output, sizeof(moved.output), "%s", lo->name);
            app->crtcs[app->crtc_count++] = moved;
            app->table_generation++;
            managed++;
            continue;
        }

        if (saved != NULL) remove_saved_crtc(app, idx);
        if (app->crtc_count >= MAX_SAVED_CRTCS) continue;

        /*
         * A monitor that was unplugged while a previous instance was tinting
         * it left its pristine ramp in the journal. Adopt that instead of
         * capturing whatever is on the wire now, and push it to the hardware
         * so the monitor comes back exactly as the user left it.
         */
        pending = find_pending(app, lo);
        if (pending >= 0) {
            StateRamp *pr = &app->pending[pending];
            gamma = XRRAllocGamma((int)pr->size);
            if (gamma != NULL) {
                memcpy(gamma->red, pr->red, (size_t)pr->size * sizeof(unsigned short));
                memcpy(gamma->green, pr->green, (size_t)pr->size * sizeof(unsigned short));
                memcpy(gamma->blue, pr->blue, (size_t)pr->size * sizeof(unsigned short));
                XRRSetCrtcGamma(app->dpy, lo->crtc, gamma);
                memset(&app->crtcs[app->crtc_count], 0, sizeof(app->crtcs[0]));
                app->crtcs[app->crtc_count].id = lo->crtc;
                app->crtcs[app->crtc_count].edid_hash = lo->edid_hash;
                app->crtcs[app->crtc_count].active = 1;
                (void)snprintf(app->crtcs[app->crtc_count].output,
                               sizeof(app->crtcs[app->crtc_count].output), "%s", lo->name);
                app->crtcs[app->crtc_count].original = gamma;
                app->crtcs[app->crtc_count].state = app->state;
                app->crtc_count++;
                app->table_generation++;
                pending_remove(app, (size_t)pending);
                managed++;
                continue;
            }
        }

        gamma = XRRGetCrtcGamma(app->dpy, lo->crtc);
        if (gamma == NULL) continue;
        if (gamma->size != lo->gamma_size) {
            XRRFreeGamma(gamma);
            continue;
        }
        memset(&app->crtcs[app->crtc_count], 0, sizeof(app->crtcs[0]));
        app->crtcs[app->crtc_count].id = lo->crtc;
        app->crtcs[app->crtc_count].edid_hash = lo->edid_hash;
        app->crtcs[app->crtc_count].active = 1;
        (void)snprintf(app->crtcs[app->crtc_count].output,
                       sizeof(app->crtcs[app->crtc_count].output), "%s", lo->name);
        app->crtcs[app->crtc_count].original = gamma;
        app->crtcs[app->crtc_count].state = app->state;
        app->crtc_count++;
        app->table_generation++;
        managed++;
    }

    return managed;
}

static size_t active_crtc_count(const App *app)
{
    size_t i, n = 0U;
    for (i = 0U; i < app->crtc_count; ++i) {
        if (app->crtcs[i].active) n++;
    }
    return n;
}

static int capture_original_crtcs(App *app)
{
    free_saved_crtcs(app);
    return crtc_table_sync(app) > 0 ? 0 : -1;
}

/* fsync the directory so the rename itself survives a power loss. */
static void fsync_parent_dir(const char *path)
{
    char dir[PATH_MAX];
    char *slash;
    int fd;

    if (snprintf(dir, sizeof(dir), "%s", path) < 0) return;
    slash = strrchr(dir, '/');
    if (slash == NULL) return;
    if (slash == dir) dir[1] = '\0';
    else *slash = '\0';
    fd = open(dir, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (fd < 0) return;
    (void)fsync(fd);
    (void)close(fd);
}

static int write_ramp_record(FILE *fp, uint64_t crtc, uint64_t edid, const char *output,
                             uint32_t size, const unsigned short *r,
                             const unsigned short *g, const unsigned short *b)
{
    StateCrtcHeader ch;

    memset(&ch, 0, sizeof(ch));
    ch.crtc = crtc;
    ch.edid_hash = edid;
    ch.size = size;
    (void)snprintf(ch.output, sizeof(ch.output), "%s", output != NULL ? output : "");
    if (fwrite(&ch, sizeof(ch), 1U, fp) != 1U) return -1;
    if (fwrite(r, sizeof(unsigned short), size, fp) != size) return -1;
    if (fwrite(g, sizeof(unsigned short), size, fp) != size) return -1;
    if (fwrite(b, sizeof(unsigned short), size, fp) != size) return -1;
    return 0;
}

/*
 * Write the recovery journal atomically: temp file, fsync, rename, fsync of
 * the directory. It carries the pristine ramp of every managed output plus
 * any ramp still owed to a monitor that is currently disconnected, so an
 * unclean exit can always be undone completely.
 */
/* Is this device already recorded as changed by us (so its live "original"
 * supersedes any inherited pending record)? */
static int find_backlight_changed(const App *app, const char *name)
{
    size_t i;
    for (i = 0U; i < app->backlight_count; ++i) {
        if (app->backlights[i].changed &&
            strcmp(app->backlights[i].name, name) == 0) return 1;
    }
    return 0;
}

static int save_recovery_file(const App *app)
{
    char tmp_path[PATH_MAX];
    FILE *fp;
    StateHeader header;
    uint32_t bl_count = 0U;
    uint32_t bl_dropped = 0U;
    uint32_t written;
    size_t ramp_count;
    size_t pending_written;
    size_t i;
    int fd;
    int n;

    /*
     * A journal that cannot be identified with this boot would be refused by
     * the loader, so writing one would be worse than useless: it would look
     * like a valid safety net while being unusable. Say so instead.
     */
    if (boot_hash() == 0U && !app->runtime_is_volatile) return -1;

    n = snprintf(tmp_path, sizeof(tmp_path), "%s.tmp.%lu", app->recovery_path,
                 (unsigned long)getpid());
    if (n < 0 || (size_t)n >= sizeof(tmp_path)) return -1;

    (void)unlink(tmp_path);
    fd = open(tmp_path, O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC,
              S_IRUSR | S_IWUSR);
    if (fd < 0) return -1;
    fp = fdopen(fd, "wb");
    if (fp == NULL) {
        (void)close(fd);
        (void)unlink(tmp_path);
        return -1;
    }

    /*
     * Only devices this process actually changed are recorded, plus levels a
     * previous instance still owes. Recovery must never "restore" a backlight
     * PhosTint never touched - the user may have set it with another tool.
     */
    for (i = 0U; i < app->backlight_count; ++i) {
        if (app->backlights[i].changed) bl_count++;
    }
    for (i = 0U; i < app->pending_bl_count; ++i) {
        if (find_backlight_changed(app, app->pending_bl[i].name)) continue;
        bl_count++;
    }
    if (bl_count > MAX_JOURNAL_BACKLIGHTS) {
        bl_dropped = bl_count - MAX_JOURNAL_BACKLIGHTS;
        bl_count = MAX_JOURNAL_BACKLIGHTS;
    }

    /* The loader refuses a count above the limit, so the writer must never
     * produce one: drop the oldest pending ramps rather than emit a journal
     * this program would later reject. */
    ramp_count = app->crtc_count + app->pending_count;
    pending_written = app->pending_count;
    if (ramp_count > MAX_JOURNAL_RAMPS || bl_dropped > 0U) {
        /*
         * Fail closed. Writing a journal that silently omits recovery data
         * would hand the caller a safety net with a hole in it; every caller
         * treats a write failure as "do not touch the hardware", which is the
         * only correct response. The capacity above makes this unreachable in
         * practice - it exists so the guarantee holds even if it were not.
         */
        (void)fclose(fp);
        (void)unlink(tmp_path);
        return -1;
    }

    memset(&header, 0, sizeof(header));
    memcpy(header.magic, STATE_MAGIC, sizeof(header.magic));
    header.version = STATE_VERSION;
    header.count = (uint32_t)ramp_count;
    header.boot_hash = boot_hash();
    header.backlight_count = bl_count;

    if (fwrite(&header, sizeof(header), 1U, fp) != 1U) goto fail;
    for (i = 0U; i < app->crtc_count && i < ramp_count; ++i) {
        XRRCrtcGamma *g = app->crtcs[i].original;
        if (g == NULL || g->size <= 0) goto fail;
        if (write_ramp_record(fp, (uint64_t)app->crtcs[i].id, app->crtcs[i].edid_hash,
                              app->crtcs[i].output, (uint32_t)g->size,
                              g->red, g->green, g->blue) != 0) goto fail;
    }
    for (i = 0U; i < pending_written; ++i) {
        const StateRamp *pr = &app->pending[i];
        if (write_ramp_record(fp, pr->crtc, pr->edid_hash, pr->output, pr->size,
                              pr->red, pr->green, pr->blue) != 0) goto fail;
    }

    written = 0U;
    for (i = 0U; i < app->backlight_count && written < bl_count; ++i) {
        StateBacklight rec;
        const Backlight *bl = &app->backlights[i];
        if (!bl->changed) continue;
        memset(&rec, 0, sizeof(rec));
        (void)snprintf(rec.name, sizeof(rec.name), "%s", bl->name);
        rec.original = (uint64_t)bl->original;
        rec.maximum = (uint64_t)bl->maximum;
        if (fwrite(&rec, sizeof(rec), 1U, fp) != 1U) goto fail;
        written++;
    }
    for (i = 0U; i < app->pending_bl_count && written < bl_count; ++i) {
        if (find_backlight_changed(app, app->pending_bl[i].name)) continue;
        if (fwrite(&app->pending_bl[i], sizeof(StateBacklight), 1U, fp) != 1U) goto fail;
        written++;
    }

    if (fflush(fp) != 0) goto fail;
    if (fsync(fileno(fp)) != 0) goto fail;
    if (fclose(fp) != 0) {
        (void)unlink(tmp_path);
        return -1;
    }
    if (rename(tmp_path, app->recovery_path) != 0) {
        (void)unlink(tmp_path);
        return -1;
    }
    fsync_parent_dir(app->recovery_path);
    return 0;

fail:
    (void)fclose(fp);
    (void)unlink(tmp_path);
    return -1;
}

/*
 * Open the recovery file for reading only when it is a regular file owned by
 * the current user. Symlinks are rejected so a compromised or shared
 * directory can never redirect the read (nor the trust) elsewhere.
 */
/*
 * Sets *absent to 1 only when the file genuinely does not exist. Any other
 * failure - no permission, a symlink where a regular file must be, an I/O
 * error - means a journal may exist and simply could not be read, which is a
 * completely different situation from "there is nothing to recover".
 */
static FILE *open_state_for_read(const char *path, int *absent)
{
    int fd;
    struct stat st;
    FILE *fp;

    if (absent != NULL) *absent = 0;
    fd = open(path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) {
        if (absent != NULL && errno == ENOENT) *absent = 1;
        return NULL;
    }
    if (fstat(fd, &st) != 0 || !S_ISREG(st.st_mode) || st.st_uid != getuid()) {
        (void)close(fd);
        return NULL;
    }
    fp = fdopen(fd, "rb");
    if (fp == NULL) (void)close(fd);
    return fp;
}

static void state_free(StateFile *st)
{
    size_t i;
    if (st == NULL) return;
    for (i = 0U; i < st->ramp_count; ++i) {
        free(st->ramps[i].red);
        free(st->ramps[i].green);
        free(st->ramps[i].blue);
        st->ramps[i].red = NULL;
        st->ramps[i].green = NULL;
        st->ramps[i].blue = NULL;
    }
    st->ramp_count = 0U;
    st->backlight_count = 0U;
}

/*
 * Parse and fully validate a recovery file into memory. Nothing is applied
 * here and no X call is made, which is what makes the "validate everything
 * first, then apply" guarantee possible - and lets the self-test exercise
 * every corruption path without a display.
 *
 * Returns 0 on a completely valid file, 1 when no file exists, -1 when the
 * file exists but is unusable (bad magic/version, truncated, out-of-range
 * lengths, or trailing garbage).
 */
static int state_load_ex(const char *path, StateFile *out, int allow_unknown_boot)
{
    FILE *fp;
    StateHeader header;
    uint32_t i;
    char extra;
    int absent = 0;

    memset(out, 0, sizeof(*out));
    fp = open_state_for_read(path, &absent);
    if (fp == NULL) return absent ? 1 : -1;

    if (fread(&header, sizeof(header), 1U, fp) != 1U ||
        memcmp(header.magic, STATE_MAGIC, sizeof(header.magic)) != 0 ||
        header.version != STATE_VERSION ||
        header.count > MAX_JOURNAL_RAMPS ||
        header.backlight_count > MAX_JOURNAL_BACKLIGHTS) {
        goto bad;
    }
    /*
     * A journal from a previous boot describes hardware state that no longer
     * exists (the /tmp fallback directory outlives reboots). Refuse it, and
     * refuse a journal whose boot could not be identified at all.
     */
    if (header.boot_hash == 0U) {
        /* Only acceptable in a directory the system clears between sessions,
         * where a journal from a previous boot cannot survive anyway. */
        if (!allow_unknown_boot) goto bad;
    } else if (header.boot_hash != boot_hash()) {
        goto bad;
    }

    for (i = 0U; i < header.count; ++i) {
        StateCrtcHeader ch;
        StateRamp *ramp = &out->ramps[out->ramp_count];

        if (fread(&ch, sizeof(ch), 1U, fp) != 1U || ch.size == 0U || ch.size > 65536U) goto bad;
        if (memchr(ch.output, '\0', sizeof(ch.output)) == NULL) goto bad;
        ramp->crtc = ch.crtc;
        ramp->edid_hash = ch.edid_hash;
        ramp->size = ch.size;
        (void)snprintf(ramp->output, sizeof(ramp->output), "%s", ch.output);
        ramp->red = malloc((size_t)ch.size * sizeof(unsigned short));
        ramp->green = malloc((size_t)ch.size * sizeof(unsigned short));
        ramp->blue = malloc((size_t)ch.size * sizeof(unsigned short));
        if (ramp->red == NULL || ramp->green == NULL || ramp->blue == NULL) {
            free(ramp->red);
            free(ramp->green);
            free(ramp->blue);
            ramp->red = ramp->green = ramp->blue = NULL;
            goto bad;
        }
        out->ramp_count++;
        if (fread(ramp->red, sizeof(unsigned short), ch.size, fp) != ch.size ||
            fread(ramp->green, sizeof(unsigned short), ch.size, fp) != ch.size ||
            fread(ramp->blue, sizeof(unsigned short), ch.size, fp) != ch.size) {
            goto bad;
        }
    }

    for (i = 0U; i < header.backlight_count; ++i) {
        if (fread(&out->backlights[i], sizeof(StateBacklight), 1U, fp) != 1U) goto bad;
        if (memchr(out->backlights[i].name, '\0',
                   sizeof(out->backlights[i].name)) == NULL) goto bad;
        out->backlight_count++;
    }

    /*
     * The file must end exactly here. A short read is only acceptable when it
     * is a real end-of-file: an I/O error must never be mistaken for one.
     */
    if (fread(&extra, 1U, 1U, fp) != 0U) goto bad;
    if (ferror(fp) != 0 || feof(fp) == 0) goto bad;

    (void)fclose(fp);
    return 0;

bad:
    (void)fclose(fp);
    state_free(out);
    return -1;
}

static int state_load(const char *path, StateFile *out)
{
    return state_load_ex(path, out, 0);
}

/*
 * Apply a validated recovery file. dpy may be NULL for backlight-only
 * recovery when the X server is unreachable.
 *
 * Returns the number of restored CRTCs (0 when no usable file exists) and
 * sets *skipped to the number of ramps that could not be applied because the
 * CRTC is gone, disabled, or now has a different gamma size. -1 means the
 * file exists but is corrupt: nothing at all was applied.
 */
/*
 * Apply a validated journal. Records are matched to live outputs by physical
 * identity (EDID, then connector name, then CRTC id as a last resort), never
 * by CRTC id alone: the X server reassigns those freely across replugs.
 *
 * On return, st holds the parsed journal with `applied` set on the records
 * that reached the hardware; the caller owns it and must call state_free().
 * Returns 0 when the journal was usable, 1 when there was none, -1 when it
 * was corrupt (in which case nothing at all was applied).
 */
static int recovery_apply(Display *dpy, Window root, const char *path,
                          StateFile *st, RecoveryResult *out, int allow_unknown_boot)
{
    LiveOutput live[MAX_SAVED_CRTCS];
    size_t nlive = 0U;
    size_t i, l;
    int rc;

    memset(out, 0, sizeof(*out));
    rc = state_load_ex(path, st, allow_unknown_boot);
    if (rc != 0) return rc;

    if (dpy != NULL) {
        int n = enumerate_live_outputs(dpy, root, live, MAX_SAVED_CRTCS);
        /*
         * A failed enumeration leaves nlive at 0, so every record stays
         * unmatched and is carried forward as pending. That is the safe
         * outcome: the journal is kept and nothing is applied blindly.
         */
        nlive = (n > 0) ? (size_t)n : 0U;
        g_x_error_count = 0;
    }

    /*
     * Match records to outputs strictly one-to-one, strongest evidence first.
     *
     * The pass order matters as much as the one-to-one rule. Two monitors of
     * the same model report the same EDID hash, so matching on EDID alone
     * would pair them in array order and could swap their ramps. Requiring
     * EDID *and* connector first pins the unambiguous pairs; only then is a
     * weaker key allowed to place whatever is left.
     */
    {
        int claimed[MAX_SAVED_CRTCS];
        int pass;

        for (l = 0U; l < nlive; ++l) claimed[l] = 0;

        for (pass = 0; pass < 4; ++pass) {
            for (i = 0U; i < st->ramp_count; ++i) {
                StateRamp *ramp = &st->ramps[i];
                int has_edid = (ramp->edid_hash != 0U);
                if (ramp->applied) continue;
                for (l = 0U; l < nlive; ++l) {
                    int edid_eq, name_eq;
                    int hit = 0;
                    if (claimed[l]) continue;
                    if (live[l].gamma_size != (int)ramp->size) continue;
                    edid_eq = (has_edid && live[l].edid_hash != 0U &&
                               ramp->edid_hash == live[l].edid_hash);
                    name_eq = (ramp->output[0] != '\0' && live[l].name[0] != '\0' &&
                               strcmp(ramp->output, live[l].name) == 0);
                    switch (pass) {
                    case 0:  /* same panel, same port: unambiguous */
                        hit = edid_eq && name_eq;
                        break;
                    case 1:  /* same panel, moved to another port */
                        hit = edid_eq;
                        break;
                    case 2:  /* same port, and EDID cannot contradict it */
                        hit = name_eq && !(has_edid && live[l].edid_hash != 0U);
                        break;
                    default: /* nothing but the old CRTC id to go on */
                        hit = (ramp->crtc == (uint64_t)live[l].crtc &&
                               ramp->output[0] == '\0' && !has_edid);
                        break;
                    }
                    if (!hit) continue;
                    {
                        XRRCrtcGamma *g = XRRAllocGamma((int)ramp->size);
                        if (g == NULL) break;   /* leave unmatched; try nothing else */
                        memcpy(g->red, ramp->red, (size_t)ramp->size * sizeof(unsigned short));
                        memcpy(g->green, ramp->green, (size_t)ramp->size * sizeof(unsigned short));
                        memcpy(g->blue, ramp->blue, (size_t)ramp->size * sizeof(unsigned short));
                        XRRSetCrtcGamma(dpy, live[l].crtc, g);
                        XRRFreeGamma(g);
                    }
                    ramp->applied = 1;
                    claimed[l] = 1;
                    out->ramps_restored++;
                    break;
                }
            }
        }
        for (i = 0U; i < st->ramp_count; ++i) {
            if (!st->ramps[i].applied) out->ramps_unmatched++;
        }
    }

    /* Gamma requests are asynchronous: only a round-trip proves they landed. */
    if (dpy != NULL) {
        XSync(dpy, False);
        out->x_errors = g_x_error_count;
        if (out->x_errors > 0) {
            for (i = 0U; i < st->ramp_count; ++i) st->ramps[i].applied = 0;
            out->ramps_unmatched += out->ramps_restored;
            out->ramps_restored = 0;
        }
    }

    for (i = 0U; i < st->backlight_count; ++i) {
        int r = restore_backlight_record(&st->backlights[i]);
        st->backlight_applied[i] = (r > 0) ? 1 : 0;
        if (r > 0) out->backlights_restored++;
        else out->backlights_failed++;
    }
    return 0;
}

/* True when the journal has served its purpose and may be deleted. */
static int recovery_complete(const RecoveryResult *r)
{
    return r->ramps_unmatched == 0 && r->backlights_failed == 0 && r->x_errors == 0;
}

/* Returns the number of CRTCs the server actually accepted; *errors_out gets
 * the number it rejected. */
static int set_identity_gamma(Display *dpy, Window root, int *errors_out)
{
    XRRScreenResources *res;
    int i;
    int changed = 0;

    if (errors_out != NULL) *errors_out = 0;
    g_x_error_count = 0;
    res = XRRGetScreenResourcesCurrent(dpy, root);
    if (res == NULL) return -1;
    for (i = 0; i < res->ncrtc; ++i) {
        XRRCrtcInfo *info = XRRGetCrtcInfo(dpy, res, res->crtcs[i]);
        int size, j;
        XRRCrtcGamma *g;
        if (info == NULL) continue;
        if (info->mode == None || info->noutput <= 0) {
            XRRFreeCrtcInfo(info);
            continue;
        }
        XRRFreeCrtcInfo(info);
        size = XRRGetCrtcGammaSize(dpy, res->crtcs[i]);
        if (size <= 0 || size > 65536) continue;
        g = XRRAllocGamma(size);
        if (g == NULL) continue;
        for (j = 0; j < size; ++j) {
            double x = size == 1 ? 1.0 : (double)j / (double)(size - 1);
            unsigned short v = (unsigned short)llround(clamp_double(x, 0.0, 1.0) * 65535.0);
            g->red[j] = v;
            g->green[j] = v;
            g->blue[j] = v;
        }
        XRRSetCrtcGamma(dpy, res->crtcs[i], g);
        XRRFreeGamma(g);
        changed++;
    }
    XRRFreeScreenResources(res);
    XSync(dpy, False);   /* only now can the server's verdict be trusted */
    if (g_x_error_count > 0) {
        if (errors_out != NULL) *errors_out = g_x_error_count;
        return changed - g_x_error_count > 0 ? changed - g_x_error_count : 0;
    }
    return changed;
}

static void reset_color_state(ColorState *s)
{
    s->r = 1.0;
    s->g = 1.0;
    s->b = 1.0;
    s->strength = 1.0;
    s->brightness = 1.0;
    s->blue_limit = 1.0;
    s->kelvin = 0.0;
    (void)snprintf(s->mode, sizeof(s->mode), "%s", "normal");
    s->modified = 0;
}

/*
 * Final per-channel multipliers for one look. Blue is additionally scaled by
 * the blue ceiling, which is what lets blue-limit compose with every mode.
 */
static void state_factors(const ColorState *s, double *fr, double *fg, double *fb)
{
    double strength = clamp_double(s->strength, 0.0, 1.0);
    double brightness = clamp_double(s->brightness, 0.01, 1.0);
    double blue_limit = clamp_double(s->blue_limit, 0.0, 1.0);

    *fr = brightness * ((1.0 - strength) + strength * clamp_double(s->r, 0.0, 1.0));
    *fg = brightness * ((1.0 - strength) + strength * clamp_double(s->g, 0.0, 1.0));
    *fb = brightness * ((1.0 - strength) + strength * clamp_double(s->b, 0.0, 1.0)) * blue_limit;
}

static int any_state_modified(const App *app)
{
    size_t i;
    if (app->state.modified) return 1;
    for (i = 0U; i < app->crtc_count; ++i) {
        if (app->crtcs[i].state.modified) return 1;
    }
    return 0;
}

/*
 * Push every managed output's look to the hardware.
 *
 * The table is re-synchronized first, so a monitor that appeared without a
 * RandR event can never be silently left untouched. Success means: at least
 * one active output was programmed, no active output had to be skipped, and
 * the X server reported no error for the batch (verified with XSync before
 * reading the error counter). Anything else returns -1 and records why.
 */
static int apply_color_state(App *app)
{
    XRRCrtcGamma *staged[MAX_SAVED_CRTCS];
    size_t index[MAX_SAVED_CRTCS];
    size_t staged_count = 0U;
    size_t c, k;
    unsigned long before;
    int skipped = 0;
    int rc = 0;

    before = app->table_generation;
    if (crtc_table_sync(app) < 0) {
        app_warn(app, "the RandR topology could not be read; nothing was applied");
        return -1;
    }
    /*
     * The sync may have adopted, replaced or dropped an output since the last
     * journal write. Any pristine ramp must be on disk before a tint goes on
     * top of it, or a crash in between would make that tint the new
     * "original". The generation counter catches a same-count swap, which a
     * count comparison would miss entirely.
     */
    if (app->table_generation != before && save_recovery_file(app) != 0) {
        app_warn(app, "the new output set could not be journalled; nothing was applied");
        return -1;
    }

    /*
     * Phase 1 - build every ramp and check every precondition. Nothing has
     * touched the hardware yet, so any problem here aborts with the screen
     * untouched instead of leaving half the monitors converted.
     */
    for (c = 0U; c < app->crtc_count; ++c) {
        XRRCrtcGamma *base = app->crtcs[c].original;
        XRRCrtcGamma *out;
        double fr, fg, fb;
        int i;

        if (!app->crtcs[c].active) continue;   /* output is off: nothing to program */
        if (base == NULL || base->size <= 0) {
            skipped++;
            app_warn(app, "output %s: no pristine ramp is held for it", app->crtcs[c].output);
            continue;
        }
        out = XRRAllocGamma(base->size);
        if (out == NULL) {
            skipped++;
            app_warn(app, "output %s: out of memory building the ramp", app->crtcs[c].output);
            continue;
        }
        state_factors(&app->crtcs[c].state, &fr, &fg, &fb);
        for (i = 0; i < base->size; ++i) {
            double rv = (double)base->red[i] * fr;
            double gv = (double)base->green[i] * fg;
            double bv = (double)base->blue[i] * fb;
            out->red[i] = (unsigned short)llround(clamp_double(rv, 0.0, 65535.0));
            out->green[i] = (unsigned short)llround(clamp_double(gv, 0.0, 65535.0));
            out->blue[i] = (unsigned short)llround(clamp_double(bv, 0.0, 65535.0));
        }
        staged[staged_count] = out;
        index[staged_count] = c;
        staged_count++;
    }

    if (skipped > 0) {
        for (k = 0U; k < staged_count; ++k) XRRFreeGamma(staged[k]);
        return -1;   /* all-or-nothing: not one monitor was changed */
    }
    if (staged_count == 0U) {
        app_warn(app, "no active output is available to program");
        return -1;
    }

    /* Phase 2 - commit the whole batch, then one round-trip to verify it. */
    g_x_error_count = 0;
    for (k = 0U; k < staged_count; ++k) {
        XRRSetCrtcGamma(app->dpy, app->crtcs[index[k]].id, staged[k]);
    }
    XSync(app->dpy, False);
    app->next_reassert_ms = now_ms() + REASSERT_MS;

    /*
     * Phase 3 - if the server rejected any of it, put the hardware back to
     * the pristine ramps. The caller is responsible for restoring the
     * previous *look* on top of that, so the logical state and the screen
     * never disagree.
     */
    if (g_x_error_count > 0) {
        int errors = g_x_error_count;
        g_x_error_count = 0;
        for (k = 0U; k < staged_count; ++k) {
            XRRCrtcGamma *base = app->crtcs[index[k]].original;
            if (base != NULL) XRRSetCrtcGamma(app->dpy, app->crtcs[index[k]].id, base);
        }
        XSync(app->dpy, False);
        if (g_x_error_count > 0) {
            /* The rollback itself was refused: the screen may be left in the
             * rejected state, and only an explicit reset can fix that. */
            app_warn(app, "the X server rejected %d gamma request(s) AND %d of the rollback; "
                          "run 'normal', or 'emergency-reset' if the screen looks wrong",
                     errors, g_x_error_count);
        } else {
            app_warn(app, "the X server rejected %d gamma request(s); the batch was rolled back",
                     errors);
        }
        rc = -1;
    }

    for (k = 0U; k < staged_count; ++k) XRRFreeGamma(staged[k]);
    return rc;
}

/*
 * Put every changed backlight back, and retry any level inherited from a
 * previous instance. Returns the number that could NOT be put back; those
 * stay recorded so the journal keeps owing them.
 */
static int restore_backlights(App *app)
{
    size_t i;
    int failed = 0;

    for (i = 0U; i < app->backlight_count; ++i) {
        Backlight *bl = &app->backlights[i];
        if (!bl->changed) continue;
        if (bl->writable && write_long_file(bl->brightness_path, bl->original) == 0) {
            bl->changed = 0;
        } else {
            failed++;
        }
    }

    i = 0U;
    while (i < app->pending_bl_count) {
        if (restore_backlight_record(&app->pending_bl[i]) > 0) {
            memmove(&app->pending_bl[i], &app->pending_bl[i + 1U],
                    (app->pending_bl_count - i - 1U) * sizeof(app->pending_bl[0]));
            app->pending_bl_count--;
        } else {
            failed++;
            ++i;
        }
    }
    return failed;
}

/*
 * Put everything back exactly as captured. Returns 0 only when every active
 * output and every changed backlight device was restored; otherwise -1 with
 * the reason in app->last_warning, so callers never claim a clean restore
 * they did not achieve.
 */
static int restore_original(App *app)
{
    size_t i;
    int synced;
    int restored = 0;
    int skipped = 0;
    int bl_failed;

    /* Re-sync first: a monitor that appeared without an event must still be
     * restored, and one that vanished must not be counted as a failure. */
    synced = crtc_table_sync(app);

    g_x_error_count = 0;
    for (i = 0U; i < app->crtc_count; ++i) {
        XRRCrtcGamma *g = app->crtcs[i].original;
        if (!app->crtcs[i].active) continue;
        if (g == NULL || g->size <= 0) {
            skipped++;
            continue;
        }
        XRRSetCrtcGamma(app->dpy, app->crtcs[i].id, g);
        restored++;
    }
    XSync(app->dpy, False);

    noise_destroy(app);
    bl_failed = restore_backlights(app);

    /*
     * The logical state may only be declared "normal" once the hardware
     * really is. Resetting it after a failed restore would leave status
     * claiming an untinted screen while the screen is still tinted.
     */
    if (synced < 0) {
        app_warn(app, "could not read the RandR topology while restoring");
        return -1;
    }
    if (g_x_error_count > 0) {
        app_warn(app, "the X server rejected %d restore request(s)", g_x_error_count);
        return -1;
    }
    if (skipped > 0) {
        app_warn(app, "%d active output(s) had no pristine ramp to restore", skipped);
        return -1;
    }

    reset_color_state(&app->state);
    for (i = 0U; i < app->crtc_count; ++i) reset_color_state(&app->crtcs[i].state);
    app->target = -1;
    (void)restored;   /* zero is legitimate: every managed output may be off */

    /*
     * The journal must stop claiming that a backlight still needs restoring;
     * otherwise a later crash could replay a level the user has since changed
     * by hand.
     */
    if (save_recovery_file(app) != 0) {
        app_warn(app, "the recovery journal could not be refreshed after restoring");
        return -1;
    }
    if (bl_failed > 0) {
        app_warn(app, "%d backlight device(s) could not be restored", bl_failed);
        return -1;
    }
    return 0;
}

/*
 * The X connection died (server exit, session logout). Gamma state dies with
 * the server, but sysfs backlight changes persist and must still be undone.
 * Xlib requires that an IO error handler never returns.
 */
static int xio_error_handler(Display *dpy)
{
    (void)dpy;
    if (g_app != NULL) {
        int failed = restore_backlights(g_app);
        (void)unlink(g_app->socket_path);
        /*
         * The saved gamma ramps describe a server that no longer exists, and
         * the backlights have just been put back. Leaving the journal behind
         * would let a later run replay stale levels over whatever the user
         * has since set. Keep it only while something is still owed.
         */
        if (failed == 0 && g_app->pending_count == 0U && g_app->pending_bl_count == 0U) {
            (void)unlink(g_app->recovery_path);
        } else {
            g_app->crtc_count = 0U;   /* ramps are meaningless without the server */
            (void)save_recovery_file(g_app);
        }
    }
    _exit(EXIT_RUNTIME);
}

/*
 * A RandR topology change arrived (hotplug, mode switch, CRTC re-assignment).
 * Re-sync the table, persist the new pristine set, and re-assert the looks.
 */
static void topology_resync(App *app)
{
    if (crtc_table_sync(app) < 0) {
        app_warn(app, "a topology change could not be read from the X server");
        return;
    }
    /*
     * Fail closed: if the pristine ramps of the new configuration cannot be
     * journalled, do not tint on top of them. An un-journalled tint is a tint
     * that a crash could make permanent, which is exactly the failure this
     * program exists to prevent. One retry first, since the usual cause is a
     * transient ENOSPC.
     */
    if (save_recovery_file(app) != 0 && save_recovery_file(app) != 0) {
        app_warn(app, "recovery journal not writable after a topology change; "
                      "the look was NOT re-applied (run 'normal' then retry)");
        noise_resize(app);
        return;
    }
    if (any_state_modified(app)) (void)apply_color_state(app);
    noise_resize(app);
}

/*
 * Drive exactly one backlight device: the named one, or the preferred one.
 * A successful change is persisted to the recovery file immediately, so a
 * crash right after this point still restores the user's original level.
 */
static int set_backlight_percent(App *app, double percent, const char *device,
                                 char *response, size_t response_size)
{
    Backlight *bl;
    long target;
    double p = clamp_double(percent, 1.0, 100.0) / 100.0;

    if (device != NULL) {
        bl = find_backlight(app, device);
        if (bl == NULL) {
            (void)snprintf(response, response_size,
                           "No backlight device named '%s'. Use 'list' to see the available devices.",
                           device);
            return -1;
        }
        if (!bl->writable) {
            (void)snprintf(response, response_size,
                           "Backlight device '%s' is not writable by this user.", device);
            return -1;
        }
    } else {
        bl = preferred_backlight(app);
        if (bl == NULL) {
            (void)snprintf(response, response_size,
                           "Hardware backlight is not writable for this user. "
                           "No privilege changes were attempted; software brightness remains available.");
            return -1;
        }
    }

    /*
     * Refresh what "original" means before the FIRST change we make to this
     * device: the user (or the firmware, on an AC/battery transition) may have
     * moved it since the daemon started, and restoring a stale level would
     * undo their change rather than ours.
     */
    if (!bl->changed) {
        size_t pi;
        int inherited = 0;

        /*
         * A level still owed by a previous instance outranks whatever is on
         * the device right now: it is the value from before PhosTint ever
         * touched this backlight. Adopting it (and taking the debt over)
         * keeps that oldest original alive even if the user moved the
         * brightness in the meantime.
         */
        for (pi = 0U; pi < app->pending_bl_count; ++pi) {
            if (strcmp(app->pending_bl[pi].name, bl->name) != 0) continue;
            if (app->pending_bl[pi].original > (uint64_t)LONG_MAX ||
                app->pending_bl[pi].maximum > (uint64_t)LONG_MAX) break;
            if ((long)app->pending_bl[pi].maximum == bl->maximum) {
                bl->original = (long)app->pending_bl[pi].original;
                inherited = 1;
            }
            memmove(&app->pending_bl[pi], &app->pending_bl[pi + 1U],
                    (app->pending_bl_count - pi - 1U) * sizeof(app->pending_bl[0]));
            app->pending_bl_count--;
            break;
        }

        if (!inherited) {
            long current;
            long current_max;
            char max_path[PATH_MAX];
            char device_dir[PATH_MAX];

            if (snprintf(device_dir, sizeof(device_dir), "%s/%s",
                         g_backlight_root, bl->name) > 0 &&
                path_join(max_path, sizeof(max_path), device_dir, "max_brightness") == 0 &&
                read_long_file(max_path, &current_max) == 0 && current_max > 0) {
                bl->maximum = current_max;
            }
            if (read_long_file(bl->brightness_path, &current) == 0 && current >= 0) {
                bl->original = current;
            }
        }
    }

    target = (long)llround((double)bl->maximum * p);
    if (target < 1L) target = 1L;
    if (target > bl->maximum) target = bl->maximum;

    /*
     * Write-ahead: the journal must describe the level we are about to leave
     * behind BEFORE the hardware changes, otherwise a crash in between loses
     * the original for good.
     */
    bl->changed = 1;
    if (save_recovery_file(app) != 0) {
        bl->changed = 0;
        (void)snprintf(response, response_size,
                       "Refusing to change the backlight: the recovery journal could not be "
                       "written first, so the current level could not be made recoverable.");
        return -1;
    }
    if (write_long_file(bl->brightness_path, target) != 0) {
        int saved_errno = errno;
        bl->changed = 0;
        if (save_recovery_file(app) != 0) {
            /* The journal still claims a change that never happened; say so
             * rather than leave a stale entry to be replayed later. */
            (void)snprintf(response, response_size,
                           "Writing the backlight device '%s' failed (%s), and the recovery "
                           "journal could not be cleaned up. Run 'normal' to resynchronize.",
                           bl->name, strerror(saved_errno));
            return -1;
        }
        (void)snprintf(response, response_size,
                       "Writing the backlight device '%s' failed: %s.",
                       bl->name, strerror(saved_errno));
        return -1;
    }
    (void)snprintf(response, response_size,
                   "Hardware backlight '%s' (%s) set to %.0f%% (%ld/%ld).",
                   bl->name, backlight_type_name(bl->type), percent, target, bl->maximum);
    return 0;
}

static int parse_number(const char *s, double lo, double hi, double *out)
{
    char *end = NULL;
    double v;
    if (s == NULL || out == NULL) return -1;
    errno = 0;
    v = strtod(s, &end);
    if (errno != 0 || end == s || *end != '\0' || !isfinite(v) || v < lo || v > hi) return -1;
    *out = v;
    return 0;
}

static int hex_value(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c = tolower((unsigned char)c);
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static int parse_hex_color(const char *s, double *r, double *g, double *b)
{
    int vals[6];
    int i;
    double rr, gg, bb, maxv;
    if (s == NULL || r == NULL || g == NULL || b == NULL) return -1;
    if (*s == '#') ++s;
    if (strlen(s) != 6U) return -1;
    for (i = 0; i < 6; ++i) {
        vals[i] = hex_value((unsigned char)s[i]);
        if (vals[i] < 0) return -1;
    }
    rr = (double)(vals[0] * 16 + vals[1]) / 255.0;
    gg = (double)(vals[2] * 16 + vals[3]) / 255.0;
    bb = (double)(vals[4] * 16 + vals[5]) / 255.0;
    maxv = fmax(rr, fmax(gg, bb));
    if (maxv <= 0.0) return -1;
    /* Treat hex as hue/tint; brightness is controlled separately. */
    *r = rr / maxv;
    *g = gg / maxv;
    *b = bb / maxv;
    return 0;
}

/*
 * Approximate blackbody white point for a correlated color temperature,
 * based on the widely used curve fit published by Tanner Helland (2012).
 * Accurate to a few percent across 1000K..10000K, which is sufficient for
 * comfort tinting; it is not a colorimetric CCT conversion.
 *
 * The result is normalized so the strongest channel is exactly 1.0. That
 * keeps the white point from darkening the image more than necessary, but it
 * is NOT luminance preservation: a warm white point still lowers the panel's
 * measured luminance because the blue (and some green) energy is removed.
 * Use "brightness" for an explicit luminance control.
 */
static void kelvin_to_rgb(double kelvin, double *r, double *g, double *b)
{
    double t = clamp_double(kelvin, 1000.0, 10000.0) / 100.0;
    double red, green, blue, maxv;

    if (t <= 66.0) {
        red = 255.0;
        green = 99.4708025861 * log(t) - 161.1195681661;
    } else {
        red = 329.698727446 * pow(t - 60.0, -0.1332047592);
        green = 288.1221695283 * pow(t - 60.0, -0.0755148492);
    }
    if (t >= 66.0) {
        blue = 255.0;
    } else if (t <= 19.0) {
        blue = 0.0;
    } else {
        blue = 138.5177312231 * log(t - 10.0) - 305.0447927307;
    }

    red = clamp_double(red, 0.0, 255.0);
    green = clamp_double(green, 0.0, 255.0);
    blue = clamp_double(blue, 0.0, 255.0);
    maxv = fmax(red, fmax(green, blue));
    if (maxv <= 0.0) {
        *r = 1.0;
        *g = 1.0;
        *b = 1.0;
        return;
    }
    *r = red / maxv;
    *g = green / maxv;
    *b = blue / maxv;
}

static int set_preset(ColorState *s, const char *name)
{
    if (strcasecmp(name, "green") == 0) {
        s->r = 0.06; s->g = 1.00; s->b = 0.06; s->strength = 0.95;
    } else if (strcasecmp(name, "green-soft") == 0) {
        s->r = 0.25; s->g = 1.00; s->b = 0.25; s->strength = 0.75;
    } else if (strcasecmp(name, "amber") == 0) {
        s->r = 1.00; s->g = 0.52; s->b = 0.02; s->strength = 0.95;
    } else if (strcasecmp(name, "red") == 0) {
        s->r = 1.00; s->g = 0.04; s->b = 0.04; s->strength = 0.95;
    } else if (strcasecmp(name, "pink") == 0) {
        s->r = 1.00; s->g = 0.25; s->b = 0.70; s->strength = 0.85;
    } else if (strcasecmp(name, "sepia") == 0) {
        s->r = 1.00; s->g = 0.70; s->b = 0.40; s->strength = 0.70;
    } else if (strcasecmp(name, "warm") == 0) {
        s->r = 1.00; s->g = 0.78; s->b = 0.48; s->strength = 0.80;
    } else if (strcasecmp(name, "low-blue") == 0) {
        s->r = 1.00; s->g = 0.88; s->b = 0.20; s->strength = 1.00;
    } else if (strcasecmp(name, "ultra-low-blue") == 0) {
        s->r = 1.00; s->g = 0.75; s->b = 0.03; s->strength = 1.00;
    } else if (strcasecmp(name, "zero-blue") == 0) {
        s->r = 1.00; s->g = 0.75; s->b = 0.00; s->strength = 1.00;
    } else {
        return -1;
    }
    s->kelvin = 0.0;
    (void)snprintf(s->mode, sizeof(s->mode), "%s", name);
    s->modified = 1;
    return 0;
}

/*
 * Level of the device a bare "backlight" command would drive, in percent, or
 * -1 when there is none. It must be the *preferred* device, not merely the
 * first writable one, or status and the TUI would report a slider that no
 * command actually moves.
 */
static double preferred_backlight_percent(App *app)
{
    const Backlight *bl = preferred_backlight(app);
    long current;

    if (bl == NULL || bl->maximum <= 0) return -1.0;
    if (read_long_file(bl->brightness_path, &current) != 0) return -1.0;
    return 100.0 * (double)current / (double)bl->maximum;
}

static void status_text(App *app, char *response, size_t response_size)
{
    size_t used = 0U;
    size_t i;
    int writable = 0;
    int changed = 0;

    for (i = 0U; i < app->backlight_count; ++i) {
        if (app->backlights[i].writable) writable++;
        if (app->backlights[i].changed) changed++;
    }
    buf_append(response, response_size, &used,
               "mode=%s brightness=%.0f%% strength=%.0f%% rgb=%.0f/%.0f/%.0f%% "
               "blue_limit=%.0f%% noise=%d%% kelvin=%.0f backlight=%.0f%% "
               "active_crtcs=%zu managed_crtcs=%zu pending_ramps=%zu "
               "backlights=%zu writable_backlights=%d changed_backlights=%d "
               "pending_backlights=%zu",
               app->state.mode,
               app->state.brightness * 100.0,
               app->state.strength * 100.0,
               app->state.r * 100.0,
               app->state.g * 100.0,
               app->state.b * 100.0,
               app->state.blue_limit * 100.0,
               app->noise.intensity,
               app->state.kelvin,
               preferred_backlight_percent(app),
               active_crtc_count(app),
               app->crtc_count,
               app->pending_count,
               app->backlight_count,
               writable,
               changed,
               app->pending_bl_count);

    buf_append(response, response_size, &used, " outputs=");
    if (app->crtc_count == 0U) {
        buf_append(response, response_size, &used, "none");
    } else {
        for (i = 0U; i < app->crtc_count; ++i) {
            buf_append(response, response_size, &used, "%s%s%s:%s",
                       i == 0U ? "" : ",",
                       app->crtcs[i].active ? "" : "-",
                       app->crtcs[i].output, app->crtcs[i].state.mode);
        }
    }
    if (app->last_warning[0] != '\0') {
        buf_append(response, response_size, &used, " warning=\"%s\"", app->last_warning);
    }
    if (app->sticky_warning[0] != '\0') {
        buf_append(response, response_size, &used, " attention=\"%s\"", app->sticky_warning);
    }
}

static void list_text(App *app, char *response, size_t response_size)
{
    XRRScreenResources *res;
    size_t used = 0U;
    size_t i;
    int o;
    int printed = 0;

    buf_append(response, response_size, &used, "outputs:");
    res = XRRGetScreenResourcesCurrent(app->dpy, app->root);
    if (res != NULL) {
        for (o = 0; o < res->noutput; ++o) {
            XRROutputInfo *info = XRRGetOutputInfo(app->dpy, res, res->outputs[o]);
            if (info == NULL) continue;
            if (info->connection == RR_Connected) {
                int gsize = 0;
                if (info->crtc != None) {
                    gsize = XRRGetCrtcGammaSize(app->dpy, info->crtc);
                }
                buf_append(response, response_size, &used, " %s[%s gamma=%d]",
                           info->name != NULL ? info->name : "?",
                           info->crtc != None ? "active" : "off",
                           gsize);
                printed++;
            }
            XRRFreeOutputInfo(info);
        }
        XRRFreeScreenResources(res);
    }
    if (printed == 0) buf_append(response, response_size, &used, " none");

    buf_append(response, response_size, &used, " | backlights:");
    if (app->backlight_count == 0U) {
        buf_append(response, response_size, &used, " none");
        return;
    }
    {
        const Backlight *preferred = preferred_backlight(app);
        for (i = 0U; i < app->backlight_count; ++i) {
            Backlight *bl = &app->backlights[i];
            long current = -1L;
            int have = read_long_file(bl->brightness_path, &current) == 0;
            buf_append(response, response_size, &used, " %s%s[%s]=",
                       bl == preferred ? "*" : "", bl->name,
                       backlight_type_name(bl->type));
            if (have) buf_append(response, response_size, &used, "%ld/%ld", current, bl->maximum);
            else buf_append(response, response_size, &used, "?/%ld", bl->maximum);
            if (!bl->writable) buf_append(response, response_size, &used, "(read-only)");
        }
        if (preferred != NULL) {
            buf_append(response, response_size, &used, " | * = default target");
        }
    }
}

/* ==================== Command table ==================== */

/*
 * Every command declares its arity so extra or missing arguments are a hard
 * error with a usage hint, never something silently ignored. per_output
 * marks the commands that can be aimed at a single monitor.
 */
typedef struct {
    const char *name;
    int min_args;
    int max_args;
    int per_output;
    const char *usage;
} CommandSpec;

static const CommandSpec COMMANDS[] = {
    { "status",     0, 0, 0, "status" },
    { "list",       0, 0, 0, "list" },
    { "normal",     0, 0, 1, "normal" },
    { "stop",       0, 0, 0, "stop" },
    { "preset",     1, 1, 1, "preset NAME" },
    { "temp",       1, 1, 1, "temp KELVIN" },
    { "brightness", 1, 1, 1, "brightness PERCENT" },
    { "strength",   1, 1, 1, "strength PERCENT" },
    { "blue",       1, 1, 1, "blue PERCENT" },
    { "blue-limit", 1, 1, 1, "blue-limit PERCENT" },
    { "color",      1, 2, 1, "color RRGGBB [strength]" },
    { "rgb",        3, 4, 1, "rgb R G B [strength]" },
    { "backlight",  1, 2, 0, "backlight PERCENT [device]" },
    { "noise",      1, 1, 0, "noise PERCENT|off" }
};

static const CommandSpec *command_spec(const char *name)
{
    size_t i;
    if (name == NULL) return NULL;
    if (strcasecmp(name, "bluelimit") == 0) name = "blue-limit";
    for (i = 0U; i < sizeof(COMMANDS) / sizeof(COMMANDS[0]); ++i) {
        if (strcasecmp(COMMANDS[i].name, name) == 0) return &COMMANDS[i];
    }
    return NULL;
}

static int find_output_index(const App *app, const char *name)
{
    size_t i;
    for (i = 0U; i < app->crtc_count; ++i) {
        if (strcasecmp(app->crtcs[i].output, name) == 0) return (int)i;
    }
    return -1;
}

/*
 * Push the current look(s) to the hardware. A global change first copies the
 * global look onto every managed output; a targeted change leaves the other
 * outputs exactly as they are.
 */
/*
 * A copy of every look in effect, used to undo a failed change completely:
 * the hardware AND the logical state go back to what the user was actually
 * looking at, not to the untinted original.
 */
typedef struct {
    ColorState global;
    ColorState per_output[MAX_SAVED_CRTCS];
    RRCrtc ids[MAX_SAVED_CRTCS];
    size_t count;
} LookSnapshot;

static void look_save(const App *app, LookSnapshot *snap)
{
    size_t i;
    snap->global = app->state;
    snap->count = app->crtc_count;
    for (i = 0U; i < app->crtc_count; ++i) {
        snap->per_output[i] = app->crtcs[i].state;
        snap->ids[i] = app->crtcs[i].id;
    }
}

/* Restore the saved looks by CRTC id: the table may have been re-synced. */
static void look_restore(App *app, const LookSnapshot *snap)
{
    size_t i, j;
    app->state = snap->global;
    for (i = 0U; i < app->crtc_count; ++i) {
        app->crtcs[i].state = snap->global;
        for (j = 0U; j < snap->count; ++j) {
            if (snap->ids[j] == app->crtcs[i].id) {
                app->crtcs[i].state = snap->per_output[j];
                break;
            }
        }
    }
}

static int commit_look(App *app, int target)
{
    size_t i;
    if (target < 0) {
        for (i = 0U; i < app->crtc_count; ++i) app->crtcs[i].state = app->state;
    }
    return apply_color_state(app);
}

/*
 * Commit a change transactionally. On failure the previous look is put back
 * on the hardware and in memory, so a rejected command leaves absolutely no
 * trace - neither a half-changed screen nor a status line that lies.
 */
static int commit_or_rollback(App *app, int target, const LookSnapshot *snap)
{
    char first_failure[sizeof(app->last_warning)];

    if (commit_look(app, target) == 0) return 0;

    (void)snprintf(first_failure, sizeof(first_failure), "%s", app->last_warning);
    look_restore(app, snap);
    if (apply_color_state(app) != 0) {
        /* Both the change and the undo failed: the screen is sitting on the
         * pristine ramps while the user expected their previous look. Report
         * both problems rather than only the first. */
        app_warn(app, "%s; restoring the previous look also failed, "
                      "the outputs are at their captured ramps", first_failure);
    } else {
        app_warn(app, "%s", first_failure);
    }
    return -1;
}

static void mark_custom(ColorState *st)
{
    if (strcmp(st->mode, "normal") == 0) {
        (void)snprintf(st->mode, sizeof(st->mode), "%s", "custom");
    }
}

static int handle_command(App *app, const char *command, char *response, size_t response_size)
{
    char buf[MAX_COMMAND];
    char target_name[OUTPUT_NAME_MAX];
    LookSnapshot snap;
    char *tok[10];
    char **arg;
    size_t ntok = 0U;
    size_t nargs;
    size_t base = 0U;
    char *save = NULL;
    char *p;
    const CommandSpec *spec;
    ColorState *st;
    int target = -1;
    int overflow = 0;

    if (strlen(command) >= sizeof(buf)) {
        (void)snprintf(response, response_size, "Command too long.");
        return -1;
    }
    (void)snprintf(buf, sizeof(buf), "%s", command);
    /* One command per connection: anything after the first line is ignored. */
    buf[strcspn(buf, "\r\n")] = '\0';
    p = strtok_r(buf, " \t", &save);
    while (p != NULL) {
        if (ntok >= sizeof(tok) / sizeof(tok[0])) {
            overflow = 1;
            break;
        }
        tok[ntok++] = p;
        p = strtok_r(NULL, " \t", &save);
    }
    if (ntok == 0U) {
        (void)snprintf(response, response_size, "Empty command.");
        return -1;
    }

    /*
     * Warnings describe the operation that is starting now; a problem that
     * has since been resolved must not keep haunting status output. "status"
     * and "list" are read-only and keep whatever the last real command left.
     */
    if (strcasecmp(tok[0], "status") != 0 && strcasecmp(tok[0], "list") != 0) {
        app->last_warning[0] = '\0';
    }

    target_name[0] = '\0';
    if (strcasecmp(tok[0], "output") == 0) {
        if (ntok < 3U) {
            (void)snprintf(response, response_size,
                           "Usage: output NAME COMMAND [args]. Run 'list' to see connector names.");
            return -1;
        }
        target = find_output_index(app, tok[1]);
        if (target < 0) {
            size_t used = 0U;
            size_t i;
            buf_append(response, response_size, &used,
                       "No managed output named '%s'. Available:", tok[1]);
            if (app->crtc_count == 0U) buf_append(response, response_size, &used, " none");
            for (i = 0U; i < app->crtc_count; ++i) {
                buf_append(response, response_size, &used, " %s", app->crtcs[i].output);
            }
            return -1;
        }
        (void)snprintf(target_name, sizeof(target_name), "%s", app->crtcs[target].output);
        base = 2U;
    }

    spec = command_spec(tok[base]);
    if (spec == NULL) {
        (void)snprintf(response, response_size,
                       "Unknown command '%s'. Try: status, list, normal, preset, color, rgb, temp, "
                       "blue, blue-limit, strength, brightness, backlight, noise, stop; "
                       "prefix any of them with 'output NAME' to target one monitor.",
                       tok[base]);
        return -1;
    }
    nargs = ntok - base - 1U;
    if (overflow || nargs > (size_t)spec->max_args) {
        (void)snprintf(response, response_size, "Too many arguments. Usage: %s.", spec->usage);
        return -1;
    }
    if (nargs < (size_t)spec->min_args) {
        (void)snprintf(response, response_size, "Missing arguments. Usage: %s.", spec->usage);
        return -1;
    }
    if (target >= 0 && !spec->per_output) {
        (void)snprintf(response, response_size,
                       "'%s' affects the whole session and cannot be aimed at a single output.",
                       spec->name);
        return -1;
    }
    arg = &tok[base + 1U];
    st = (target < 0) ? &app->state : &app->crtcs[target].state;
    /* Snapshot before any mutation so a rejected command can be undone. */
    look_save(app, &snap);

    if (strcmp(spec->name, "status") == 0) {
        status_text(app, response, response_size);
        return 0;
    }
    if (strcmp(spec->name, "list") == 0) {
        list_text(app, response, response_size);
        return 0;
    }
    if (strcmp(spec->name, "normal") == 0) {
        if (target >= 0) {
            reset_color_state(st);
            if (commit_or_rollback(app, target, &snap) != 0) {
                (void)snprintf(response, response_size,
                               "Output %s could not be restored: %s.", target_name, app->last_warning);
                return -1;
            }
            (void)snprintf(response, response_size, "Output %s restored to its captured ramp.", target_name);
            return 0;
        }
        if (restore_original(app) != 0) {
            (void)snprintf(response, response_size, "Restoration incomplete: %s.", app->last_warning);
            return -1;
        }
        (void)snprintf(response, response_size,
                       "Original display state restored (ramps, backlight, noise overlay).");
        return 0;
    }
    if (strcmp(spec->name, "stop") == 0) {
        int ok = restore_original(app) == 0;
        g_stop_requested = 1;
        if (!ok) {
            (void)snprintf(response, response_size,
                           "Daemon stopping, but restoration was incomplete: %s. "
                           "Run 'phostint emergency-reset' if the screen still looks wrong.",
                           app->last_warning);
            return -1;
        }
        (void)snprintf(response, response_size, "Original display state restored. Daemon stopping.");
        return 0;
    }
    if (strcmp(spec->name, "preset") == 0) {
        if (strcasecmp(arg[0], "normal") == 0) {
            /* "preset normal" is an alias of "normal": a full session restore
             * (ramps, backlight, noise), or a single-output restore when it
             * is aimed at one monitor. */
            if (target < 0) return handle_command(app, "normal", response, response_size);
            reset_color_state(st);
            if (commit_or_rollback(app, target, &snap) != 0) {
                (void)snprintf(response, response_size,
                               "Output %s could not be restored: %s.", target_name,
                               app->last_warning);
                return -1;
            }
            (void)snprintf(response, response_size,
                           "Output %s restored to its captured ramp.", target_name);
            return 0;
        }
        if (set_preset(st, arg[0]) != 0) {
            (void)snprintf(response, response_size,
                           "Unknown preset '%s'. Use green, green-soft, amber, red, pink, sepia, "
                           "warm, low-blue, ultra-low-blue, zero-blue.", arg[0]);
            return -1;
        }
        if (commit_or_rollback(app, target, &snap) != 0) {
            (void)snprintf(response, response_size, "Preset selected but not fully applied: %s.",
                           app->last_warning);
            return -1;
        }
        (void)snprintf(response, response_size, "Preset '%s' applied%s%s.", arg[0],
                       target >= 0 ? " to output " : "", target >= 0 ? target_name : "");
        return 0;
    }
    if (strcmp(spec->name, "temp") == 0) {
        double kelvin;
        if (parse_number(arg[0], 1000.0, 10000.0, &kelvin) != 0) {
            (void)snprintf(response, response_size, "Color temperature must be 1000..10000 Kelvin.");
            return -1;
        }
        kelvin_to_rgb(kelvin, &st->r, &st->g, &st->b);
        st->strength = 1.0;
        st->kelvin = kelvin;
        st->modified = 1;
        (void)snprintf(st->mode, sizeof(st->mode), "temp-%.0fK", kelvin);
        if (commit_or_rollback(app, target, &snap) != 0) {
            (void)snprintf(response, response_size, "Temperature selected but not fully applied: %s.",
                           app->last_warning);
            return -1;
        }
        (void)snprintf(response, response_size,
                       "Approximate white point set to %.0f K (blackbody approximation).", kelvin);
        return 0;
    }
    if (strcmp(spec->name, "brightness") == 0) {
        double value;
        if (parse_number(arg[0], 1.0, 100.0, &value) != 0) {
            (void)snprintf(response, response_size, "Software brightness must be 1..100 percent.");
            return -1;
        }
        st->brightness = value / 100.0;
        st->modified = 1;
        mark_custom(st);
        if (commit_or_rollback(app, target, &snap) != 0) {
            (void)snprintf(response, response_size, "Brightness changed but not fully applied: %s.",
                           app->last_warning);
            return -1;
        }
        (void)snprintf(response, response_size, "Software brightness set to %.0f%%.", value);
        return 0;
    }
    if (strcmp(spec->name, "strength") == 0) {
        double value;
        if (parse_number(arg[0], 0.0, 100.0, &value) != 0) {
            (void)snprintf(response, response_size, "Tint strength must be 0..100 percent.");
            return -1;
        }
        st->strength = value / 100.0;
        st->modified = 1;
        mark_custom(st);
        if (commit_or_rollback(app, target, &snap) != 0) {
            (void)snprintf(response, response_size, "Strength changed but not fully applied: %s.",
                           app->last_warning);
            return -1;
        }
        (void)snprintf(response, response_size, "Tint strength set to %.0f%%.", value);
        return 0;
    }
    if (strcmp(spec->name, "blue") == 0) {
        double value;
        if (parse_number(arg[0], 0.0, 100.0, &value) != 0) {
            (void)snprintf(response, response_size, "Blue level must be 0..100 percent.");
            return -1;
        }
        st->r = 1.0;
        st->g = 1.0;
        st->b = value / 100.0;
        st->strength = 1.0;
        st->kelvin = 0.0;
        st->modified = 1;
        (void)snprintf(st->mode, sizeof(st->mode), "%s", "blue-control");
        if (commit_or_rollback(app, target, &snap) != 0) {
            (void)snprintf(response, response_size, "Blue level changed but not fully applied: %s.",
                           app->last_warning);
            return -1;
        }
        (void)snprintf(response, response_size,
                       "Digital blue-channel level set to %.0f%% (this does not guarantee zero "
                       "physical blue wavelengths).", value);
        return 0;
    }
    if (strcmp(spec->name, "blue-limit") == 0) {
        double value;
        if (parse_number(arg[0], 0.0, 100.0, &value) != 0) {
            (void)snprintf(response, response_size, "Blue limit must be 0..100 percent.");
            return -1;
        }
        st->blue_limit = value / 100.0;
        st->modified = 1;
        mark_custom(st);
        if (commit_or_rollback(app, target, &snap) != 0) {
            (void)snprintf(response, response_size, "Blue limit changed but not fully applied: %s.",
                           app->last_warning);
            return -1;
        }
        (void)snprintf(response, response_size,
                       "Blue-channel ceiling set to %.0f%%; it multiplies every preset, "
                       "temperature and tint.", value);
        return 0;
    }
    if (strcmp(spec->name, "color") == 0) {
        double r, g, b;
        double strength = 70.0;
        if (parse_hex_color(arg[0], &r, &g, &b) != 0) {
            (void)snprintf(response, response_size,
                           "Color must be a non-black RRGGBB hex value, e.g. 33FF66 or FF66CC.");
            return -1;
        }
        if (nargs >= 2U && parse_number(arg[1], 0.0, 100.0, &strength) != 0) {
            (void)snprintf(response, response_size, "Optional color strength must be 0..100 percent.");
            return -1;
        }
        st->r = r;
        st->g = g;
        st->b = b;
        st->strength = strength / 100.0;
        st->kelvin = 0.0;
        st->modified = 1;
        (void)snprintf(st->mode, sizeof(st->mode), "%s", "custom-color");
        if (commit_or_rollback(app, target, &snap) != 0) {
            (void)snprintf(response, response_size, "Color selected but not fully applied: %s.",
                           app->last_warning);
            return -1;
        }
        (void)snprintf(response, response_size, "Custom tint %s applied at %.0f%% strength.",
                       arg[0], strength);
        return 0;
    }
    if (strcmp(spec->name, "rgb") == 0) {
        double r, g, b, strength = 100.0;
        if (parse_number(arg[0], 0.0, 100.0, &r) != 0 ||
            parse_number(arg[1], 0.0, 100.0, &g) != 0 ||
            parse_number(arg[2], 0.0, 100.0, &b) != 0) {
            (void)snprintf(response, response_size, "RGB values must each be 0..100 percent.");
            return -1;
        }
        if (nargs >= 4U && parse_number(arg[3], 0.0, 100.0, &strength) != 0) {
            (void)snprintf(response, response_size, "Optional RGB strength must be 0..100 percent.");
            return -1;
        }
        if (r < 1.0 && g < 1.0 && b < 1.0) {
            (void)snprintf(response, response_size,
                           "At least one RGB channel must stay at 1%% or higher so the screen "
                           "remains visible.");
            return -1;
        }
        st->r = r / 100.0;
        st->g = g / 100.0;
        st->b = b / 100.0;
        st->strength = strength / 100.0;
        st->kelvin = 0.0;
        st->modified = 1;
        (void)snprintf(st->mode, sizeof(st->mode), "%s", "custom-rgb");
        if (commit_or_rollback(app, target, &snap) != 0) {
            (void)snprintf(response, response_size, "RGB selected but not fully applied: %s.",
                           app->last_warning);
            return -1;
        }
        (void)snprintf(response, response_size,
                       "RGB tint %.0f/%.0f/%.0f%% applied at %.0f%% strength.", r, g, b, strength);
        return 0;
    }
    if (strcmp(spec->name, "backlight") == 0) {
        double value;
        if (parse_number(arg[0], 1.0, 100.0, &value) != 0) {
            (void)snprintf(response, response_size, "Hardware backlight must be 1..100 percent.");
            return -1;
        }
        return set_backlight_percent(app, value, nargs >= 2U ? arg[1] : NULL,
                                     response, response_size);
    }
    if (strcmp(spec->name, "noise") == 0) {
        double value;
        char nerr[192];
        if (strcasecmp(arg[0], "off") == 0) {
            value = 0.0;
        } else if (parse_number(arg[0], 0.0, 100.0, &value) != 0) {
            (void)snprintf(response, response_size,
                           "Noise intensity must be 0..100 percent (0 or 'off' disables).");
            return -1;
        }
        if (noise_set(app, (int)llround(value), nerr, sizeof(nerr)) != 0) {
            (void)snprintf(response, response_size, "Noise unavailable: %s.", nerr);
            return -1;
        }
        if (value <= 0.0) {
            (void)snprintf(response, response_size, "Noise overlay disabled.");
        } else {
            (void)snprintf(response, response_size,
                           "Golden-ratio noise at %.0f%% (13 frames, 89 ms cadence, "
                           "server-side tiling).", value);
        }
        return 0;
    }

    (void)snprintf(response, response_size, "Command '%s' is not implemented.", spec->name);
    return -1;
}

static int create_server_socket(const char *path)
{
    int fd;
    struct sockaddr_un addr;
    size_t len = strlen(path);

    if (len >= sizeof(addr.sun_path)) return -1;
    fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    memcpy(addr.sun_path, path, len + 1U);
    (void)unlink(path);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        close(fd);
        return -1;
    }
    if (chmod(path, S_IRUSR | S_IWUSR) != 0) {
        close(fd);
        (void)unlink(path);
        return -1;
    }
    if (listen(fd, 8) != 0) {
        close(fd);
        (void)unlink(path);
        return -1;
    }
    return fd;
}

static int connect_to_daemon(const char *path)
{
    int fd;
    struct sockaddr_un addr;
    size_t len = strlen(path);

    if (len >= sizeof(addr.sun_path)) return -1;
    fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    memcpy(addr.sun_path, path, len + 1U);
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static int send_daemon_command(const char *socket_path, const char *command,
                               char *response, size_t response_size)
{
    int fd;
    size_t len;
    ssize_t n;
    size_t used = 0U;
    struct timeval tv;

    fd = connect_to_daemon(socket_path);
    if (fd < 0) return -1;
    /* A wedged daemon must never hang the CLI (or the TUI) forever. */
    tv.tv_sec = 10;
    tv.tv_usec = 0;
    (void)setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, (socklen_t)sizeof(tv));
    (void)setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, (socklen_t)sizeof(tv));
    len = strlen(command);
    if (write_all(fd, command, len) != 0 || write_all(fd, "\n", 1U) != 0) {
        (void)close(fd);
        return -1;
    }
    (void)shutdown(fd, SHUT_WR);

    while (used + 1U < response_size) {
        n = read(fd, response + used, response_size - used - 1U);
        if (n < 0) {
            if (errno == EINTR) continue;
            close(fd);
            return -1;
        }
        if (n == 0) break;
        used += (size_t)n;
    }
    response[used] = '\0';
    close(fd);
    return 0;
}

/*
 * Single-instance enforcement per (uid, DISPLAY). The fcntl lock disappears
 * automatically when the daemon process dies, so a crashed daemon can never
 * block a new one. The lock file itself is intentionally never unlinked.
 */
static int acquire_instance_lock(const char *path)
{
    int fd;
    struct flock fl;
    char buf[32];
    int n;

    fd = open(path, O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, S_IRUSR | S_IWUSR);
    if (fd < 0) return -1;
    memset(&fl, 0, sizeof(fl));
    fl.l_type = F_WRLCK;
    fl.l_whence = SEEK_SET;
    if (fcntl(fd, F_SETLK, &fl) != 0) {
        (void)close(fd);
        return -1;
    }
    n = snprintf(buf, sizeof(buf), "%ld\n", (long)getpid());
    if (n > 0 && (size_t)n < sizeof(buf)) {
        if (ftruncate(fd, 0) == 0) {
            (void)write_all(fd, buf, (size_t)n);
        }
    }
    return fd;
}

static const char *daemon_error_text(const char *code)
{
    if (strncmp(code, "EBusy", 5U) == 0)
        return "another PhosTint daemon already owns this display's instance lock";
    if (strncmp(code, "EX11", 4U) == 0)
        return "cannot open the X11 display (check DISPLAY; Wayland is not supported)";
    if (strncmp(code, "ERandR", 6U) == 0)
        return "the display server does not provide XRandR 1.2 or newer";
    if (strncmp(code, "EGamma", 6U) == 0)
        return "no active CRTC with readable gamma ramps was found";
    if (strncmp(code, "EState", 6U) == 0)
        return "could not write the private recovery state file";
    if (strncmp(code, "ESocket", 7U) == 0)
        return "could not create the private control socket";
    if (strncmp(code, "ESetsid", 7U) == 0 || strncmp(code, "EFork", 5U) == 0)
        return "could not detach the daemon process";
    if (strncmp(code, "Epaths", 6U) == 0)
        return "could not prepare the private runtime directory";
    if (strncmp(code, "ECorrupt", 8U) == 0)
        return "the recovery journal is unreadable, so the current ramps cannot be trusted as "
               "the baseline. Either reset to a neutral ramp with "
               "'emergency-reset --identity', or accept what is on screen now with "
               "'start --accept-current'";
    return "the daemon failed to initialize";
}

static void daemon_report(int notify_fd, const char *code)
{
    if (notify_fd >= 0) {
        (void)write_all(notify_fd, code, strlen(code));
    } else {
        fprintf(stderr, "%s: %s.\n", APP_NAME, daemon_error_text(code));
    }
}

static void set_error(char *err, size_t err_size, const char *msg)
{
    if (err != NULL && err_size > 0U) {
        (void)snprintf(err, err_size, "%s", msg);
    }
}

/* Clamp "deadline - now" into a poll timeout, keeping the smaller of the
 * running value and the new delta. */
static int deadline_delta(uint64_t deadline, uint64_t now, int current)
{
    uint64_t d = deadline > now ? deadline - now : 0U;
    int v = d > 3600000U ? 3600000 : (int)d;
    return (current < 0 || v < current) ? v : current;
}

/*
 * Drain queued X events. RandR notifications trigger one topology re-sync
 * after the queue is empty. A poll() wakeup with nothing queued forces a
 * round-trip so a dead X server is detected by the IO error handler.
 */
static void service_x_events(App *app, int woke_for_x)
{
    int topology_changed = 0;

    if (woke_for_x && XPending(app->dpy) == 0) {
        XSync(app->dpy, False);
    }
    while (XPending(app->dpy) > 0) {
        XEvent ev;
        XNextEvent(app->dpy, &ev);
        if (ev.type == app->randr_event_base + RRScreenChangeNotify) {
            (void)XRRUpdateConfiguration(&ev);
            topology_changed = 1;
        } else if (ev.type == app->randr_event_base + RRNotify) {
            topology_changed = 1;
        }
    }
    if (topology_changed) topology_resync(app);
}

/*
 * Control connections are serviced without ever blocking the event loop: a
 * client that connects and then goes silent must not stall the noise
 * animation, the periodic re-assert, or anybody else's command. Each
 * connection is non-blocking, lives in the same poll() set as everything
 * else, and is dropped when its deadline passes.
 */
typedef struct {
    int fd;
    char buf[MAX_COMMAND];
    size_t used;
    char out[MAX_RESPONSE + 8];   /* reply still to be flushed */
    size_t out_len;
    size_t out_sent;
    int draining;                 /* discard the rest of an over-long command */
    uint64_t deadline_ms;
} Client;

static void client_close(Client *c)
{
    if (c->fd >= 0) (void)close(c->fd);
    c->fd = -1;
    c->used = 0U;
    c->out_len = 0U;
    c->out_sent = 0U;
    c->draining = 0;
}

/*
 * Flush as much of the pending reply as the socket accepts. A non-blocking
 * write can transfer only part of the buffer, so the remainder is kept and
 * retried from the event loop: the client must never receive a truncated
 * answer to a command that actually ran.
 * Returns 1 when the reply is fully delivered (or the peer is gone).
 */
static int client_flush(Client *c)
{
    while (c->out_sent < c->out_len) {
        ssize_t n = write(c->fd, c->out + c->out_sent, c->out_len - c->out_sent);
        if (n > 0) {
            c->out_sent += (size_t)n;
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
        return 1;   /* peer gone: nothing more can be done */
    }
    return 1;
}

static void client_accept(App *app, int server_fd, Client *clients, size_t max_clients)
{
    int fd;
    size_t i;
    int flags;

    (void)app;
    fd = accept(server_fd, NULL, NULL);
    if (fd < 0) return;
    for (i = 0U; i < max_clients; ++i) {
        if (clients[i].fd < 0) break;
    }
    if (i == max_clients) {
        /* Backpressure instead of unbounded growth. */
        (void)write_all(fd, "ERR Too many concurrent connections.\n", 37U);
        (void)close(fd);
        return;
    }
    flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
        (void)close(fd);
        return;
    }
    clients[i].fd = fd;
    clients[i].used = 0U;
    clients[i].deadline_ms = now_ms() + CLIENT_TIMEOUT_MS;
}

/* Returns 1 when the connection is finished and was closed. */
static int client_service(App *app, Client *c)
{
    char response[MAX_RESPONSE];
    int complete = 0;
    int oversized = 0;
    int wn;
    int result;

    /*
     * Swallow the tail of a rejected over-long command before answering.
     * Closing the socket with unread data in flight makes the kernel send
     * RST, and the client would see "connection reset" instead of the
     * explanation it needs.
     */
    if (c->draining) {
        char sink[512];
        for (;;) {
            ssize_t n = read(c->fd, sink, sizeof(sink));
            if (n > 0) continue;
            if (n < 0 && errno == EINTR) continue;
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
            break;   /* EOF or error: the peer has finished */
        }
        c->draining = 0;
        if (client_flush(c)) {
            client_close(c);
            return 1;
        }
        return 0;
    }

    /* A reply already in flight just needs more room in the socket. */
    if (c->out_len > 0U) {
        if (client_flush(c)) {
            client_close(c);
            return 1;
        }
        return 0;
    }

    for (;;) {
        ssize_t n = read(c->fd, c->buf + c->used, sizeof(c->buf) - c->used - 1U);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
            client_close(c);
            return 1;
        }
        if (n == 0) {           /* peer finished writing */
            complete = 1;
            break;
        }
        c->used += (size_t)n;
        if (memchr(c->buf, '\n', c->used) != NULL) {
            complete = 1;
            break;
        }
        if (c->used + 1U >= sizeof(c->buf)) {
            /*
             * Refuse an over-long command outright. Executing the truncated
             * prefix would be far worse than an error: "brightness 100000..."
             * cut short becomes a perfectly valid, completely different
             * command.
             */
            oversized = 1;
            complete = 1;
            break;
        }
    }
    if (!complete) return 0;

    c->buf[c->used] = '\0';
    if (oversized) {
        (void)snprintf(response, sizeof(response),
                       "Command longer than %d bytes was refused (never truncated).",
                       (int)sizeof(c->buf) - 1);
        result = -1;
    } else {
        result = handle_command(app, c->buf, response, sizeof(response));
    }
    wn = snprintf(c->out, sizeof(c->out), "%s %s\n", result == 0 ? "OK" : "ERR", response);
    if (wn <= 0 || (size_t)wn >= sizeof(c->out)) {
        client_close(c);
        return 1;
    }
    c->out_len = (size_t)wn;
    c->out_sent = 0U;
    if (oversized) {
        c->draining = 1;   /* answer only after the peer stops writing */
        return 0;
    }
    if (client_flush(c)) {
        client_close(c);
        return 1;
    }
    /* Partially written: keep the connection until the rest fits or the
     * deadline passes. */
    return 0;
}

static int daemon_loop(int notify_fd, int accept_baseline)
{
    App app;
    int server_fd = -1;
    int randr_error_base = 0;
    int major = 0, minor = 0;
    int clean_restore;
    struct sigaction sa;
    struct pollfd pfd[2 + MAX_CLIENTS];
    Client clients[MAX_CLIENTS];
    size_t ci;

    memset(&app, 0, sizeof(app));
    app.lock_fd = -1;
    app.target = -1;
    reset_color_state(&app.state);
    g_app = &app;
    umask(0077);

    if (make_runtime_paths_ex(app.socket_path, sizeof(app.socket_path),
                              app.recovery_path, sizeof(app.recovery_path),
                              app.lock_path, sizeof(app.lock_path),
                              &app.runtime_is_volatile) != 0) {
        daemon_report(notify_fd, "Epaths\n");
        return EXIT_RUNTIME;
    }
    app.lock_fd = acquire_instance_lock(app.lock_path);
    if (app.lock_fd < 0) {
        daemon_report(notify_fd, "EBusy\n");
        return EXIT_RUNTIME;
    }

    app.dpy = XOpenDisplay(NULL);
    if (app.dpy == NULL) {
        daemon_report(notify_fd, "EX11\n");
        return EXIT_RUNTIME;
    }
    XSetErrorHandler(x_error_handler);
    XSetIOErrorHandler(xio_error_handler);
    app.root = DefaultRootWindow(app.dpy);
    if (!XRRQueryExtension(app.dpy, &app.randr_event_base, &randr_error_base) ||
        !XRRQueryVersion(app.dpy, &major, &minor) ||
        (major < 1 || (major == 1 && minor < 2))) {
        daemon_report(notify_fd, "ERandR\n");
        XCloseDisplay(app.dpy);
        return EXIT_RUNTIME;
    }
    XRRSelectInput(app.dpy, app.root,
                   RRScreenChangeNotifyMask | RRCrtcChangeNotifyMask |
                   RROutputChangeNotifyMask);

    /*
     * Recover exact pre-modification state after a prior unclean exit.
     *
     * Anything that could not be applied - typically a monitor that is
     * unplugged right now - is carried forward in memory and re-written to
     * the new journal, so replacing the journal below can never destroy a
     * pristine ramp that is still owed to some monitor.
     */
    {
        StateFile st;
        RecoveryResult rr;
        int rc = recovery_apply(app.dpy, app.root, app.recovery_path, &st, &rr,
                                app.runtime_is_volatile);

        if (rc < 0) {
            /*
             * A journal exists but cannot be read or parsed. Whatever the
             * previous instance did is unknown, so the ramps on screen right
             * now may already be tinted - capturing them as "original" would
             * silently make a tint permanent.
             *
             * Fail closed: preserve the file (never delete the only evidence)
             * and refuse to start. The user decides explicitly whether to
             * trust the current ramps or to reset to a neutral baseline.
             */
            char keep[PATH_MAX];
            int n = snprintf(keep, sizeof(keep), "%s.corrupt", app.recovery_path);
            if (n > 0 && (size_t)n < sizeof(keep)) {
                (void)rename(app.recovery_path, keep);  /* on failure: leave it in place */
            }
            if (!accept_baseline) {
                daemon_report(notify_fd, "ECorrupt\n");
                XCloseDisplay(app.dpy);
                return EXIT_RUNTIME;
            }
            (void)snprintf(app.sticky_warning, sizeof(app.sticky_warning), "%s",
                           "started with --accept-current after an unreadable journal: the "
                           "captured baseline is whatever was on screen, which may already "
                           "be tinted");
        } else if (rc == 0) {
            size_t k;
            size_t dropped = 0U;
            for (k = 0U; k < st.ramp_count; ++k) {
                if (st.ramps[k].applied) continue;
                if (pending_take(&app, &st.ramps[k]) != 0) dropped++;
            }
            for (k = 0U; k < st.backlight_count; ++k) {
                if (st.backlight_applied[k]) continue;
                if (app.pending_bl_count >= MAX_BACKLIGHTS) {
                    dropped++;
                    continue;
                }
                app.pending_bl[app.pending_bl_count++] = st.backlights[k];
            }
            if (dropped > 0U) {
                /* Only reachable with a hand-crafted journal, but losing
                 * recovery data is never something to pass over in silence. */
                (void)snprintf(app.sticky_warning, sizeof(app.sticky_warning),
                               "%zu recovery record(s) from the journal exceeded the in-memory "
                               "limits and were dropped", dropped);
            }
            if (rr.x_errors > 0) {
                app_warn(&app, "the X server rejected %d recovery request(s)", rr.x_errors);
            } else if (app.pending_count > 0U) {
                app_warn(&app, "%zu pristine ramp(s) are still owed to disconnected monitors",
                         app.pending_count);
            }
            if (rr.backlights_failed > 0) {
                app_warn(&app, "%d backlight value(s) from the journal are still owed",
                         rr.backlights_failed);
            }
            state_free(&st);
        }
    }

    if (capture_original_crtcs(&app) != 0) {
        daemon_report(notify_fd, "EGamma\n");
        XCloseDisplay(app.dpy);
        return EXIT_RUNTIME;
    }
    discover_backlights(&app);
    if (save_recovery_file(&app) != 0) {
        daemon_report(notify_fd, "EState\n");
        free_saved_crtcs(&app);
        XCloseDisplay(app.dpy);
        return EXIT_RUNTIME;
    }

    server_fd = create_server_socket(app.socket_path);
    if (server_fd < 0) {
        /* Keep the journal unless the rollback is proven complete. */
        if (restore_original(&app) == 0 && app.pending_count == 0U) {
            (void)unlink(app.recovery_path);
        }
        daemon_report(notify_fd, "ESocket\n");
        pending_clear(&app);
        free_saved_crtcs(&app);
        XCloseDisplay(app.dpy);
        return EXIT_RUNTIME;
    }

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    (void)sigaction(SIGINT, &sa, NULL);
    (void)sigaction(SIGTERM, &sa, NULL);
    (void)sigaction(SIGHUP, &sa, NULL);
    (void)sigaction(SIGQUIT, &sa, NULL);
    (void)signal(SIGPIPE, SIG_IGN);

    if (notify_fd >= 0) {
        (void)write_all(notify_fd, "OK\n", 3U);
        (void)close(notify_fd);
    } else {
        fprintf(stderr, "%s %s: running in the foreground; Ctrl+C restores and exits.\n",
                APP_NAME, APP_VERSION);
    }

    for (ci = 0U; ci < MAX_CLIENTS; ++ci) clients[ci].fd = -1;

    while (!g_stop_requested) {
        int pr;
        int timeout_ms = -1;
        uint64_t now;
        nfds_t nfds;
        size_t map[MAX_CLIENTS];

        service_x_events(&app, 0);
        if (g_stop_requested) break;

        /* Sleep exactly until the next deadline: a noise frame (89 ms while
         * the overlay is active) or the once-a-minute gamma re-assertion.
         * With neither pending the daemon blocks indefinitely - zero idle
         * CPU, fully event-driven. */
        now = now_ms();
        if (app.noise.intensity > 0) {
            timeout_ms = deadline_delta(app.noise.next_frame_ms, now, timeout_ms);
        }
        if (any_state_modified(&app)) {
            timeout_ms = deadline_delta(app.next_reassert_ms, now, timeout_ms);
        }

        pfd[0].fd = server_fd;
        pfd[0].events = POLLIN;
        pfd[0].revents = 0;
        pfd[1].fd = ConnectionNumber(app.dpy);
        pfd[1].events = POLLIN;
        pfd[1].revents = 0;
        nfds = 2U;
        for (ci = 0U; ci < MAX_CLIENTS; ++ci) {
            if (clients[ci].fd < 0) continue;
            map[nfds - 2U] = ci;
            pfd[nfds].fd = clients[ci].fd;
            pfd[nfds].events = (clients[ci].out_len > 0U && !clients[ci].draining)
                                   ? POLLOUT : POLLIN;
            pfd[nfds].revents = 0;
            timeout_ms = deadline_delta(clients[ci].deadline_ms, now, timeout_ms);
            nfds++;
        }

        pr = poll(pfd, nfds, timeout_ms);
        if (pr < 0) {
            if (errno == EINTR) continue;
            break;
        }
        now = now_ms();
        if (app.noise.intensity > 0 && now >= app.noise.next_frame_ms) {
            noise_frame_tick(&app);
        }
        if (any_state_modified(&app) && now >= app.next_reassert_ms) {
            (void)apply_color_state(&app);
        }
        if (pr > 0) {
            nfds_t k;
            if (pfd[1].revents != 0) service_x_events(&app, 1);
            for (k = 2U; k < nfds; ++k) {
                Client *cl = &clients[map[k - 2U]];
                if (cl->fd < 0) continue;
                if (pfd[k].revents & (POLLIN | POLLOUT | POLLHUP | POLLERR)) {
                    (void)client_service(&app, cl);
                }
            }
            if (pfd[0].revents & POLLIN) client_accept(&app, server_fd, clients, MAX_CLIENTS);
        }
        /* Drop connections that went silent past their deadline. */
        now = now_ms();
        for (ci = 0U; ci < MAX_CLIENTS; ++ci) {
            if (clients[ci].fd >= 0 && now >= clients[ci].deadline_ms) client_close(&clients[ci]);
        }
    }

    for (ci = 0U; ci < MAX_CLIENTS; ++ci) client_close(&clients[ci]);

    /*
     * The recovery file is the only way back if this restore did not fully
     * succeed, so it is removed only after a verified clean restoration.
     */
    clean_restore = restore_original(&app) == 0 && app.pending_count == 0U;
    (void)unlink(app.socket_path);
    if (clean_restore) {
        (void)unlink(app.recovery_path);
    } else if (notify_fd < 0) {
        fprintf(stderr, "%s: restoration incomplete (%s); the recovery file was kept. "
                        "Run '%s emergency-reset' if the display still looks wrong.\n",
                APP_NAME, app.last_warning, APP_NAME);
    }
    (void)close(server_fd);
    if (app.lock_fd >= 0) (void)close(app.lock_fd);
    pending_clear(&app);
    free_saved_crtcs(&app);
    g_app = NULL;
    XCloseDisplay(app.dpy);
    return clean_restore ? EXIT_OK : EXIT_RUNTIME;
}

static int start_daemon(const char *socket_path, char *err, size_t err_size,
                        int accept_baseline)
{
    int test_fd;
    int pipefd[2];
    pid_t pid;
    int status;
    char msg[64];
    ssize_t n;

    if (err != NULL && err_size > 0U) err[0] = '\0';

    test_fd = connect_to_daemon(socket_path);
    if (test_fd >= 0) {
        close(test_fd);
        return 0;
    }

    if (pipe(pipefd) != 0) {
        set_error(err, err_size, "could not create the startup pipe");
        return -1;
    }
    pid = fork();
    if (pid < 0) {
        close(pipefd[0]);
        close(pipefd[1]);
        set_error(err, err_size, "could not fork the daemon process");
        return -1;
    }
    if (pid == 0) {
        pid_t inner;
        close(pipefd[0]);
        if (setsid() < 0) {
            (void)write_all(pipefd[1], "ESetsid\n", 8U);
            _exit(EXIT_RUNTIME);
        }
        /*
         * Double fork: the daemon is re-parented to init immediately, so it
         * can never become a zombie of a long-lived caller (interactive
         * mode) and can never re-acquire a controlling terminal.
         */
        inner = fork();
        if (inner < 0) {
            (void)write_all(pipefd[1], "EFork\n", 6U);
            _exit(EXIT_RUNTIME);
        }
        if (inner > 0) _exit(EXIT_OK);
        {
            int nullfd = open("/dev/null", O_RDWR | O_CLOEXEC);
            if (nullfd >= 0) {
                (void)dup2(nullfd, STDIN_FILENO);
                (void)dup2(nullfd, STDOUT_FILENO);
                (void)dup2(nullfd, STDERR_FILENO);
                if (nullfd > STDERR_FILENO) close(nullfd);
            }
        }
        if (chdir("/") != 0) {
            /* Not fatal: the daemon only uses absolute paths anyway. */
        }
        _exit(daemon_loop(pipefd[1], accept_baseline));
    }

    close(pipefd[1]);
    while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}

    /*
     * Bounded wait: a daemon wedged before it reports readiness (an
     * unresponsive X server is the realistic case) must not hang the CLI.
     */
    {
        struct pollfd pp;
        uint64_t deadline = now_ms() + 15000U;
        for (;;) {
            uint64_t nowv = now_ms();
            int left = deadline > nowv ? (int)(deadline - nowv) : 0;
            pp.fd = pipefd[0];
            pp.events = POLLIN;
            pp.revents = 0;
            n = poll(&pp, 1U, left);
            if (n < 0 && errno == EINTR) continue;
            if (n <= 0) {
                close(pipefd[0]);
                set_error(err, err_size,
                          "the daemon did not report readiness within 15 seconds "
                          "(is the X server responding?)");
                return -1;
            }
            break;
        }
    }
    do {
        n = read(pipefd[0], msg, sizeof(msg) - 1U);
    } while (n < 0 && errno == EINTR);
    close(pipefd[0]);
    if (n <= 0) {
        set_error(err, err_size, "the daemon exited before reporting its status");
        return -1;
    }
    msg[n] = '\0';
    if (strncmp(msg, "OK", 2U) == 0) return 0;

    if (strncmp(msg, "EBusy", 5U) == 0) {
        /* Lost a startup race: wait briefly for the winner's socket. */
        struct timespec ts;
        int attempt;
        ts.tv_sec = 0;
        ts.tv_nsec = 50L * 1000000L;
        for (attempt = 0; attempt < 60; ++attempt) {
            int fd = connect_to_daemon(socket_path);
            if (fd >= 0) {
                (void)close(fd);
                return 0;
            }
            (void)nanosleep(&ts, NULL);
        }
        set_error(err, err_size,
                  "another instance holds the lock, but its control socket did not appear");
        return -1;
    }
    set_error(err, err_size, daemon_error_text(msg));
    return -1;
}

/* ==================== Full-screen terminal UI (plain VT100) ==================== */

/*
 * The TUI is a daemon client, like the CLI. It is implemented with raw
 * termios plus VT100 escape sequences only, so it works in every Linux
 * terminal without linking ncurses. ASCII fallbacks replace the UTF-8
 * glyphs automatically when the locale is not UTF-8.
 */

enum {
    TKEY_NONE = 0,
    TKEY_UP = 1000,
    TKEY_DOWN,
    TKEY_LEFT,
    TKEY_RIGHT,
    TKEY_PGUP,
    TKEY_PGDN
};

typedef enum {
    ROW_TEMP = 0,
    ROW_RED,
    ROW_GREEN,
    ROW_BLUE,
    ROW_STRENGTH,
    ROW_BLUE_LIMIT,
    ROW_BRIGHT,
    ROW_NOISE,
    ROW_BACKLIGHT,
    ROW_COUNT
} TuiRow;

typedef struct {
    double kelvin;
    double red;         /* Tint rows: the strength-resolved channel values, */
    double green;       /* ((1-s) + s*c) * 100. Editing one of them never   */
    double blue;        /* disturbs the other two. Brightness and the blue  */
    double blue_limit;  /* ceiling are applied on top (see tui_final_output).*/
    double brightness;
    double strength;
    double noise;
    double backlight;   /* -1 when no writable device exists */
    char mode[64];
} TuiVals;

typedef struct {
    struct termios saved;
    int saved_valid;
    int entered;
} TuiTerm;

static const struct {
    const char *label;
    double min, max, step, big;
} TUI_ROWS[ROW_COUNT] = {
    { "Temperature (K)",   1000.0, 10000.0, 100.0, 500.0 },
    { "Red tint",             0.0,   100.0,   5.0,  20.0 },
    { "Green tint",           0.0,   100.0,   5.0,  20.0 },
    { "Blue tint",            0.0,   100.0,   5.0,  20.0 },
    { "Tint strength",        0.0,   100.0,   5.0,  20.0 },
    { "Blue limit",           0.0,   100.0,   5.0,  20.0 },
    { "Brightness (soft)",    1.0,   100.0,   5.0,  20.0 },
    { "Noise",                0.0,   100.0,   5.0,  20.0 },
    { "Backlight (hw)",       1.0,   100.0,   5.0,  20.0 },
};

static TuiTerm *g_tui_term = NULL;
static volatile sig_atomic_t g_tui_winch = 0;
static volatile sig_atomic_t g_tui_resumed = 0;

static void on_winch(int sig)
{
    (void)sig;
    g_tui_winch = 1;
}

static void tui_screen_enter(void)
{
    (void)write_all(STDOUT_FILENO, "\x1b[?1049h\x1b[?25l\x1b[2J", 18U);
}

static void tui_screen_leave(void)
{
    (void)write_all(STDOUT_FILENO, "\x1b[?25h\x1b[?1049l", 14U);
}

static int tui_raw_enter(TuiTerm *t)
{
    struct termios raw;

    if (!t->saved_valid) {
        if (tcgetattr(STDIN_FILENO, &t->saved) != 0) return -1;
        t->saved_valid = 1;
    }
    raw = t->saved;
    raw.c_lflag &= ~(tcflag_t)(ICANON | ECHO);
    raw.c_iflag &= ~(tcflag_t)(IXON | ICRNL);
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) != 0) return -1;
    t->entered = 1;
    tui_screen_enter();
    return 0;
}

static void tui_raw_leave(TuiTerm *t)
{
    if (t == NULL || !t->entered) return;
    tui_screen_leave();
    if (t->saved_valid) (void)tcsetattr(STDIN_FILENO, TCSAFLUSH, &t->saved);
    t->entered = 0;
}

/*
 * Ctrl+Z. Handing a sane terminal back to the shell has to happen before we
 * actually stop, so it must be done here - but only with async-signal-safe
 * calls: write(2) and tcsetattr(3) both are. The handler is installed with
 * SA_RESETHAND, so re-raising reaches the default action and really stops
 * the process; the main loop re-arms it after SIGCONT.
 */
static void on_tstp(int sig)
{
    (void)sig;
    tui_raw_leave(g_tui_term);
    (void)raise(SIGTSTP);
}

/* Resuming only sets flags; the real work happens in the main loop. */
static void on_cont(int sig)
{
    (void)sig;
    g_tui_resumed = 1;
    g_tui_winch = 1;
}

static void tui_install_tstp(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_tstp;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = (int)SA_RESETHAND;   /* the macro is unsigned on glibc */
    (void)sigaction(SIGTSTP, &sa, NULL);
}

static int tui_read_key(void)
{
    unsigned char c;
    ssize_t n;

    n = read(STDIN_FILENO, &c, 1U);
    if (n == 0) return -1;
    if (n < 0) return errno == EINTR ? TKEY_NONE : -1;
    if (c != 0x1bU) return (int)c;

    /*
     * Consume the WHOLE escape sequence up to its final byte (0x40..0x7E for
     * CSI). Partial parsing would leak modifier parameters ("ESC[1;5C" for
     * Ctrl+Right) back into the key stream, where the '5' would fire a
     * preset out of nowhere.
     */
    {
        struct pollfd p;
        unsigned char b0 = 0U;
        unsigned char fin = 0U;
        unsigned char first_param = 0U;

        p.fd = STDIN_FILENO;
        p.events = POLLIN;
        p.revents = 0;
        if (poll(&p, 1U, 50) <= 0) return TKEY_NONE; /* lone ESC: ignore */
        if (read(STDIN_FILENO, &b0, 1U) != 1) return TKEY_NONE;
        if (b0 != '[' && b0 != 'O') return TKEY_NONE;
        for (;;) {
            unsigned char cb = 0U;
            p.revents = 0;
            if (poll(&p, 1U, 50) <= 0) return TKEY_NONE;
            if (read(STDIN_FILENO, &cb, 1U) != 1) return TKEY_NONE;
            if (cb >= 0x40U && cb <= 0x7eU) {
                fin = cb;
                break;
            }
            if (first_param == 0U) first_param = cb;
        }
        if (fin == 'A') return TKEY_UP;
        if (fin == 'B') return TKEY_DOWN;
        if (fin == 'C') return TKEY_RIGHT;
        if (fin == 'D') return TKEY_LEFT;
        if (fin == '~' && first_param == '5') return TKEY_PGUP;
        if (fin == '~' && first_param == '6') return TKEY_PGDN;
        return TKEY_NONE; /* anything else (Home, Del, F-keys...) is ignored whole */
    }
}

static int tui_find_kv(const char *resp, const char *key, double *out)
{
    const char *p = strstr(resp, key);
    char *end = NULL;
    double v;

    if (p == NULL) return -1;
    p += strlen(key);
    errno = 0;
    v = strtod(p, &end);
    if (end == p || errno != 0) return -1;
    *out = v;
    return 0;
}

static void tui_find_mode(const char *resp, char *mode, size_t size)
{
    const char *p = strstr(resp, "mode=");
    size_t i = 0U;

    if (size == 0U) return;
    if (p != NULL) {
        p += 5;
        while (*p != '\0' && *p != ' ' && i + 1U < size) mode[i++] = *p++;
    }
    mode[i] = '\0';
}

static int tui_fetch(const char *socket_path, TuiVals *v)
{
    char resp[MAX_RESPONSE];
    const char *p;
    char *end = NULL;
    double k = 0.0;
    double raw_r = 100.0, raw_g = 100.0, raw_b = 100.0;
    double s;

    if (send_daemon_command(socket_path, "status", resp, sizeof(resp)) != 0) return -1;
    if (strncmp(resp, "OK ", 3U) != 0) return -1;

    v->kelvin = 6500.0;
    v->blue_limit = 100.0;
    v->brightness = 100.0;
    v->strength = 100.0;
    v->noise = 0.0;
    v->backlight = -1.0;
    tui_find_mode(resp, v->mode, sizeof(v->mode));
    (void)tui_find_kv(resp, "brightness=", &v->brightness);
    (void)tui_find_kv(resp, "strength=", &v->strength);
    (void)tui_find_kv(resp, "blue_limit=", &v->blue_limit);
    (void)tui_find_kv(resp, "noise=", &v->noise);
    (void)tui_find_kv(resp, "backlight=", &v->backlight);
    (void)tui_find_kv(resp, "kelvin=", &k);
    if (k >= 1000.0) v->kelvin = k;

    p = strstr(resp, "rgb=");
    if (p != NULL) {
        double t;
        p += 4;
        t = strtod(p, &end);
        if (end != p && *end == '/') {
            raw_r = t;
            p = end + 1;
            t = strtod(p, &end);
            if (end != p && *end == '/') {
                raw_g = t;
                p = end + 1;
                t = strtod(p, &end);
                if (end != p) raw_b = t;
            }
        }
    }
    /* Convert state channels to the EFFECTIVE percentages the display gets:
     * eff = (1 - strength) + strength * channel. */
    s = clamp_double(v->strength / 100.0, 0.0, 1.0);
    v->red = ((1.0 - s) + s * clamp_double(raw_r / 100.0, 0.0, 1.0)) * 100.0;
    v->green = ((1.0 - s) + s * clamp_double(raw_g / 100.0, 0.0, 1.0)) * 100.0;
    v->blue = ((1.0 - s) + s * clamp_double(raw_b / 100.0, 0.0, 1.0)) * 100.0;
    return 0;
}

/*
 * What the panel actually receives per channel: tint, then brightness, then
 * the blue ceiling on blue. Shown as its own line so the tint rows are never
 * mistaken for the final output.
 */
static void tui_final_output(const TuiVals *v, double *r, double *g, double *b)
{
    double br = clamp_double(v->brightness, 1.0, 100.0) / 100.0;
    double bl = clamp_double(v->blue_limit, 0.0, 100.0) / 100.0;

    *r = clamp_double(v->red, 0.0, 100.0) * br;
    *g = clamp_double(v->green, 0.0, 100.0) * br;
    *b = clamp_double(v->blue, 0.0, 100.0) * br * bl;
}

static double tui_row_value(const TuiVals *v, int row)
{
    switch (row) {
    case ROW_TEMP: return v->kelvin;
    case ROW_RED: return v->red;
    case ROW_GREEN: return v->green;
    case ROW_BLUE: return v->blue;
    case ROW_STRENGTH: return v->strength;
    case ROW_BLUE_LIMIT: return v->blue_limit;
    case ROW_BRIGHT: return v->brightness;
    case ROW_NOISE: return v->noise;
    case ROW_BACKLIGHT: return v->backlight;
    default: return 0.0;
    }
}

/*
 * Editing an R/G/B row sends the full effective triple at 100% strength:
 * the two untouched channels keep their exact on-screen values, so the only
 * visible change is the channel the user is moving.
 */
static void tui_row_command(const TuiVals *v, int row, double value, char *cmd, size_t size)
{
    switch (row) {
    case ROW_TEMP: (void)snprintf(cmd, size, "temp %.0f", value); break;
    case ROW_RED:
        (void)snprintf(cmd, size, "rgb %.0f %.0f %.0f 100", value, v->green, v->blue);
        break;
    case ROW_GREEN:
        (void)snprintf(cmd, size, "rgb %.0f %.0f %.0f 100", v->red, value, v->blue);
        break;
    case ROW_BLUE:
        (void)snprintf(cmd, size, "rgb %.0f %.0f %.0f 100", v->red, v->green, value);
        break;
    case ROW_STRENGTH: (void)snprintf(cmd, size, "strength %.0f", value); break;
    case ROW_BLUE_LIMIT: (void)snprintf(cmd, size, "blue-limit %.0f", value); break;
    case ROW_BRIGHT: (void)snprintf(cmd, size, "brightness %.0f", value); break;
    case ROW_NOISE: (void)snprintf(cmd, size, "noise %.0f", value); break;
    case ROW_BACKLIGHT: (void)snprintf(cmd, size, "backlight %.0f", value); break;
    default: if (size > 0U) cmd[0] = '\0'; break;
    }
}

static void tui_send(const char *socket_path, const char *cmd, char *msg, size_t msg_size)
{
    char resp[MAX_RESPONSE];
    const char *p;

    if (send_daemon_command(socket_path, cmd, resp, sizeof(resp)) != 0) {
        /* The daemon may have been stopped externally: restart it once. */
        char err[256];
        if (start_daemon(socket_path, err, sizeof(err), 0) != 0 ||
            send_daemon_command(socket_path, cmd, resp, sizeof(resp)) != 0) {
            (void)snprintf(msg, msg_size, "%s", "Could not reach the daemon.");
            return;
        }
    }
    p = resp;
    if (strncmp(p, "OK ", 3U) == 0) p += 3;
    else if (strncmp(p, "ERR ", 4U) == 0) p += 4;
    /* The message line is one row tall: keep only the first 200 bytes. */
    (void)snprintf(msg, msg_size, "%.200s", p);
    msg[strcspn(msg, "\r\n")] = '\0';
}

/* Terminal geometry, with a sane default when the ioctl is unavailable. */
static void tui_size(int *cols, int *rows)
{
    struct winsize ws;

    *cols = 80;
    *rows = 24;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0) {
        if (ws.ws_col > 0) *cols = (int)ws.ws_col;
        if (ws.ws_row > 0) *rows = (int)ws.ws_row;
    }
}

#define TUI_MIN_COLS 62
#define TUI_FIXED_LINES 12   /* everything that is not a slider row */

static void tui_draw(const TuiVals *v, int sel_row, const char *msg, int utf8,
                     int bl_available, int cols, int rows)
{
    char out[8192];
    size_t used = 0U;
    const char *hz = utf8 ? "\xe2\x94\x80" : "-";       /* ─ */
    const char *fill = utf8 ? "\xe2\x96\x88" : "#";     /* █ */
    const char *empty = utf8 ? "\xe2\x96\x91" : ".";    /* ░ */
    const char *arrow = utf8 ? "\xe2\x86\x92" : ">";    /* → */
    int visible_rows = bl_available ? ROW_COUNT : ROW_COUNT - 1;
    int label_w = 18;
    int bar;
    int rule;
    int narrow;
    int budget;
    int show_keys, show_mode, show_panel, show_rules, show_blanks, show_presets;
    int max_sliders, first_slider, slot;
    int row, i;

    /*
     * Adapt instead of overflowing, in two dimensions.
     *
     * Width: the bar, the label column and the footer text shrink in steps so
     * no line is ever wider than `cols`.
     * Height: the header, the sliders and the message line are mandatory;
     * everything else is spent from a line budget in priority order, so the
     * panel never scrolls in a small window.
     */
    /*
     * A slider line costs 13 characters of chrome (marker, gaps, value), so
     * label + bar must fit in cols - 13. Below the minimum there is no honest
     * way to draw the panel and we say so instead of scribbling.
     */
    if (cols - 13 < 8 || rows < 4) {
        buf_append(out, sizeof(out), &used, "\x1b[H\x1b[2J%.*s\r\n",
                   cols > 1 ? cols - 1 : 1, "window too small");
        (void)write_all(STDOUT_FILENO, out, used);
        return;
    }
    if (cols >= 76) {
        bar = 24;
    } else if (cols >= 62) {
        bar = 16;
    } else if (cols >= 50) {
        bar = 10;
    } else if (cols >= 40) {
        bar = 8;
        label_w = 14;
    } else {
        int avail = cols - 13;
        label_w = (avail * 2) / 3;
        if (label_w > 18) label_w = 18;
        bar = avail - label_w;
        if (bar < 3) {
            bar = 3;
            label_w = avail - bar;
        }
    }
    narrow = (cols < 76) ? 1 : 0;
    rule = cols - 4;
    if (rule > 60) rule = 60;
    if (rule < 8) rule = 8;

    /*
     * The sliders themselves may not fit either. When they do not, show a
     * window of them around the selection rather than overflowing the screen.
     */
    max_sliders = rows - 2;                   /* header + message are mandatory */
    if (max_sliders < 1) max_sliders = 1;
    if (max_sliders > visible_rows) max_sliders = visible_rows;
    {
        int pos = 0;
        int seen = 0;
        for (row = 0; row < ROW_COUNT; ++row) {
            if (row == ROW_BACKLIGHT && !bl_available) continue;
            if (row == sel_row) pos = seen;
            seen++;
        }
        first_slider = pos - max_sliders / 2;
        if (first_slider > visible_rows - max_sliders) first_slider = visible_rows - max_sliders;
        if (first_slider < 0) first_slider = 0;
    }

    budget = rows - (1 + max_sliders + 1);    /* header + sliders + message */
    show_keys = budget >= 1;
    if (show_keys) budget -= 1;
    show_mode = budget >= 1;
    if (show_mode) budget -= 1;
    show_panel = budget >= 1;
    if (show_panel) budget -= 1;
    show_rules = budget >= 2;
    if (show_rules) budget -= 2;
    show_blanks = budget >= 2;
    if (show_blanks) budget -= 2;
    show_presets = (budget >= 2) && !narrow;

    buf_append(out, sizeof(out), &used, "\x1b[H");
    if (cols >= 50) {
        buf_append(out, sizeof(out), &used, " \x1b[1m%s %s\x1b[0m  all outputs\x1b[K\r\n",
                   APP_NAME, APP_VERSION);
    } else {
        buf_append(out, sizeof(out), &used, " \x1b[1m%s\x1b[0m %s\x1b[K\r\n",
                   APP_NAME, APP_VERSION);
    }
    if (show_rules) {
        buf_append(out, sizeof(out), &used, " ");
        for (i = 0; i < rule; ++i) buf_append(out, sizeof(out), &used, "%s", hz);
        buf_append(out, sizeof(out), &used, "\x1b[K\r\n");
    }
    if (show_mode) {
        buf_append(out, sizeof(out), &used, " Mode: %.*s\x1b[K\r\n",
                   cols > 10 ? cols - 8 : 2, v->mode);
    }
    if (show_blanks) buf_append(out, sizeof(out), &used, "\x1b[K\r\n");

    slot = 0;
    for (row = 0; row < ROW_COUNT; ++row) {
        double val, span;
        int filled;
        char valstr[32];

        if (row == ROW_BACKLIGHT && !bl_available) continue;
        if (slot < first_slider || slot >= first_slider + max_sliders) {
            slot++;
            continue;
        }
        slot++;
        val = tui_row_value(v, row);
        span = TUI_ROWS[row].max - TUI_ROWS[row].min;
        filled = (int)llround((val - TUI_ROWS[row].min) / span * (double)bar);
        if (filled < 0) filled = 0;
        if (filled > bar) filled = bar;

        if (row == ROW_TEMP) {
            (void)snprintf(valstr, sizeof(valstr), "%5.0f K", val);
        } else if (row == ROW_NOISE && val <= 0.0) {
            (void)snprintf(valstr, sizeof(valstr), "%s", "   off");
        } else {
            (void)snprintf(valstr, sizeof(valstr), "%4.0f %%", val);
        }

        buf_append(out, sizeof(out), &used, " %s %s%-*.*s ",
                   row == sel_row ? arrow : " ",
                   row == sel_row ? "\x1b[7m" : "",
                   label_w, label_w, TUI_ROWS[row].label);
        for (i = 0; i < bar; ++i) {
            buf_append(out, sizeof(out), &used, "%s", i < filled ? fill : empty);
        }
        buf_append(out, sizeof(out), &used, " %s%s\x1b[K\r\n",
                   valstr, row == sel_row ? "\x1b[0m" : "");
    }

    if (show_panel) {
        double fr, fg, fb;
        tui_final_output(v, &fr, &fg, &fb);
        if (narrow) {
            buf_append(out, sizeof(out), &used,
                       " Panel: R%.0f G%.0f B%.0f\x1b[K\r\n", fr, fg, fb);
        } else {
            buf_append(out, sizeof(out), &used,
                       "   Panel receives (tint x brightness x blue limit): "
                       "R %.0f%%  G %.0f%%  B %.0f%%\x1b[K\r\n", fr, fg, fb);
        }
    }

    if (show_presets) {
        buf_append(out, sizeof(out), &used,
                   " Presets: 1 green  2 green-soft  3 amber  4 red  5 pink\x1b[K\r\n");
        buf_append(out, sizeof(out), &used,
                   "          6 sepia  7 warm  8 low-blue  9 ultra-low  0 zero-blue\x1b[K\r\n");
    }
    if (show_keys) {
        if (cols >= 76) {
            buf_append(out, sizeof(out), &used,
                       " Keys: %s select  %s adjust  PgUp/PgDn coarse  0-9 preset  n normal"
                       "  q quit\x1b[K\r\n",
                       utf8 ? "\xe2\x86\x91\xe2\x86\x93" : "Up/Dn",
                       utf8 ? "\xe2\x86\x90\xe2\x86\x92" : "Lf/Rt");
        } else if (cols >= 62) {
            buf_append(out, sizeof(out), &used,
                       " Keys: arrows move/adjust  0-9 preset  n normal  q quit\x1b[K\r\n");
        } else if (cols >= 34) {
            buf_append(out, sizeof(out), &used, " arrows 0-9 n=normal q=quit\x1b[K\r\n");
        } else {
            buf_append(out, sizeof(out), &used, " 0-9 n q\x1b[K\r\n");
        }
    }
    if (show_rules) {
        buf_append(out, sizeof(out), &used, " ");
        for (i = 0; i < rule; ++i) buf_append(out, sizeof(out), &used, "%s", hz);
        buf_append(out, sizeof(out), &used, "\x1b[K\r\n");
    }
    buf_append(out, sizeof(out), &used, " %.*s\x1b[K\r\n",
               cols > 3 ? cols - 2 : 1, msg);
    buf_append(out, sizeof(out), &used, "\x1b[0J");
    (void)write_all(STDOUT_FILENO, out, used);
}

static int run_tui(const char *socket_path)
{
    TuiTerm term;
    TuiVals vals;
    struct sigaction sa;
    char err[256];
    char msg[256];
    const char *cs;
    int utf8;
    int sel = 0;

    if (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO)) {
        fprintf(stderr,
                "The TUI needs an interactive terminal ('phostint interactive' is the plain menu).\n");
        return EXIT_USAGE;
    }
    if (start_daemon(socket_path, err, sizeof(err), 0) != 0) {
        fprintf(stderr, "Could not start the PhosTint daemon: %s.\n", err);
        return EXIT_RUNTIME;
    }
    memset(&vals, 0, sizeof(vals));
    if (tui_fetch(socket_path, &vals) != 0) {
        fprintf(stderr, "Could not read the daemon status.\n");
        return EXIT_RUNTIME;
    }

    (void)setlocale(LC_CTYPE, "");
    cs = nl_langinfo(CODESET);
    utf8 = (cs != NULL && strcmp(cs, "UTF-8") == 0) ? 1 : 0;

    g_stop_requested = 0;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    (void)sigaction(SIGINT, &sa, NULL);
    (void)sigaction(SIGTERM, &sa, NULL);
    (void)sigaction(SIGHUP, &sa, NULL);
    (void)sigaction(SIGQUIT, &sa, NULL);
    sa.sa_handler = on_winch;
    (void)sigaction(SIGWINCH, &sa, NULL);
    sa.sa_handler = on_cont;
    (void)sigaction(SIGCONT, &sa, NULL);
    tui_install_tstp();

    memset(&term, 0, sizeof(term));
    g_tui_term = &term;
    if (tui_raw_enter(&term) != 0) {
        g_tui_term = NULL;
        fprintf(stderr, "Could not switch the terminal to raw mode.\n");
        return EXIT_RUNTIME;
    }
    (void)snprintf(msg, sizeof(msg), "%s",
                   "R/G/B rows are the live per-channel percentages. 'q' keeps the look.");

    for (;;) {
        int bl_available = vals.backlight >= 0.0 ? 1 : 0;
        int vis[ROW_COUNT];
        int nvis = 0;
        int key;
        int row;
        int cols, trows;

        for (row = 0; row < ROW_COUNT; ++row) {
            if (row == ROW_BACKLIGHT && !bl_available) continue;
            vis[nvis++] = row;
        }
        if (sel >= nvis) sel = nvis - 1;
        if (sel < 0) sel = 0;

        tui_size(&cols, &trows);
        tui_draw(&vals, vis[sel], msg, utf8, bl_available, cols, trows);
        key = tui_read_key();
        if (g_stop_requested || key == -1) break;
        if (g_tui_resumed) {
            /* Came back from Ctrl+Z: restore raw mode and re-arm the handler
             * here, in normal context, never inside the signal handler. */
            g_tui_resumed = 0;
            if (term.saved_valid) (void)tui_raw_enter(&term);
            tui_install_tstp();
            (void)tui_fetch(socket_path, &vals);
        }
        if (g_tui_winch) g_tui_winch = 0;

        if (key == 'q' || key == 'Q') break;
        if (key == TKEY_UP) {
            if (sel > 0) sel--;
        } else if (key == TKEY_DOWN) {
            if (sel + 1 < nvis) sel++;
        } else if (key == TKEY_LEFT || key == TKEY_RIGHT || key == TKEY_PGUP ||
                   key == TKEY_PGDN || key == '-' || key == '+' || key == '=') {
            int r2 = vis[sel];
            double step = (key == TKEY_PGUP || key == TKEY_PGDN)
                              ? TUI_ROWS[r2].big : TUI_ROWS[r2].step;
            double dir = (key == TKEY_LEFT || key == '-' || key == TKEY_PGDN) ? -1.0 : 1.0;
            double nv = clamp_double(tui_row_value(&vals, r2) + dir * step,
                                     TUI_ROWS[r2].min, TUI_ROWS[r2].max);
            char cmd[64];
            tui_row_command(&vals, r2, nv, cmd, sizeof(cmd));
            tui_send(socket_path, cmd, msg, sizeof(msg));
            (void)tui_fetch(socket_path, &vals);
        } else if (key >= '0' && key <= '9') {
            static const char *preset_names[10] = {
                "zero-blue", "green", "green-soft", "amber", "red",
                "pink", "sepia", "warm", "low-blue", "ultra-low-blue"
            };
            char cmd[64];
            (void)snprintf(cmd, sizeof(cmd), "preset %s", preset_names[key - '0']);
            tui_send(socket_path, cmd, msg, sizeof(msg));
            (void)tui_fetch(socket_path, &vals);
        } else if (key == 'n' || key == 'N') {
            tui_send(socket_path, "normal", msg, sizeof(msg));
            (void)tui_fetch(socket_path, &vals);
        } else if (key == 'r' || key == 'R') {
            if (tui_fetch(socket_path, &vals) == 0) {
                (void)snprintf(msg, sizeof(msg), "%s", "Status refreshed.");
            } else {
                (void)snprintf(msg, sizeof(msg), "%s", "Could not reach the daemon.");
            }
        }
    }

    tui_raw_leave(&term);
    g_tui_term = NULL;
    puts("PhosTint TUI closed; the daemon keeps the current look. Use 'phostint normal' to restore.");
    return EXIT_OK;
}

/*
 * Offline self-test of the pure computation and validation helpers.
 * Runs without any X server; intended for packagers and CI.
 */
/* Remove a flat directory of regular files (self-test scaffolding only). */
static int unlink_tree(const char *dir)
{
    DIR *d = opendir(dir);
    struct dirent *ent;
    char path[PATH_MAX];

    if (d == NULL) return -1;
    while ((ent = readdir(d)) != NULL) {
        if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) continue;
        if (path_join(path, sizeof(path), dir, ent->d_name) == 0) (void)unlink(path);
    }
    (void)closedir(d);
    return rmdir(dir);
}

static void check(int *failures, int condition, const char *name)
{
    if (condition) {
        printf("ok   %s\n", name);
    } else {
        printf("FAIL %s\n", name);
        (*failures)++;
    }
}

static int run_selftest(void)
{
    int failures = 0;
    double r, g, b;

    kelvin_to_rgb(6500.0, &r, &g, &b);
    check(&failures, r > 0.99 && g > 0.93 && b > 0.93, "kelvin 6500 is near neutral");
    kelvin_to_rgb(3400.0, &r, &g, &b);
    check(&failures, r == 1.0 && g > 0.70 && g < 0.79 && b > 0.48 && b < 0.58,
          "kelvin 3400 matches the blackbody curve fit");
    kelvin_to_rgb(1000.0, &r, &g, &b);
    check(&failures, r == 1.0 && b == 0.0 && g < 0.35, "kelvin 1000 has no blue");
    {
        double b2, b4, b65, tr, tg;
        kelvin_to_rgb(2000.0, &tr, &tg, &b2);
        kelvin_to_rgb(4000.0, &tr, &tg, &b4);
        kelvin_to_rgb(6500.0, &tr, &tg, &b65);
        check(&failures, b2 < b4 && b4 < b65, "blue rises monotonically with kelvin");
    }

    check(&failures, parse_hex_color("FF8000", &r, &g, &b) == 0 &&
          r == 1.0 && fabs(g - 128.0 / 255.0) < 1e-9 && b == 0.0,
          "hex color FF8000 parses as a normalized tint");
    check(&failures, parse_hex_color("#33FF66", &r, &g, &b) == 0 && g == 1.0,
          "hex color with # prefix parses");
    check(&failures, parse_hex_color("000000", &r, &g, &b) != 0, "black hex color is rejected");
    check(&failures, parse_hex_color("12345", &r, &g, &b) != 0, "short hex color is rejected");
    check(&failures, parse_hex_color("GGGGGG", &r, &g, &b) != 0, "non-hex color is rejected");

    {
        double v;
        check(&failures, parse_number("50", 0.0, 100.0, &v) == 0 && v == 50.0, "number in range parses");
        check(&failures, parse_number("101", 0.0, 100.0, &v) != 0, "number above range is rejected");
        check(&failures, parse_number("nan", 0.0, 100.0, &v) != 0, "nan is rejected");
        check(&failures, parse_number("", 0.0, 100.0, &v) != 0, "empty number is rejected");
        check(&failures, parse_number("5x", 0.0, 100.0, &v) != 0, "trailing junk is rejected");
    }

    {
        StateBacklight rec;
        memset(&rec, 0, sizeof(rec));
        (void)snprintf(rec.name, sizeof(rec.name), "%s", "../etc");
        rec.original = 1U;
        rec.maximum = 1U;
        check(&failures, restore_backlight_record(&rec) == 0, "path-traversal device name is rejected");
        (void)snprintf(rec.name, sizeof(rec.name), "%s", "a/b");
        check(&failures, restore_backlight_record(&rec) == 0, "device name with slash is rejected");
        memset(rec.name, 0, sizeof(rec.name));
        check(&failures, restore_backlight_record(&rec) == 0, "empty device name is rejected");
    }

    check(&failures, clamp_double(2.0, 0.0, 1.0) == 1.0 &&
          clamp_double(-1.0, 0.0, 1.0) == 0.0, "clamp bounds hold");

    {
        enum { NW = 89, NH = 55 };  /* Fibonacci test tile */
        static uint32_t buf_a[NW * NH];
        static uint32_t buf_b[NW * NH];
        size_t i, filled_full = 0U, filled_low = 0U, premult_bad = 0U;

        noise_fill_buffer(buf_a, NW, NH, 100, 12345U);
        noise_fill_buffer(buf_b, NW, NH, 100, 12345U);
        check(&failures, memcmp(buf_a, buf_b, sizeof(buf_a)) == 0,
              "noise is deterministic for a given seed");
        noise_fill_buffer(buf_b, NW, NH, 100, 54321U);
        check(&failures, memcmp(buf_a, buf_b, sizeof(buf_a)) != 0,
              "noise frames differ across seeds");
        for (i = 0U; i < (size_t)(NW * NH); ++i) {
            uint32_t a = buf_a[i] >> 24;
            if (buf_a[i] != 0U) filled_full++;
            if (((buf_a[i] >> 16) & 0xffU) > a ||
                ((buf_a[i] >> 8) & 0xffU) > a ||
                (buf_a[i] & 0xffU) > a) premult_bad++;
        }
        check(&failures, premult_bad == 0U,
              "noise pixels are premultiplied (channel never exceeds alpha)");
        check(&failures,
              filled_full > (size_t)(NW * NH) / 4 && filled_full < (size_t)(NW * NH) / 2,
              "noise density at 100 tracks 1/phi^2 (about 38 percent)");
        noise_fill_buffer(buf_b, NW, NH, 10, 12345U);
        for (i = 0U; i < (size_t)(NW * NH); ++i) {
            if (buf_b[i] != 0U) filled_low++;
        }
        check(&failures, filled_low < filled_full,
              "noise density scales down with intensity");
        check(&failures, golden_hash(1U) == golden_hash(1U) &&
              golden_hash(1U) != golden_hash(2U),
              "golden-ratio hash is stable and discriminating");
    }

    /* ---- command table: arity is enforced, not merely documented ---- */
    {
        const CommandSpec *s;
        check(&failures, command_spec("status") != NULL, "known command resolves");
        check(&failures, command_spec("BLUE-LIMIT") != NULL, "command lookup is case-insensitive");
        check(&failures, command_spec("bluelimit") == command_spec("blue-limit"),
              "bluelimit is an alias of blue-limit");
        check(&failures, command_spec("frobnicate") == NULL, "unknown command is rejected");
        check(&failures, command_spec(NULL) == NULL, "NULL command name is rejected");
        s = command_spec("rgb");
        check(&failures, s != NULL && s->min_args == 3 && s->max_args == 4 && s->per_output,
              "rgb accepts 3..4 args and is per-output");
        s = command_spec("noise");
        check(&failures, s != NULL && s->min_args == 1 && s->max_args == 1 && !s->per_output,
              "noise takes exactly 1 arg and is session-wide");
        s = command_spec("status");
        check(&failures, s != NULL && s->max_args == 0, "status takes no arguments");
        {
            size_t k;
            int sane = 1;
            for (k = 0U; k < sizeof(COMMANDS) / sizeof(COMMANDS[0]); ++k) {
                if (COMMANDS[k].min_args > COMMANDS[k].max_args ||
                    COMMANDS[k].name == NULL || COMMANDS[k].usage == NULL ||
                    strncmp(COMMANDS[k].usage, COMMANDS[k].name,
                            strlen(COMMANDS[k].name)) != 0) {
                    sane = 0;
                }
            }
            check(&failures, sane, "every command spec is self-consistent");
        }
    }

    /* ---- hotplug policy (pure decision function, no hardware needed) ---- */
    check(&failures, crtc_plan(1, "eDP-1", 0xAA, 1024, "eDP-1", 0xAA, 1024, 0, -1) == CRTC_KEEP,
          "hotplug: the same monitor keeps its pristine ramp");
    check(&failures, crtc_plan(1, "HDMI-1", 0xAA, 1024, "HDMI-1", 0xBB, 1024, 0, -1) == CRTC_CAPTURE,
          "hotplug: a DIFFERENT monitor swapped into the same port is re-captured");
    check(&failures, crtc_plan(1, "HDMI-1", 0xAA, 1024, "DP-1", 0xAA, 1024, 0, -1) == CRTC_KEEP,
          "hotplug: the same monitor moved to another port keeps its ramp");
    check(&failures, crtc_plan(1, "HDMI-1", 0U, 1024, "eDP-1", 0U, 1024, 0, -1) == CRTC_CAPTURE,
          "hotplug: without EDID the connector name decides");
    check(&failures, crtc_plan(1, "eDP-1", 0U, 1024, "eDP-1", 0U, 1024, 0, -1) == CRTC_KEEP,
          "hotplug: without EDID the same connector is kept");
    check(&failures, crtc_plan(1, "eDP-1", 0xAA, 256, "eDP-1", 0xAA, 1024, 0, -1) == CRTC_CAPTURE,
          "hotplug: a changed gamma size forces a re-capture");
    check(&failures, crtc_plan(0, NULL, 0U, -1, "HDMI-1", 0xAA, 1024, 1, 1024) == CRTC_TRANSFER,
          "hotplug: a monitor moved to another CRTC keeps its own capture");
    check(&failures, crtc_plan(0, NULL, 0U, -1, "HDMI-1", 0xAA, 1024, 1, 256) == CRTC_CAPTURE,
          "hotplug: a donor with a different ramp size is not reused");
    check(&failures, crtc_plan(0, NULL, 0U, -1, "DP-2", 0xCC, 1024, 0, -1) == CRTC_CAPTURE,
          "hotplug: a brand-new output is captured");

    /* ---- recovery file: every corruption path must be refused ---- */
    {
        char dir[PATH_MAX];
        char path[PATH_MAX];
        StateFile st;
        int have_dir = get_runtime_base(dir, sizeof(dir)) == 0;

        if (!have_dir || path_join(path, sizeof(path), dir, "phostint-selftest.state") != 0) {
            puts("skip recovery-file tests (no writable runtime directory)");
        } else {
            static const unsigned short ramp[4] = { 0U, 21845U, 43690U, 65535U };
            StateHeader h;
            StateCrtcHeader ch;
            StateBacklight blrec;
            FILE *fp;
            size_t good_size = 0U;
            int step;

            memset(&h, 0, sizeof(h));
            memcpy(h.magic, STATE_MAGIC, sizeof(h.magic));
            h.version = STATE_VERSION;
            h.count = 1U;
            h.boot_hash = boot_hash();
            h.backlight_count = 1U;
            memset(&ch, 0, sizeof(ch));
            ch.crtc = 4242U;
            ch.edid_hash = 0xABCDEF01ULL;
            ch.size = 4U;
            (void)snprintf(ch.output, sizeof(ch.output), "%s", "TEST-1");
            memset(&blrec, 0, sizeof(blrec));
            (void)snprintf(blrec.name, sizeof(blrec.name), "%s", "selftest_bl");
            blrec.original = 700U;
            blrec.maximum = 1000U;

#define WRITE_GOOD_STATE(extra_tail)                                          \
            do {                                                              \
                fp = fopen(path, "wb");                                       \
                if (fp != NULL) {                                             \
                    (void)fwrite(&h, sizeof(h), 1U, fp);                      \
                    (void)fwrite(&ch, sizeof(ch), 1U, fp);                    \
                    (void)fwrite(ramp, sizeof(unsigned short), 4U, fp);       \
                    (void)fwrite(ramp, sizeof(unsigned short), 4U, fp);       \
                    (void)fwrite(ramp, sizeof(unsigned short), 4U, fp);       \
                    (void)fwrite(&blrec, sizeof(blrec), 1U, fp);              \
                    if ((extra_tail) != 0) (void)fwrite("junk", 1U, 4U, fp);  \
                    good_size = (size_t)ftell(fp);                            \
                    (void)fclose(fp);                                         \
                }                                                             \
            } while (0)

            WRITE_GOOD_STATE(0);
            check(&failures, state_load(path, &st) == 0 && st.ramp_count == 1U &&
                  st.ramps[0].crtc == 4242U && st.ramps[0].size == 4U &&
                  st.ramps[0].edid_hash == 0xABCDEF01ULL &&
                  strcmp(st.ramps[0].output, "TEST-1") == 0 &&
                  st.ramps[0].red[3] == 65535U && st.backlight_count == 1U &&
                  strcmp(st.backlights[0].name, "selftest_bl") == 0,
                  "journal round-trips with connector name and EDID hash");
            state_free(&st);

            /* Every truncation point must be rejected, never half-applied. */
            {
                int all_rejected = 1;
                size_t full = good_size;
                for (step = 1; step < (int)full; step += 7) {
                    if (truncate(path, (off_t)step) != 0) continue;
                    if (state_load(path, &st) != -1) all_rejected = 0;
                    state_free(&st);
                }
                check(&failures, all_rejected && full > 0U,
                      "every truncated journal is rejected");
            }

            WRITE_GOOD_STATE(1);
            check(&failures, state_load(path, &st) == -1, "trailing garbage is rejected");
            state_free(&st);

            /* A journal written before a reboot must never be replayed. */
            {
                StateHeader keep = h;
                h.boot_hash = keep.boot_hash ^ 0xFFFFFFFFULL;
                WRITE_GOOD_STATE(0);
                check(&failures, state_load(path, &st) == -1,
                      "a journal from another boot is rejected");
                state_free(&st);
                h.boot_hash = 0U;
                WRITE_GOOD_STATE(0);
                check(&failures, state_load(path, &st) == -1,
                      "a journal with no boot identity is rejected");
                state_free(&st);
                h = keep;
            }

            /* Unterminated strings must not escape the loader. */
            {
                StateCrtcHeader keep = ch;
                memset(ch.output, 'A', sizeof(ch.output));
                WRITE_GOOD_STATE(0);
                check(&failures, state_load(path, &st) == -1,
                      "an unterminated connector name is rejected");
                state_free(&st);
                ch = keep;
            }
            {
                StateBacklight keep = blrec;
                memset(blrec.name, 'B', sizeof(blrec.name));
                WRITE_GOOD_STATE(0);
                check(&failures, state_load(path, &st) == -1,
                      "an unterminated backlight name is rejected");
                state_free(&st);
                blrec = keep;
            }

            /* Bad magic / wrong version. */
            fp = fopen(path, "wb");
            if (fp != NULL) {
                StateHeader bad = h;
                memcpy(bad.magic, "NOTPHOS", 7U);
                (void)fwrite(&bad, sizeof(bad), 1U, fp);
                (void)fclose(fp);
            }
            check(&failures, state_load(path, &st) == -1, "bad magic is rejected");
            state_free(&st);

            fp = fopen(path, "wb");
            if (fp != NULL) {
                StateHeader bad = h;
                bad.version = 2U;
                (void)fwrite(&bad, sizeof(bad), 1U, fp);
                (void)fclose(fp);
            }
            check(&failures, state_load(path, &st) == -1, "an older journal version is rejected");
            state_free(&st);

            /* Impossible ramp size / counts. */
            fp = fopen(path, "wb");
            if (fp != NULL) {
                StateCrtcHeader bad = ch;
                bad.size = 70000U;
                (void)fwrite(&h, sizeof(h), 1U, fp);
                (void)fwrite(&bad, sizeof(bad), 1U, fp);
                (void)fclose(fp);
            }
            check(&failures, state_load(path, &st) == -1, "an out-of-range ramp size is rejected");
            state_free(&st);

            fp = fopen(path, "wb");
            if (fp != NULL) {
                StateHeader bad = h;
                bad.count = MAX_SAVED_CRTCS + 1U;
                (void)fwrite(&bad, sizeof(bad), 1U, fp);
                (void)fclose(fp);
            }
            check(&failures, state_load(path, &st) == -1, "an out-of-range CRTC count is rejected");
            state_free(&st);

            fp = fopen(path, "wb");
            if (fp != NULL) {
                StateHeader bad = h;
                bad.backlight_count = MAX_BACKLIGHTS + 1U;
                (void)fwrite(&bad, sizeof(bad), 1U, fp);
                (void)fclose(fp);
            }
            check(&failures, state_load(path, &st) == -1,
                  "an out-of-range backlight count is rejected");
            state_free(&st);

            (void)unlink(path);
            check(&failures, state_load(path, &st) == 1, "a missing journal is not an error");
            state_free(&st);

            /*
             * A journal that exists but cannot be opened must never look like
             * "no journal": that difference decides whether the baseline can
             * be trusted.
             */
            {
                FILE *f = fopen(path, "wb");
                if (f != NULL) {
                    (void)fwrite(&h, sizeof(h), 1U, f);
                    (void)fclose(f);
                }
                if (chmod(path, 0) == 0 && access(path, R_OK) != 0) {
                    check(&failures, state_load(path, &st) == -1,
                          "an unreadable journal is an error, not an absence");
                    state_free(&st);
                } else {
                    puts("skip unreadable-journal test (running with override permissions)");
                }
                (void)chmod(path, S_IRUSR | S_IWUSR);
                (void)unlink(path);
            }
            {
                char link_path[sizeof(path) + 8];
                (void)snprintf(link_path, sizeof(link_path), "%s.link", path);
                (void)unlink(link_path);
                if (symlink("/etc/hostname", link_path) == 0) {
                    check(&failures, state_load(link_path, &st) == -1,
                          "a symlink in place of the journal is an error, not an absence");
                    state_free(&st);
                    (void)unlink(link_path);
                } else {
                    puts("skip symlink-journal test (symlink not permitted here)");
                }
            }
#undef WRITE_GOOD_STATE
        }
    }

    /* ---- physical monitor identity ---- */
    {
        LiveOutput lo;
        memset(&lo, 0, sizeof(lo));
        lo.crtc = 7;
        (void)snprintf(lo.name, sizeof(lo.name), "%s", "HDMI-1");
        lo.edid_hash = 0x1234U;

        check(&failures, identity_matches("DP-3", 0x1234U, 99U, &lo),
              "EDID wins over the connector name");
        check(&failures, !identity_matches("HDMI-1", 0x9999U, 7U, &lo),
              "a different monitor on the same port is not the same monitor");
        check(&failures, identity_matches("HDMI-1", 0U, 99U, &lo),
              "the connector name is used when one side has no EDID");
        check(&failures, !identity_matches("HDMI-2", 0U, 99U, &lo),
              "a different connector without EDID does not match");
        check(&failures, identity_matches("", 0U, 7U, &lo),
              "CRTC id is the last-resort key");
        check(&failures, fnv1a64("a", 1U) != fnv1a64("b", 1U) &&
              fnv1a64("abc", 3U) == fnv1a64("abc", 3U),
              "the identity hash is stable and discriminating");
        check(&failures, boot_hash() != 0U && boot_hash() == boot_hash(),
              "this boot has a stable identity");
    }

    /* ---- backlight preference follows the kernel ABI ---- */
    check(&failures, (int)BL_TYPE_FIRMWARE > (int)BL_TYPE_PLATFORM &&
          (int)BL_TYPE_PLATFORM > (int)BL_TYPE_RAW &&
          (int)BL_TYPE_RAW > (int)BL_TYPE_UNKNOWN,
          "backlight preference is firmware > platform > raw (kernel ABI)");
    {
        App a;
        Backlight *pick;
        memset(&a, 0, sizeof(a));
        a.backlight_count = 3U;
        (void)snprintf(a.backlights[0].name, sizeof(a.backlights[0].name), "%s", "intel_backlight");
        a.backlights[0].type = BL_TYPE_RAW;
        a.backlights[0].writable = 1;
        (void)snprintf(a.backlights[1].name, sizeof(a.backlights[1].name), "%s", "acpi_video0");
        a.backlights[1].type = BL_TYPE_FIRMWARE;
        a.backlights[1].writable = 1;
        (void)snprintf(a.backlights[2].name, sizeof(a.backlights[2].name), "%s", "thinkpad_screen");
        a.backlights[2].type = BL_TYPE_PLATFORM;
        a.backlights[2].writable = 1;
        pick = preferred_backlight(&a);
        check(&failures, pick != NULL && strcmp(pick->name, "acpi_video0") == 0,
              "firmware device is chosen over platform and raw");
        a.backlights[1].writable = 0;
        pick = preferred_backlight(&a);
        check(&failures, pick != NULL && strcmp(pick->name, "thinkpad_screen") == 0,
              "an unwritable preferred device falls through to the next one");
        a.backlights[0].writable = 0;
        a.backlights[2].writable = 0;
        check(&failures, preferred_backlight(&a) == NULL,
              "no writable device yields no target");
        check(&failures, find_backlight(&a, "acpi_video0") == &a.backlights[1] &&
              find_backlight(&a, "nope") == NULL, "backlight lookup by name works");
    }

    /* ---- the identity hash is the real FNV-1a, and the journal caps hold ---- */
    check(&failures, fnv1a64("", 0U) == 14695981039346656037ULL,
          "the empty string hashes to the FNV-1a 64-bit offset basis");
    check(&failures, fnv1a64("a", 1U) == 12638187200555641996ULL,
          "\"a\" matches the published FNV-1a 64-bit vector");
    check(&failures, fnv1a64("foobar", 6U) == 9625390261332436968ULL,
          "\"foobar\" matches the published FNV-1a 64-bit vector");

    /* ---- the journal can hold everything the program can hold ---- */
    check(&failures, MAX_JOURNAL_RAMPS >= MAX_SAVED_CRTCS * 2,
          "journal ramp capacity covers live outputs plus pending ones");
    check(&failures, MAX_JOURNAL_BACKLIGHTS >= MAX_BACKLIGHTS * 2,
          "journal backlight capacity covers changed plus pending devices");
    {
        /* The writer's worst case must stay inside what the loader accepts,
         * otherwise the fail-closed branch would be reachable in normal use. */
        size_t worst_ramps = (size_t)MAX_SAVED_CRTCS + (size_t)MAX_SAVED_CRTCS;
        size_t worst_bl = (size_t)MAX_BACKLIGHTS + (size_t)MAX_BACKLIGHTS;
        check(&failures, worst_ramps <= MAX_JOURNAL_RAMPS,
              "a full table plus a full pending list still fits the journal");
        check(&failures, worst_bl <= MAX_JOURNAL_BACKLIGHTS,
              "every backlight record still fits the journal");
    }

    /* ---- identical monitors need one-to-one matching, not just EDID ---- */
    {
        LiveOutput first, second;
        memset(&first, 0, sizeof(first));
        memset(&second, 0, sizeof(second));
        first.crtc = 1; first.edid_hash = 0x5555U; first.gamma_size = 1024;
        (void)snprintf(first.name, sizeof(first.name), "%s", "DP-1");
        second.crtc = 2; second.edid_hash = 0x5555U; second.gamma_size = 1024;
        (void)snprintf(second.name, sizeof(second.name), "%s", "DP-2");
        check(&failures, identity_matches("DP-1", 0x5555U, 1U, &first) &&
              identity_matches("DP-1", 0x5555U, 1U, &second),
              "two panels of the same model are indistinguishable by EDID alone");
        check(&failures, strcmp(first.name, second.name) != 0,
              "their connector names differ, which is what the claim pass uses");
    }

    /* ---- backlight against a simulated sysfs tree ---- */
    {
        /* Deliberately small so every composed path is provably in bounds. */
        char root[192];
        char base[PATH_MAX];
        int have_dir = get_runtime_base(base, sizeof(base)) == 0;

        if (!have_dir || path_join(root, sizeof(root), base, "phostint-bltest") != 0) {
            puts("skip backlight tests (no writable runtime directory)");
        } else {
            const char *saved_root = g_backlight_root;
            App a;
            char resp[MAX_RESPONSE];
            long v = 0L;

            (void)mkdir(root, S_IRWXU);

#define MAKE_DEV(devname, typ, maxv, curv)                                     \
            do {                                                               \
                char dpath[PATH_MAX], fpath[PATH_MAX];                         \
                FILE *f;                                                       \
                (void)snprintf(dpath, sizeof(dpath), "%s/%s", root, devname);  \
                (void)mkdir(dpath, S_IRWXU);                                   \
                (void)path_join(fpath, sizeof(fpath), dpath, "type");          \
                f = fopen(fpath, "w"); if (f) { fputs(typ, f); fclose(f); }    \
                (void)path_join(fpath, sizeof(fpath), dpath, "max_brightness");\
                f = fopen(fpath, "w"); if (f) { fprintf(f, "%d\n", maxv); fclose(f); } \
                (void)path_join(fpath, sizeof(fpath), dpath, "brightness");    \
                f = fopen(fpath, "w"); if (f) { fprintf(f, "%d\n", curv); fclose(f); } \
            } while (0)

            MAKE_DEV("zz_raw", "raw", 1000, 500);
            MAKE_DEV("aa_firmware", "firmware", 100, 80);

            g_backlight_root = root;
            memset(&a, 0, sizeof(a));
            (void)snprintf(a.recovery_path, sizeof(a.recovery_path), "%s/journal", root);
            discover_backlights(&a);
            check(&failures, a.backlight_count == 2U, "both simulated devices are discovered");
            check(&failures, find_backlight(&a, "aa_firmware") != NULL &&
                  find_backlight(&a, "aa_firmware")->type == BL_TYPE_FIRMWARE &&
                  find_backlight(&a, "zz_raw")->type == BL_TYPE_RAW,
                  "the sysfs 'type' file is parsed");
            check(&failures, preferred_backlight(&a) != NULL &&
                  strcmp(preferred_backlight(&a)->name, "aa_firmware") == 0,
                  "firmware is preferred even when raw sorts first on disk");

            /* One device per command, and only the chosen one. */
            check(&failures, set_backlight_percent(&a, 50.0, NULL, resp, sizeof(resp)) == 0,
                  "a bare backlight command succeeds");
            {
                char p[PATH_MAX];
                (void)snprintf(p, sizeof(p), "%s/aa_firmware/brightness", root);
                (void)read_long_file(p, &v);
                check(&failures, v == 50L, "the preferred device received the change");
                (void)snprintf(p, sizeof(p), "%s/zz_raw/brightness", root);
                (void)read_long_file(p, &v);
                check(&failures, v == 500L, "the other device was left untouched");
            }

            /* Write-ahead: the journal already holds the pre-change level. */
            {
                StateFile js;
                check(&failures, state_load(a.recovery_path, &js) == 0 &&
                      js.backlight_count == 1U &&
                      strcmp(js.backlights[0].name, "aa_firmware") == 0 &&
                      js.backlights[0].original == 80U &&
                      js.backlights[0].maximum == 100U,
                      "the journal recorded the original level before the write");
                state_free(&js);
            }

            /* A level changed behind our back becomes the new "original". */
            {
                char p[PATH_MAX];
                (void)snprintf(p, sizeof(p), "%s/zz_raw/brightness", root);
                (void)write_long_file(p, 900L);
                check(&failures, set_backlight_percent(&a, 10.0, "zz_raw", resp, sizeof(resp)) == 0,
                      "a named device can be driven");
                check(&failures, find_backlight(&a, "zz_raw")->original == 900L,
                      "the original is re-read just before our first change");
                (void)read_long_file(p, &v);
                check(&failures, v == 100L, "the named device reached the requested level");
            }

            check(&failures, set_backlight_percent(&a, 50.0, "nope", resp, sizeof(resp)) != 0,
                  "an unknown device name is refused");

            /* Restoration puts both devices back where they were found. */
            check(&failures, restore_backlights(&a) == 0, "restoring reports no failure");
            {
                char p[PATH_MAX];
                (void)snprintf(p, sizeof(p), "%s/aa_firmware/brightness", root);
                (void)read_long_file(p, &v);
                check(&failures, v == 80L, "the firmware device is back to its original level");
                (void)snprintf(p, sizeof(p), "%s/zz_raw/brightness", root);
                (void)read_long_file(p, &v);
                check(&failures, v == 900L, "the raw device is back to the level found on disk");
            }

            /* A device whose scale changed must not be written blindly. */
            {
                StateBacklight rec;
                char p[PATH_MAX];
                memset(&rec, 0, sizeof(rec));
                (void)snprintf(rec.name, sizeof(rec.name), "%s", "aa_firmware");
                rec.original = 40U;
                rec.maximum = 999U;   /* the device really reports 100 */
                check(&failures, restore_backlight_record(&rec) == 0,
                      "a changed max_brightness makes the stored level unusable");
                (void)snprintf(p, sizeof(p), "%s/aa_firmware/brightness", root);
                (void)read_long_file(p, &v);
                check(&failures, v == 80L, "and nothing was written in that case");
                rec.maximum = 100U;
                check(&failures, restore_backlight_record(&rec) == 1 &&
                      read_long_file(p, &v) == 0 && v == 40L,
                      "a matching scale restores the exact level");
            }

            /*
             * A level still owed by a crashed instance must survive the user
             * moving the brightness by hand: the debt is older, and it is the
             * value from before PhosTint first touched the device.
             */
            {
                char p[PATH_MAX];
                a.backlights[0].changed = 0;
                a.backlights[1].changed = 0;
                a.pending_bl_count = 1U;
                memset(&a.pending_bl[0], 0, sizeof(a.pending_bl[0]));
                (void)snprintf(a.pending_bl[0].name, sizeof(a.pending_bl[0].name),
                               "%s", "aa_firmware");
                a.pending_bl[0].original = 77U;      /* the true pre-PhosTint level */
                a.pending_bl[0].maximum = 100U;
                (void)snprintf(p, sizeof(p), "%s/aa_firmware/brightness", root);
                (void)write_long_file(p, 12L);       /* user moved it meanwhile */
                (void)snprintf(a.recovery_path, sizeof(a.recovery_path), "%s/journal2", root);

                check(&failures,
                      set_backlight_percent(&a, 90.0, "aa_firmware", resp, sizeof(resp)) == 0,
                      "a device with an owed level can still be driven");
                check(&failures, find_backlight(&a, "aa_firmware")->original == 77L,
                      "the owed original wins over the level the user set meanwhile");
                check(&failures, a.pending_bl_count == 0U,
                      "the debt moved from the pending list to the live device");
                check(&failures, restore_backlights(&a) == 0 &&
                      read_long_file(p, &v) == 0 && v == 77L,
                      "restoring puts back the oldest original, not the interim value");
                (void)unlink(a.recovery_path);
            }

            /* Refuse to touch hardware when the journal cannot be written. */
            {
                char p[PATH_MAX];
                long before = -1L;

                (void)unlink(a.recovery_path);
                (void)snprintf(a.recovery_path, sizeof(a.recovery_path),
                               "%s/missing-dir/journal", root);
                a.backlights[0].changed = 0;
                a.backlights[1].changed = 0;
                a.pending_bl_count = 0U;
                (void)snprintf(p, sizeof(p), "%s/aa_firmware/brightness", root);
                (void)read_long_file(p, &before);
                check(&failures,
                      set_backlight_percent(&a, 25.0, "aa_firmware", resp, sizeof(resp)) != 0,
                      "an unwritable journal blocks the backlight change");
                (void)read_long_file(p, &v);
                check(&failures, before > 0L && v == before,
                      "and the hardware was not touched");
            }

            g_backlight_root = saved_root;
            {
                /* Leave nothing behind in the user's runtime directory. */
                char cleanup[256];
                (void)snprintf(cleanup, sizeof(cleanup), "%s/aa_firmware", root);
                (void)unlink_tree(cleanup);
                (void)snprintf(cleanup, sizeof(cleanup), "%s/zz_raw", root);
                (void)unlink_tree(cleanup);
                if (unlink_tree(root) != 0) {
                    printf("note: could not remove the temporary test tree %s\n", root);
                }
            }
#undef MAKE_DEV
        }
    }

    /* ---- a journal may only be discarded when nothing is outstanding ---- */
    {
        RecoveryResult rres;
        memset(&rres, 0, sizeof(rres));
        check(&failures, recovery_complete(&rres), "an empty result is complete");
        rres.ramps_unmatched = 1;
        check(&failures, !recovery_complete(&rres), "an unmatched ramp keeps the journal");
        memset(&rres, 0, sizeof(rres));
        rres.backlights_failed = 1;
        check(&failures, !recovery_complete(&rres), "a failed backlight keeps the journal");
        memset(&rres, 0, sizeof(rres));
        rres.x_errors = 1;
        check(&failures, !recovery_complete(&rres), "an X error keeps the journal");
    }

    /* ---- misc helpers used on every code path ---- */
    check(&failures, deadline_delta(1000U, 400U, -1) == 600 &&
          deadline_delta(1000U, 400U, 100) == 100 &&
          deadline_delta(300U, 400U, -1) == 0,
          "poll deadlines never go negative and keep the nearest one");
    {
        char small[8];
        size_t used = 0U;
        buf_append(small, sizeof(small), &used, "%s", "0123456789abcdef");
        check(&failures, used == sizeof(small) && small[sizeof(small) - 1U] == '\0',
              "buf_append truncates safely and stays terminated");
        buf_append(small, sizeof(small), &used, "%s", "more");
        check(&failures, used == sizeof(small), "buf_append is a no-op once full");
    }
    {
        ColorState s;
        double fr, fg, fb;
        reset_color_state(&s);
        state_factors(&s, &fr, &fg, &fb);
        check(&failures, fr == 1.0 && fg == 1.0 && fb == 1.0,
              "a reset look is the identity transform");
        s.b = 0.0;
        s.strength = 1.0;
        state_factors(&s, &fr, &fg, &fb);
        check(&failures, fr == 1.0 && fb == 0.0, "blue 0 removes all digital blue");
        reset_color_state(&s);
        s.blue_limit = 0.5;
        s.brightness = 0.5;
        state_factors(&s, &fr, &fg, &fb);
        check(&failures, fabs(fr - 0.5) < 1e-9 && fabs(fb - 0.25) < 1e-9,
              "blue limit composes multiplicatively with brightness");
        reset_color_state(&s);
        s.strength = 0.0;
        s.r = 0.0;
        s.g = 0.0;
        s.b = 0.0;
        state_factors(&s, &fr, &fg, &fb);
        check(&failures, fr == 1.0 && fg == 1.0 && fb == 1.0,
              "strength 0 neutralizes any tint");
    }
    {
        TuiVals v;
        double fr, fg, fb;
        memset(&v, 0, sizeof(v));
        v.red = 100.0;
        v.green = 80.0;
        v.blue = 50.0;
        v.brightness = 50.0;
        v.blue_limit = 50.0;
        tui_final_output(&v, &fr, &fg, &fb);
        check(&failures, fabs(fr - 50.0) < 1e-9 && fabs(fg - 40.0) < 1e-9 &&
              fabs(fb - 12.5) < 1e-9,
              "TUI reports what the panel receives, including brightness and blue limit");
    }

    if (failures == 0) {
        puts("All self-tests passed.");
        return EXIT_OK;
    }
    printf("%d self-test(s) failed.\n", failures);
    return EXIT_RUNTIME;
}

static void print_presets(void)
{
    puts("Presets:");
    puts("  green            strong green tint");
    puts("  green-soft       gentler green tint");
    puts("  amber            amber/phosphor-like tint");
    puts("  red              strong red tint");
    puts("  pink             pink/magenta tint");
    puts("  sepia            mild sepia tint");
    puts("  warm             warm, lower-blue tint");
    puts("  low-blue         blue digital channel at 20%");
    puts("  ultra-low-blue   blue digital channel at 3%");
    puts("  zero-blue        blue digital channel at 0% (not physical spectral zero)");
    puts("");
    puts("Color temperature is separate: 'temp 1000..10000' (e.g. temp 3400).");
}

static void print_help(const char *argv0)
{
    printf("%s %s - safe X11/i3wm tint and brightness controller\n\n", APP_NAME, APP_VERSION);
    printf("Usage:\n");
    printf("  %s start                  start the background daemon\n", argv0);
    printf("  %s foreground             run the daemon in the foreground\n", argv0);
    printf("  %s status                 one-line daemon status\n", argv0);
    printf("  %s list                   connected outputs and backlight devices\n", argv0);
    printf("  %s preset NAME            apply a named preset\n", argv0);
    printf("  %s color RRGGBB [str%%]    tint with a hex color\n", argv0);
    printf("  %s rgb R%% G%% B%% [str%%]    tint with RGB channel percentages\n", argv0);
    printf("  %s temp KELVIN            approximate white point, 1000..10000 K\n", argv0);
    printf("  %s blue PERCENT           digital blue-channel level, 0..100\n", argv0);
    printf("  %s blue-limit PERCENT     blue ceiling over any mode, 0..100\n", argv0);
    printf("  %s strength PERCENT       re-scale the current tint, 0..100\n", argv0);
    printf("  %s brightness PERCENT     software brightness, 1..100\n", argv0);
    printf("  %s backlight PERCENT [DEV] hardware backlight, 1..100 (if writable)\n", argv0);
    printf("  %s noise PERCENT          golden-ratio CRT noise, 0..100 (needs a compositor)\n", argv0);
    printf("  %s output NAME COMMAND    aim any color command at one monitor\n", argv0);
    printf("  %s normal                 restore the captured original state\n", argv0);
    printf("  %s tui                    full-screen control panel (arrow keys)\n", argv0);
    printf("  %s interactive            plain question-and-answer menu\n", argv0);
    printf("  %s stop                   restore and stop the daemon\n", argv0);
    printf("  %s emergency-reset        recover without a running daemon\n", argv0);
    printf("  %s presets                list preset names\n", argv0);
    printf("  %s selftest               offline checks of internal math (no X needed)\n\n", argv0);
    printf("Examples:\n");
    printf("  %s tui\n", argv0);
    printf("  %s preset green\n", argv0);
    printf("  %s temp 3400\n", argv0);
    printf("  %s color FF66CC 80\n", argv0);
    printf("  %s rgb 100 60 10 90\n", argv0);
    printf("  %s brightness 45\n", argv0);
    printf("  %s blue 0\n", argv0);
    printf("  %s preset green && %s noise 35   (green phosphor CRT)\n", argv0, argv0);
    printf("  %s output HDMI-1 temp 3000       (one monitor only)\n", argv0);
    printf("  %s normal\n\n", argv0);
    printf("Notes:\n");
    printf("  * Commands automatically start the background daemon when needed.\n");
    printf("  * 'normal' restores the exact gamma ramps captured at daemon startup.\n");
    printf("  * Monitor hotplug and mode changes re-apply the current look automatically;\n");
    printf("    a monitor keeps its own look and its own pristine ramp across replugs.\n");
    printf("  * 'backlight' drives exactly one device. The default follows the kernel's own\n");
    printf("    preference (firmware, then platform, then raw); 'list' marks it with '*'\n");
    printf("    and shows every device name, so you can target another one by name.\n");
    printf("  * Do not run redshift/gammastep/night-light tools at the same time.\n");
    printf("  * i3wm autostart: exec --no-startup-id %s start\n", argv0);
    printf("  * Digital color control cannot guarantee identical physical (spectral) output.\n");
}

static int command_from_args(int argc, char **argv, int first, char *out, size_t out_size)
{
    int i;
    size_t used = 0U;
    for (i = first; i < argc; ++i) {
        int n = snprintf(out + used, out_size - used, "%s%s", i == first ? "" : " ", argv[i]);
        if (n < 0 || (size_t)n >= out_size - used) return -1;
        used += (size_t)n;
    }
    return 0;
}

static void print_response(const char *response)
{
    if (strncmp(response, "OK ", 3U) == 0) {
        puts(response + 3);
    } else if (strncmp(response, "ERR ", 4U) == 0) {
        fprintf(stderr, "%s\n", response + 4);
    } else {
        fputs(response, stdout);
        if (response[0] != '\0' && response[strlen(response) - 1U] != '\n') putchar('\n');
    }
}

static int emergency_reset(const char *socket_path, const char *recovery_path,
                           int allow_identity, int runtime_is_volatile)
{
    Display *dpy;
    Window root = None;
    char response[MAX_RESPONSE];
    StateFile st;
    RecoveryResult rr;
    int rc;
    int identity = 0;
    int identity_errors = 0;

    /*
     * A healthy daemon is the best recovery path: it still holds the exact
     * pristine capture in memory. The 10 s client timeout guarantees this
     * probe cannot hang when the daemon is wedged.
     */
    if (send_daemon_command(socket_path, "normal", response, sizeof(response)) == 0 &&
        strncmp(response, "OK ", 3U) == 0) {
        puts("A running PhosTint daemon restored the original state.");
        return EXIT_OK;
    }

    dpy = XOpenDisplay(NULL);
    if (dpy != NULL) {
        XSetErrorHandler(x_error_handler);
        root = DefaultRootWindow(dpy);
    } else {
        fprintf(stderr, "Warning: cannot open the X11 display; trying backlight-only recovery.\n");
    }

    /* Same boot-stamp policy as the daemon, or the two would disagree about
     * whether the very same file is usable. */
    rc = recovery_apply(dpy, root, recovery_path, &st, &rr, runtime_is_volatile);
    if (rc < 0) {
        fprintf(stderr, "The recovery journal is unusable (corrupt, or from a previous boot);"
                        " it was not applied.\n");
    } else if (rc == 0) {
        printf("Recovered %d gamma ramp(s) and %d backlight value(s) from the journal.\n",
               rr.ramps_restored, rr.backlights_restored);
        if (rr.x_errors > 0) {
            fprintf(stderr, "The X server rejected %d request(s).\n", rr.x_errors);
        }
        if (rr.ramps_unmatched > 0) {
            printf("%d ramp(s) matched no connected monitor and were kept in the journal.\n",
                   rr.ramps_unmatched);
        }
        if (rr.backlights_failed > 0) {
            fprintf(stderr, "%d backlight value(s) could not be restored; the journal was kept.\n",
                    rr.backlights_failed);
        }
        state_free(&st);
        if (recovery_complete(&rr)) (void)unlink(recovery_path);
        if (dpy != NULL) XCloseDisplay(dpy);
        return recovery_complete(&rr) ? EXIT_OK : EXIT_RUNTIME;
    }

    /*
     * No journal. Identity gamma would overwrite every connected monitor,
     * including ones PhosTint never touched and any ICC calibration loaded by
     * another tool, so it is never done implicitly.
     */
    if (!allow_identity) {
        if (dpy != NULL) XCloseDisplay(dpy);
        fprintf(stderr,
                "No usable recovery journal was found, so there is nothing to restore precisely.\n"
                "If the screen still looks wrong, you can force a neutral ramp on EVERY connected\n"
                "monitor with:  phostint emergency-reset --identity\n"
                "That discards any ICC calibration loaded by other tools, which is why it is not\n"
                "done automatically.\n");
        return EXIT_RUNTIME;
    }
    if (dpy != NULL) {
        identity = set_identity_gamma(dpy, root, &identity_errors);
        XCloseDisplay(dpy);
    }
    if (identity_errors > 0) {
        fprintf(stderr, "The X server rejected %d of the identity requests; "
                        "%d output(s) were reset.\n", identity_errors, identity);
        return EXIT_RUNTIME;
    }
    if (identity > 0) {
        printf("Set %d active CRTC(s) to identity gamma at your explicit request.\n", identity);
        printf("Note: identity gamma does not reproduce a custom ICC calibration.\n");
        return EXIT_OK;
    }
    fprintf(stderr, "Could not reset any XRandR CRTC.\n");
    return EXIT_RUNTIME;
}

/*
 * Strip the newline from an fgets() buffer. When the typed line was longer
 * than the buffer, discard the remainder so it cannot leak into the next
 * menu prompt as a bogus answer.
 */
static void trim_line(char *line)
{
    size_t len = strlen(line);
    if (len > 0U && line[len - 1U] != '\n') {
        int c;
        while ((c = getchar()) != EOF && c != '\n') {}
    }
    line[strcspn(line, "\r\n")] = '\0';
}

static int run_interactive(const char *socket_path)
{
    char line[128];
    char command[MAX_COMMAND];
    char response[MAX_RESPONSE];
    char err[256];

    if (start_daemon(socket_path, err, sizeof(err), 0) != 0) {
        fprintf(stderr, "Could not start the PhosTint daemon: %s.\n", err);
        return EXIT_RUNTIME;
    }

    for (;;) {
        puts("\nPhosTint interactive control  (tip: 'phostint tui' is the full panel)");
        puts("  1) Preset");
        puts("  2) Custom color (RRGGBB)");
        puts("  3) Color temperature (Kelvin)");
        puts("  4) Software brightness");
        puts("  5) Digital blue-channel level");
        puts("  6) Hardware backlight (if permitted)");
        puts("  7) Status");
        puts("  8) List outputs and backlights");
        puts("  9) Restore normal");
        puts("  b) Blue-light limit over any mode (0..100)");
        puts("  n) Noise overlay intensity (0..100)");
        puts("  0) Exit menu (daemon keeps running)");
        puts("  x) Restore normal and stop daemon");
        printf("Choice: ");
        fflush(stdout);
        if (fgets(line, sizeof(line), stdin) == NULL) break;
        trim_line(line);

        if (line[0] == '1') {
            print_presets();
            printf("Preset name: ");
            fflush(stdout);
            if (fgets(line, sizeof(line), stdin) == NULL) continue;
            trim_line(line);
            (void)snprintf(command, sizeof(command), "preset %s", line);
        } else if (line[0] == '2') {
            char color[32];
            char strength[32];
            printf("Color RRGGBB (example 33FF66): ");
            fflush(stdout);
            if (fgets(color, sizeof(color), stdin) == NULL) continue;
            trim_line(color);
            printf("Tint strength 0..100 [70]: ");
            fflush(stdout);
            if (fgets(strength, sizeof(strength), stdin) == NULL) continue;
            trim_line(strength);
            if (strength[0] == '\0') (void)snprintf(strength, sizeof(strength), "%s", "70");
            (void)snprintf(command, sizeof(command), "color %s %s", color, strength);
        } else if (line[0] == '3') {
            printf("Temperature in Kelvin 1000..10000 [3400]: ");
            fflush(stdout);
            if (fgets(line, sizeof(line), stdin) == NULL) continue;
            trim_line(line);
            if (line[0] == '\0') (void)snprintf(line, sizeof(line), "%s", "3400");
            (void)snprintf(command, sizeof(command), "temp %s", line);
        } else if (line[0] == '4') {
            printf("Software brightness 1..100: ");
            fflush(stdout);
            if (fgets(line, sizeof(line), stdin) == NULL) continue;
            trim_line(line);
            (void)snprintf(command, sizeof(command), "brightness %s", line);
        } else if (line[0] == '5') {
            printf("Digital blue-channel level 0..100: ");
            fflush(stdout);
            if (fgets(line, sizeof(line), stdin) == NULL) continue;
            trim_line(line);
            (void)snprintf(command, sizeof(command), "blue %s", line);
        } else if (line[0] == '6') {
            printf("Hardware backlight 1..100: ");
            fflush(stdout);
            if (fgets(line, sizeof(line), stdin) == NULL) continue;
            trim_line(line);
            (void)snprintf(command, sizeof(command), "backlight %s", line);
        } else if (line[0] == '7') {
            (void)snprintf(command, sizeof(command), "%s", "status");
        } else if (line[0] == '8') {
            (void)snprintf(command, sizeof(command), "%s", "list");
        } else if (line[0] == '9') {
            (void)snprintf(command, sizeof(command), "%s", "normal");
        } else if (line[0] == 'b' || line[0] == 'B') {
            printf("Blue-light limit 0..100 (100 = no extra blocking): ");
            fflush(stdout);
            if (fgets(line, sizeof(line), stdin) == NULL) continue;
            trim_line(line);
            (void)snprintf(command, sizeof(command), "blue-limit %s", line);
        } else if (line[0] == 'n' || line[0] == 'N') {
            printf("Noise intensity 0..100 (0 = off; needs a compositor like picom): ");
            fflush(stdout);
            if (fgets(line, sizeof(line), stdin) == NULL) continue;
            trim_line(line);
            (void)snprintf(command, sizeof(command), "noise %s", line);
        } else if (line[0] == '0') {
            puts("Menu closed. PhosTint daemon remains active.");
            return EXIT_OK;
        } else if (line[0] == 'x' || line[0] == 'X') {
            (void)snprintf(command, sizeof(command), "%s", "stop");
        } else {
            puts("Invalid choice.");
            continue;
        }

        if (send_daemon_command(socket_path, command, response, sizeof(response)) != 0) {
            fprintf(stderr, "Could not communicate with daemon.\n");
            continue;
        }
        print_response(response);
        if (strcmp(command, "stop") == 0) return EXIT_OK;
    }
    return EXIT_OK;
}

int main(int argc, char **argv)
{
    char socket_path[PATH_MAX];
    char recovery_path[PATH_MAX];
    char lock_path[PATH_MAX];
    char command[MAX_COMMAND];
    char response[MAX_RESPONSE];
    char err[256];
    int runtime_volatile = 0;
    int accept_baseline = 0;
    int rc;

    /* A peer disappearing during IPC must become an error, never SIGPIPE. */
    (void)signal(SIGPIPE, SIG_IGN);

    if (make_runtime_paths_ex(socket_path, sizeof(socket_path),
                              recovery_path, sizeof(recovery_path),
                              lock_path, sizeof(lock_path), &runtime_volatile) != 0) {
        fprintf(stderr, "Could not create runtime paths.\n");
        return EXIT_RUNTIME;
    }

    if (argc < 2) {
        print_help(argv[0]);
        return EXIT_OK;
    }
    /*
     * Opt in to trusting the ramps currently on screen as the baseline. Only
     * meaningful after an unreadable journal, which is the one case where the
     * daemon refuses to guess.
     */
    if (argc >= 3 && strcmp(argv[argc - 1], "--accept-current") == 0) {
        accept_baseline = 1;
        argc--;
    }
    if (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "help") == 0) {
        print_help(argv[0]);
        return EXIT_OK;
    }
    if (strcmp(argv[1], "--version") == 0 || strcmp(argv[1], "version") == 0) {
        printf("%s %s\n", APP_NAME, APP_VERSION);
        return EXIT_OK;
    }
    if (strcmp(argv[1], "presets") == 0) {
        print_presets();
        return EXIT_OK;
    }
    if (strcmp(argv[1], "selftest") == 0) {
        return run_selftest();
    }
    if (strcmp(argv[1], "emergency-reset") == 0) {
        int allow_identity = (argc >= 3 &&
                              (strcmp(argv[2], "--identity") == 0 ||
                               strcmp(argv[2], "--force") == 0)) ? 1 : 0;
        if (argc > 3 || (argc == 3 && !allow_identity)) {
            fprintf(stderr, "Usage: %s emergency-reset [--identity]\n", argv[0]);
            return EXIT_USAGE;
        }
        return emergency_reset(socket_path, recovery_path, allow_identity, runtime_volatile);
    }
    if (strcmp(argv[1], "foreground") == 0) {
        return daemon_loop(-1, accept_baseline);
    }
    if (strcmp(argv[1], "tui") == 0) {
        return run_tui(socket_path);
    }
    if (strcmp(argv[1], "interactive") == 0) {
        return run_interactive(socket_path);
    }
    if (strcmp(argv[1], "start") == 0) {
        if (start_daemon(socket_path, err, sizeof(err), accept_baseline) != 0) {
            fprintf(stderr, "Could not start PhosTint: %s.\n", err);
            return EXIT_RUNTIME;
        }
        puts("PhosTint daemon is running in the background.");
        return EXIT_OK;
    }

    if (strcmp(argv[1], "status") == 0 || strcmp(argv[1], "stop") == 0) {
        if (send_daemon_command(socket_path, argv[1], response, sizeof(response)) != 0) {
            if (strcmp(argv[1], "status") == 0) {
                puts("PhosTint daemon is not running.");
                return EXIT_OK;
            }
            puts("PhosTint daemon is not running; nothing to stop.");
            return EXIT_OK;
        }
        print_response(response);
        return strncmp(response, "OK ", 3U) == 0 ? EXIT_OK : EXIT_RUNTIME;
    }

    if (command_from_args(argc, argv, 1, command, sizeof(command)) != 0) {
        fprintf(stderr, "Command is too long.\n");
        return EXIT_USAGE;
    }

    /* User-friendly behavior: mutation commands auto-start the daemon. */
    if (start_daemon(socket_path, err, sizeof(err), accept_baseline) != 0) {
        fprintf(stderr, "Could not start PhosTint: %s.\n", err);
        return EXIT_RUNTIME;
    }
    rc = send_daemon_command(socket_path, command, response, sizeof(response));
    if (rc != 0) {
        fprintf(stderr, "Could not communicate with PhosTint daemon.\n");
        return EXIT_RUNTIME;
    }
    print_response(response);
    return strncmp(response, "OK ", 3U) == 0 ? EXIT_OK : EXIT_RUNTIME;
}
