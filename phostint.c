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
 *     changed by this process.
 *   - SIGINT/SIGTERM/SIGHUP/SIGQUIT trigger restoration before exit.
 *   - If the X server dies, changed backlight values are still restored.
 *   - "emergency-reset" first tries the recovery file; if unavailable it
 *     falls back to an identity gamma ramp.
 *   - A per-display file lock guarantees a single daemon instance.
 *   - RandR events are monitored: monitor hotplug and mode changes re-sync
 *     the captured state and re-apply the current tint automatically.
 *   - The recovery file is only trusted when it is a regular file owned by
 *     the current user (never followed through a symlink).
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
 *      $(pkg-config --cflags --libs x11 xrandr) -lm -o phostint
 */

#define _POSIX_C_SOURCE 200809L

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/extensions/Xrandr.h>
#include <X11/extensions/shape.h>

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
#define APP_VERSION "1.2.0"
#define MAX_SAVED_CRTCS 64
#define MAX_BACKLIGHTS 32
#define MAX_COMMAND 512
#define MAX_RESPONSE 2048
#define STATE_MAGIC "PHOST01"
#define STATE_VERSION 2U

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

typedef struct {
    RRCrtc id;
    XRRCrtcGamma *original;
} SavedCrtc;

typedef struct {
    char name[NAME_MAX + 1];
    char brightness_path[PATH_MAX];
    long original;
    long maximum;
    int writable;
    int changed;
} Backlight;

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

typedef struct {
    Display *dpy;
    Window root;
    int randr_event_base;
    int lock_fd;
    SavedCrtc crtcs[MAX_SAVED_CRTCS];
    size_t crtc_count;
    Backlight backlights[MAX_BACKLIGHTS];
    size_t backlight_count;
    ColorState state;
    NoiseOverlay noise;
    uint64_t next_reassert_ms;
    char socket_path[PATH_MAX];
    char recovery_path[PATH_MAX];
    char lock_path[PATH_MAX];
} App;

typedef struct {
    char magic[8];
    uint32_t version;
    uint32_t count;
} StateHeader;

typedef struct {
    uint64_t crtc;
    uint32_t size;
    uint32_t reserved;
} StateCrtcHeader;

/* Fixed-size record so the state file layout never depends on NAME_MAX. */
typedef struct {
    char name[256];
    uint64_t original;
    uint64_t maximum;
} StateBacklight;

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

static int get_runtime_base(char *out, size_t out_size)
{
    const char *dir = getenv("XDG_RUNTIME_DIR");
    struct stat st;
    int n;

    if (dir != NULL && dir[0] == '/' && stat(dir, &st) == 0 &&
        S_ISDIR(st.st_mode) && st.st_uid == getuid()) {
        n = snprintf(out, out_size, "%s", dir);
        return (n < 0 || (size_t)n >= out_size) ? -1 : 0;
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

static int make_runtime_paths(char *socket_path, size_t socket_size,
                              char *recovery_path, size_t recovery_size,
                              char *lock_path, size_t lock_size)
{
    char base[PATH_MAX];
    char socket_name[128];
    char state_name[128];
    char lock_name[128];
    const char *display = getenv("DISPLAY");
    unsigned long h;
    int n1, n2, n3;

    if (get_runtime_base(base, sizeof(base)) != 0) return -1;
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

static void discover_backlights(App *app)
{
    const char *base = "/sys/class/backlight";
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
        bl->writable = (access(bl->brightness_path, W_OK) == 0) ? 1 : 0;
        bl->changed = 0;
        app->backlight_count++;
    }
    (void)closedir(dir);
}

/*
 * Restore one backlight value described by a state-file record. The device
 * name is validated and the sysfs path is rebuilt locally, so a corrupted or
 * hostile state file can never steer writes outside /sys/class/backlight.
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

    n = snprintf(device_dir, sizeof(device_dir), "/sys/class/backlight/%s", rec->name);
    if (n < 0 || (size_t)n >= sizeof(device_dir)) return 0;
    if (path_join(bright_path, sizeof(bright_path), device_dir, "brightness") != 0) return 0;
    if (path_join(max_path, sizeof(max_path), device_dir, "max_brightness") != 0) return 0;

    if (read_long_file(max_path, &maximum) != 0 || maximum <= 0) return 0;
    target = (long)rec->original;
    if (target < 0L) return 0;
    if (target > maximum) target = maximum;
    if (access(bright_path, W_OK) != 0) return 0;
    return write_long_file(bright_path, target) == 0 ? 1 : 0;
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
    return ok ? 0 : -1;
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

        if (!XMatchVisualInfo(app->dpy, screen, 32, TrueColor, &vinfo)) {
            (void)snprintf(err, err_size, "%s",
                           "no 32-bit ARGB visual is available on this display");
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

/*
 * Gamma size of an active CRTC, using screen resources the caller already
 * fetched (one XRRGetScreenResourcesCurrent per operation, not per CRTC).
 * Returns 0 when the CRTC is missing, disabled, or has no outputs.
 */
static int crtc_size_in(Display *dpy, XRRScreenResources *res, RRCrtc target)
{
    int size = 0;
    int i;

    for (i = 0; i < res->ncrtc; ++i) {
        if (res->crtcs[i] == target) {
            XRRCrtcInfo *info = XRRGetCrtcInfo(dpy, res, target);
            if (info != NULL) {
                if (info->mode != None && info->noutput > 0) {
                    size = XRRGetCrtcGammaSize(dpy, target);
                }
                XRRFreeCrtcInfo(info);
            }
            break;
        }
    }
    return size;
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

static int capture_original_crtcs(App *app)
{
    XRRScreenResources *res;
    int i;

    free_saved_crtcs(app);
    res = XRRGetScreenResourcesCurrent(app->dpy, app->root);
    if (res == NULL) return -1;

    for (i = 0; i < res->ncrtc && app->crtc_count < MAX_SAVED_CRTCS; ++i) {
        XRRCrtcInfo *info = XRRGetCrtcInfo(app->dpy, res, res->crtcs[i]);
        int gamma_size;
        XRRCrtcGamma *gamma;

        if (info == NULL) continue;
        if (info->mode == None || info->noutput <= 0) {
            XRRFreeCrtcInfo(info);
            continue;
        }
        gamma_size = XRRGetCrtcGammaSize(app->dpy, res->crtcs[i]);
        XRRFreeCrtcInfo(info);
        if (gamma_size <= 0 || gamma_size > 65536) continue;

        gamma = XRRGetCrtcGamma(app->dpy, res->crtcs[i]);
        if (gamma == NULL || gamma->size != gamma_size) {
            if (gamma != NULL) XRRFreeGamma(gamma);
            continue;
        }
        app->crtcs[app->crtc_count].id = res->crtcs[i];
        app->crtcs[app->crtc_count].original = gamma;
        app->crtc_count++;
    }

    XRRFreeScreenResources(res);
    return app->crtc_count > 0U ? 0 : -1;
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
    if (app->crtcs[index].original != NULL) {
        XRRFreeGamma(app->crtcs[index].original);
    }
    memmove(&app->crtcs[index], &app->crtcs[index + 1U],
            (app->crtc_count - index - 1U) * sizeof(app->crtcs[0]));
    app->crtc_count--;
}

static int save_recovery_file(const App *app)
{
    char tmp_path[PATH_MAX];
    FILE *fp;
    StateHeader header;
    uint32_t bl_count;
    size_t i;
    int fd;
    int n;

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

    memset(&header, 0, sizeof(header));
    memcpy(header.magic, STATE_MAGIC, sizeof(header.magic));
    header.version = STATE_VERSION;
    header.count = (uint32_t)app->crtc_count;

    if (fwrite(&header, sizeof(header), 1U, fp) != 1U) goto fail;
    for (i = 0U; i < app->crtc_count; ++i) {
        StateCrtcHeader ch;
        XRRCrtcGamma *g = app->crtcs[i].original;
        if (g == NULL || g->size <= 0) goto fail;
        memset(&ch, 0, sizeof(ch));
        ch.crtc = (uint64_t)app->crtcs[i].id;
        ch.size = (uint32_t)g->size;
        if (fwrite(&ch, sizeof(ch), 1U, fp) != 1U) goto fail;
        if (fwrite(g->red, sizeof(unsigned short), (size_t)g->size, fp) != (size_t)g->size) goto fail;
        if (fwrite(g->green, sizeof(unsigned short), (size_t)g->size, fp) != (size_t)g->size) goto fail;
        if (fwrite(g->blue, sizeof(unsigned short), (size_t)g->size, fp) != (size_t)g->size) goto fail;
    }

    bl_count = (uint32_t)app->backlight_count;
    if (fwrite(&bl_count, sizeof(bl_count), 1U, fp) != 1U) goto fail;
    for (i = 0U; i < app->backlight_count; ++i) {
        StateBacklight rec;
        const Backlight *bl = &app->backlights[i];
        memset(&rec, 0, sizeof(rec));
        (void)snprintf(rec.name, sizeof(rec.name), "%s", bl->name);
        rec.original = (uint64_t)bl->original;
        rec.maximum = (uint64_t)bl->maximum;
        if (fwrite(&rec, sizeof(rec), 1U, fp) != 1U) goto fail;
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
static FILE *open_state_for_read(const char *path)
{
    int fd;
    struct stat st;
    FILE *fp;

    fd = open(path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) return NULL;
    if (fstat(fd, &st) != 0 || !S_ISREG(st.st_mode) || st.st_uid != getuid()) {
        (void)close(fd);
        return NULL;
    }
    fp = fdopen(fd, "rb");
    if (fp == NULL) (void)close(fd);
    return fp;
}

/*
 * Restore gamma ramps (and, for version >= 2 files, backlight values)
 * captured by a previous instance. dpy may be NULL for backlight-only
 * recovery when the X server is unreachable. Returns the number of restored
 * CRTCs, 0 when no usable file exists, or -1 for a corrupt file.
 */
static int restore_from_recovery_file(Display *dpy, Window root, const char *path,
                                      int *backlights_restored)
{
    FILE *fp;
    StateHeader header;
    XRRScreenResources *res = NULL;
    uint32_t i;
    uint32_t bl_count = 0U;
    int restored = 0;
    int bl_done = 0;

    if (backlights_restored != NULL) *backlights_restored = 0;
    fp = open_state_for_read(path);
    if (fp == NULL) return 0;
    if (fread(&header, sizeof(header), 1U, fp) != 1U ||
        memcmp(header.magic, STATE_MAGIC, sizeof(header.magic)) != 0 ||
        (header.version != 1U && header.version != STATE_VERSION) ||
        header.count > MAX_SAVED_CRTCS) {
        (void)fclose(fp);
        return -1;
    }
    if (dpy != NULL) res = XRRGetScreenResourcesCurrent(dpy, root);

    for (i = 0U; i < header.count; ++i) {
        StateCrtcHeader ch;
        XRRCrtcGamma *g;

        if (fread(&ch, sizeof(ch), 1U, fp) != 1U || ch.size == 0U || ch.size > 65536U) {
            if (res != NULL) XRRFreeScreenResources(res);
            (void)fclose(fp);
            return restored > 0 ? restored : -1;
        }
        g = XRRAllocGamma((int)ch.size);
        if (g == NULL) {
            if (res != NULL) XRRFreeScreenResources(res);
            (void)fclose(fp);
            return restored > 0 ? restored : -1;
        }
        if (fread(g->red, sizeof(unsigned short), ch.size, fp) != ch.size ||
            fread(g->green, sizeof(unsigned short), ch.size, fp) != ch.size ||
            fread(g->blue, sizeof(unsigned short), ch.size, fp) != ch.size) {
            XRRFreeGamma(g);
            if (res != NULL) XRRFreeScreenResources(res);
            (void)fclose(fp);
            return restored > 0 ? restored : -1;
        }

        if (res != NULL &&
            crtc_size_in(dpy, res, (RRCrtc)ch.crtc) == (int)ch.size) {
            XRRSetCrtcGamma(dpy, (RRCrtc)ch.crtc, g);
            restored++;
        }
        XRRFreeGamma(g);
    }

    if (header.version >= 2U &&
        fread(&bl_count, sizeof(bl_count), 1U, fp) == 1U &&
        bl_count <= MAX_BACKLIGHTS) {
        for (i = 0U; i < bl_count; ++i) {
            StateBacklight rec;
            if (fread(&rec, sizeof(rec), 1U, fp) != 1U) break;
            bl_done += restore_backlight_record(&rec);
        }
    }

    if (res != NULL) XRRFreeScreenResources(res);
    (void)fclose(fp);
    if (dpy != NULL) XSync(dpy, False);
    if (backlights_restored != NULL) *backlights_restored = bl_done;
    return restored;
}

static int set_identity_gamma(Display *dpy, Window root)
{
    XRRScreenResources *res;
    int i;
    int changed = 0;

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
    XSync(dpy, False);
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

static int apply_color_state(App *app)
{
    XRRScreenResources *res;
    size_t c;
    double strength = clamp_double(app->state.strength, 0.0, 1.0);
    double brightness = clamp_double(app->state.brightness, 0.01, 1.0);
    double blue_limit = clamp_double(app->state.blue_limit, 0.0, 1.0);
    double fr = brightness * ((1.0 - strength) + strength * clamp_double(app->state.r, 0.0, 1.0));
    double fg = brightness * ((1.0 - strength) + strength * clamp_double(app->state.g, 0.0, 1.0));
    double fb = brightness * ((1.0 - strength) + strength * clamp_double(app->state.b, 0.0, 1.0)) * blue_limit;
    int applied = 0;

    g_x_error_count = 0;
    res = XRRGetScreenResourcesCurrent(app->dpy, app->root);
    if (res == NULL) return -1;
    for (c = 0U; c < app->crtc_count; ++c) {
        XRRCrtcGamma *base = app->crtcs[c].original;
        XRRCrtcGamma *out;
        int i;
        if (base == NULL || base->size <= 0) continue;
        if (crtc_size_in(app->dpy, res, app->crtcs[c].id) != base->size) continue;
        out = XRRAllocGamma(base->size);
        if (out == NULL) continue;
        for (i = 0; i < base->size; ++i) {
            double rv = (double)base->red[i] * fr;
            double gv = (double)base->green[i] * fg;
            double bv = (double)base->blue[i] * fb;
            out->red[i] = (unsigned short)llround(clamp_double(rv, 0.0, 65535.0));
            out->green[i] = (unsigned short)llround(clamp_double(gv, 0.0, 65535.0));
            out->blue[i] = (unsigned short)llround(clamp_double(bv, 0.0, 65535.0));
        }
        XRRSetCrtcGamma(app->dpy, app->crtcs[c].id, out);
        XRRFreeGamma(out);
        applied++;
    }
    XRRFreeScreenResources(res);
    XSync(app->dpy, False);
    app->next_reassert_ms = now_ms() + REASSERT_MS;
    return (applied > 0 && g_x_error_count == 0) ? 0 : -1;
}

static void restore_backlights(App *app)
{
    size_t i;
    for (i = 0U; i < app->backlight_count; ++i) {
        Backlight *bl = &app->backlights[i];
        if (bl->changed && bl->writable) {
            if (write_long_file(bl->brightness_path, bl->original) == 0) {
                bl->changed = 0;
            }
        }
    }
}

static int restore_original(App *app)
{
    XRRScreenResources *res;
    size_t i;
    int restored = 0;

    g_x_error_count = 0;
    res = XRRGetScreenResourcesCurrent(app->dpy, app->root);
    if (res != NULL) {
        for (i = 0U; i < app->crtc_count; ++i) {
            XRRCrtcGamma *g = app->crtcs[i].original;
            if (g == NULL) continue;
            if (crtc_size_in(app->dpy, res, app->crtcs[i].id) != g->size) continue;
            XRRSetCrtcGamma(app->dpy, app->crtcs[i].id, g);
            restored++;
        }
        XRRFreeScreenResources(res);
    }
    XSync(app->dpy, False);

    noise_destroy(app);
    restore_backlights(app);
    reset_color_state(&app->state);
    return (restored > 0 && g_x_error_count == 0) ? 0 : -1;
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
        restore_backlights(g_app);
        (void)unlink(g_app->socket_path);
    }
    _exit(EXIT_RUNTIME);
}

/*
 * Re-synchronize the saved-CRTC table after a RandR topology change
 * (monitor hotplug, mode switch, output-to-CRTC re-assignment):
 *   - CRTC ids that no longer exist are dropped.
 *   - Existing entries whose gamma size still matches keep their pristine
 *     original ramps (so a monitor that is disabled and re-enabled is
 *     restored from the untouched capture, not from a tinted ramp).
 *   - Newly active CRTCs are captured as-is and adopted.
 * The recovery file is rewritten and the current tint is re-applied.
 */
static void topology_resync(App *app)
{
    XRRScreenResources *res;
    size_t i;
    int c;

    res = XRRGetScreenResourcesCurrent(app->dpy, app->root);
    if (res == NULL) return;

    i = 0U;
    while (i < app->crtc_count) {
        int found = 0;
        for (c = 0; c < res->ncrtc; ++c) {
            if (res->crtcs[c] == app->crtcs[i].id) {
                found = 1;
                break;
            }
        }
        if (found) {
            ++i;
        } else {
            remove_saved_crtc(app, i);
        }
    }

    for (c = 0; c < res->ncrtc; ++c) {
        XRRCrtcInfo *info = XRRGetCrtcInfo(app->dpy, res, res->crtcs[c]);
        int active, gamma_size;
        SavedCrtc *saved;
        size_t idx = 0U;
        XRRCrtcGamma *gamma;

        if (info == NULL) continue;
        active = (info->mode != None && info->noutput > 0) ? 1 : 0;
        XRRFreeCrtcInfo(info);
        if (!active) continue; /* keep any stale original for a possible return */

        gamma_size = XRRGetCrtcGammaSize(app->dpy, res->crtcs[c]);
        saved = find_saved_crtc(app, res->crtcs[c], &idx);
        if (saved != NULL && saved->original != NULL &&
            saved->original->size == gamma_size) {
            continue;
        }
        if (saved != NULL) remove_saved_crtc(app, idx);
        if (gamma_size <= 0 || gamma_size > 65536) continue;
        if (app->crtc_count >= MAX_SAVED_CRTCS) continue;

        gamma = XRRGetCrtcGamma(app->dpy, res->crtcs[c]);
        if (gamma == NULL) continue;
        if (gamma->size != gamma_size) {
            XRRFreeGamma(gamma);
            continue;
        }
        app->crtcs[app->crtc_count].id = res->crtcs[c];
        app->crtcs[app->crtc_count].original = gamma;
        app->crtc_count++;
    }
    XRRFreeScreenResources(res);

    (void)save_recovery_file(app);
    if (app->state.modified) (void)apply_color_state(app);
    noise_resize(app);
}

static int set_backlight_percent(App *app, double percent, char *response, size_t response_size)
{
    size_t i;
    int changed = 0;
    int available = 0;
    double p = clamp_double(percent, 1.0, 100.0) / 100.0;

    for (i = 0U; i < app->backlight_count; ++i) {
        Backlight *bl = &app->backlights[i];
        long target;
        if (!bl->writable) continue;
        available++;
        target = (long)llround((double)bl->maximum * p);
        if (target < 1L) target = 1L;
        if (target > bl->maximum) target = bl->maximum;
        if (write_long_file(bl->brightness_path, target) == 0) {
            bl->changed = 1;
            changed++;
        }
    }

    if (available == 0) {
        (void)snprintf(response, response_size,
                       "Hardware backlight is not writable for this user. "
                       "No privilege changes were attempted; software brightness remains available.");
        return -1;
    }
    if (changed == 0) {
        (void)snprintf(response, response_size, "No hardware backlight value could be changed.");
        return -1;
    }
    (void)snprintf(response, response_size, "Hardware backlight set to %.0f%% on %d device(s).", percent, changed);
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
 * comfort tinting; it is not a colorimetric CCT conversion. The result is
 * normalized so the strongest channel stays at 1.0 (luminance-preserving).
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
    (void)snprintf(s->mode, sizeof(s->mode), "%s", name);
    s->modified = 1;
    return 0;
}

/* Current hardware backlight of the first writable device, in percent, or
 * -1 when no writable device exists. */
static double first_backlight_percent(const App *app)
{
    size_t i;
    for (i = 0U; i < app->backlight_count; ++i) {
        const Backlight *bl = &app->backlights[i];
        long current;
        if (!bl->writable || bl->maximum <= 0) continue;
        if (read_long_file(bl->brightness_path, &current) != 0) continue;
        return 100.0 * (double)current / (double)bl->maximum;
    }
    return -1.0;
}

static void status_text(const App *app, char *response, size_t response_size)
{
    size_t i;
    int writable = 0;
    int changed = 0;
    for (i = 0U; i < app->backlight_count; ++i) {
        if (app->backlights[i].writable) writable++;
        if (app->backlights[i].changed) changed++;
    }
    (void)snprintf(response, response_size,
                   "mode=%s brightness=%.0f%% strength=%.0f%% rgb=%.0f/%.0f/%.0f%% "
                   "blue_limit=%.0f%% noise=%d%% kelvin=%.0f backlight=%.0f%% "
                   "active_crtcs=%zu backlights=%zu writable_backlights=%d changed_backlights=%d",
                   app->state.mode,
                   app->state.brightness * 100.0,
                   app->state.strength * 100.0,
                   app->state.r * 100.0,
                   app->state.g * 100.0,
                   app->state.b * 100.0,
                   app->state.blue_limit * 100.0,
                   app->noise.intensity,
                   app->state.kelvin,
                   first_backlight_percent(app),
                   app->crtc_count,
                   app->backlight_count,
                   writable,
                   changed);
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
    for (i = 0U; i < app->backlight_count; ++i) {
        Backlight *bl = &app->backlights[i];
        long current = -1L;
        if (read_long_file(bl->brightness_path, &current) == 0) {
            buf_append(response, response_size, &used, " %s=%ld/%ld%s",
                       bl->name, current, bl->maximum,
                       bl->writable ? "" : "(read-only)");
        } else {
            buf_append(response, response_size, &used, " %s=?/%ld%s",
                       bl->name, bl->maximum,
                       bl->writable ? "" : "(read-only)");
        }
    }
}

static int handle_command(App *app, const char *command, char *response, size_t response_size)
{
    char buf[MAX_COMMAND];
    char *tok[8];
    size_t ntok = 0U;
    char *save = NULL;
    char *p;

    if (strlen(command) >= sizeof(buf)) {
        (void)snprintf(response, response_size, "Command too long.");
        return -1;
    }
    (void)snprintf(buf, sizeof(buf), "%s", command);
    /* One command per connection: anything after the first line is ignored. */
    buf[strcspn(buf, "\r\n")] = '\0';
    p = strtok_r(buf, " \t", &save);
    while (p != NULL && ntok < (sizeof(tok) / sizeof(tok[0]))) {
        tok[ntok++] = p;
        p = strtok_r(NULL, " \t", &save);
    }
    if (ntok == 0U) {
        (void)snprintf(response, response_size, "Empty command.");
        return -1;
    }

    if (strcasecmp(tok[0], "status") == 0) {
        status_text(app, response, response_size);
        return 0;
    }
    if (strcasecmp(tok[0], "list") == 0) {
        list_text(app, response, response_size);
        return 0;
    }
    if (strcasecmp(tok[0], "normal") == 0) {
        if (restore_original(app) == 0) {
            (void)snprintf(response, response_size,
                           "Original display state restored (ramps, backlight, noise overlay).");
            return 0;
        }
        (void)snprintf(response, response_size, "Restoration was attempted, but one or more active CRTCs could not be restored.");
        return -1;
    }
    if (strcasecmp(tok[0], "stop") == 0) {
        (void)restore_original(app);
        (void)snprintf(response, response_size, "Original display state restored. Daemon stopping.");
        g_stop_requested = 1;
        return 0;
    }
    if (strcasecmp(tok[0], "preset") == 0 && ntok >= 2U) {
        if (strcasecmp(tok[1], "normal") == 0) {
            return handle_command(app, "normal", response, response_size);
        }
        if (set_preset(&app->state, tok[1]) != 0) {
            (void)snprintf(response, response_size,
                           "Unknown preset. Use green, green-soft, amber, red, pink, sepia, warm, low-blue, ultra-low-blue, zero-blue.");
            return -1;
        }
        if (apply_color_state(app) != 0) {
            (void)snprintf(response, response_size, "Preset selected, but XRandR could not apply it to every active CRTC.");
            return -1;
        }
        (void)snprintf(response, response_size, "Preset '%s' applied.", tok[1]);
        return 0;
    }
    if (strcasecmp(tok[0], "temp") == 0 && ntok >= 2U) {
        double kelvin;
        if (parse_number(tok[1], 1000.0, 10000.0, &kelvin) != 0) {
            (void)snprintf(response, response_size, "Color temperature must be 1000..10000 Kelvin.");
            return -1;
        }
        kelvin_to_rgb(kelvin, &app->state.r, &app->state.g, &app->state.b);
        app->state.strength = 1.0;
        app->state.kelvin = kelvin;
        app->state.modified = 1;
        (void)snprintf(app->state.mode, sizeof(app->state.mode), "temp-%.0fK", kelvin);
        if (apply_color_state(app) != 0) {
            (void)snprintf(response, response_size, "Temperature selected, but XRandR could not apply it to every active CRTC.");
            return -1;
        }
        (void)snprintf(response, response_size,
                       "Approximate white point set to %.0f K (blackbody approximation).", kelvin);
        return 0;
    }
    if (strcasecmp(tok[0], "brightness") == 0 && ntok >= 2U) {
        double value;
        if (parse_number(tok[1], 1.0, 100.0, &value) != 0) {
            (void)snprintf(response, response_size, "Software brightness must be 1..100 percent.");
            return -1;
        }
        app->state.brightness = value / 100.0;
        app->state.modified = 1;
        if (strcmp(app->state.mode, "normal") == 0) (void)snprintf(app->state.mode, sizeof(app->state.mode), "%s", "custom");
        if (apply_color_state(app) != 0) {
            (void)snprintf(response, response_size, "Brightness state changed, but XRandR could not apply it to every active CRTC.");
            return -1;
        }
        (void)snprintf(response, response_size, "Software brightness set to %.0f%%.", value);
        return 0;
    }
    if (strcasecmp(tok[0], "strength") == 0 && ntok >= 2U) {
        double value;
        if (parse_number(tok[1], 0.0, 100.0, &value) != 0) {
            (void)snprintf(response, response_size, "Tint strength must be 0..100 percent.");
            return -1;
        }
        app->state.strength = value / 100.0;
        app->state.modified = 1;
        if (apply_color_state(app) != 0) {
            (void)snprintf(response, response_size, "Strength state changed, but XRandR could not apply it to every active CRTC.");
            return -1;
        }
        (void)snprintf(response, response_size, "Tint strength set to %.0f%%.", value);
        return 0;
    }
    if (strcasecmp(tok[0], "blue") == 0 && ntok >= 2U) {
        double value;
        if (parse_number(tok[1], 0.0, 100.0, &value) != 0) {
            (void)snprintf(response, response_size, "Blue level must be 0..100 percent.");
            return -1;
        }
        app->state.r = 1.0;
        app->state.g = 1.0;
        app->state.b = value / 100.0;
        app->state.strength = 1.0;
        app->state.modified = 1;
        (void)snprintf(app->state.mode, sizeof(app->state.mode), "%s", "blue-control");
        if (apply_color_state(app) != 0) {
            (void)snprintf(response, response_size, "Blue state changed, but XRandR could not apply it to every active CRTC.");
            return -1;
        }
        (void)snprintf(response, response_size,
                       "Digital blue-channel level set to %.0f%% (this does not guarantee zero physical blue wavelengths).", value);
        return 0;
    }
    if (strcasecmp(tok[0], "color") == 0 && ntok >= 2U) {
        double r, g, b;
        double strength = 70.0;
        if (parse_hex_color(tok[1], &r, &g, &b) != 0) {
            (void)snprintf(response, response_size, "Color must be a non-black RRGGBB hex value, e.g. 33FF66 or FF66CC.");
            return -1;
        }
        if (ntok >= 3U && parse_number(tok[2], 0.0, 100.0, &strength) != 0) {
            (void)snprintf(response, response_size, "Optional color strength must be 0..100 percent.");
            return -1;
        }
        app->state.r = r;
        app->state.g = g;
        app->state.b = b;
        app->state.strength = strength / 100.0;
        app->state.modified = 1;
        (void)snprintf(app->state.mode, sizeof(app->state.mode), "%s", "custom-color");
        if (apply_color_state(app) != 0) {
            (void)snprintf(response, response_size, "Custom color selected, but XRandR could not apply it to every active CRTC.");
            return -1;
        }
        (void)snprintf(response, response_size, "Custom tint %s applied at %.0f%% strength.", tok[1], strength);
        return 0;
    }
    if (strcasecmp(tok[0], "rgb") == 0 && ntok >= 4U) {
        double r, g, b, strength = 100.0;
        if (parse_number(tok[1], 0.0, 100.0, &r) != 0 ||
            parse_number(tok[2], 0.0, 100.0, &g) != 0 ||
            parse_number(tok[3], 0.0, 100.0, &b) != 0) {
            (void)snprintf(response, response_size, "RGB values must each be 0..100 percent.");
            return -1;
        }
        if (ntok >= 5U && parse_number(tok[4], 0.0, 100.0, &strength) != 0) {
            (void)snprintf(response, response_size, "Optional RGB strength must be 0..100 percent.");
            return -1;
        }
        app->state.r = r / 100.0;
        app->state.g = g / 100.0;
        app->state.b = b / 100.0;
        app->state.strength = strength / 100.0;
        app->state.modified = 1;
        (void)snprintf(app->state.mode, sizeof(app->state.mode), "%s", "custom-rgb");
        if (apply_color_state(app) != 0) {
            (void)snprintf(response, response_size, "Custom RGB selected, but XRandR could not apply it to every active CRTC.");
            return -1;
        }
        (void)snprintf(response, response_size, "RGB tint %.0f/%.0f/%.0f%% applied at %.0f%% strength.", r, g, b, strength);
        return 0;
    }
    if (strcasecmp(tok[0], "backlight") == 0 && ntok >= 2U) {
        double value;
        if (parse_number(tok[1], 1.0, 100.0, &value) != 0) {
            (void)snprintf(response, response_size, "Hardware backlight must be 1..100 percent.");
            return -1;
        }
        return set_backlight_percent(app, value, response, response_size);
    }
    if ((strcasecmp(tok[0], "blue-limit") == 0 || strcasecmp(tok[0], "bluelimit") == 0) &&
        ntok >= 2U) {
        double value;
        if (parse_number(tok[1], 0.0, 100.0, &value) != 0) {
            (void)snprintf(response, response_size, "Blue limit must be 0..100 percent.");
            return -1;
        }
        app->state.blue_limit = value / 100.0;
        app->state.modified = 1;
        if (strcmp(app->state.mode, "normal") == 0) {
            (void)snprintf(app->state.mode, sizeof(app->state.mode), "%s", "custom");
        }
        if (apply_color_state(app) != 0) {
            (void)snprintf(response, response_size, "Blue limit changed, but XRandR could not apply it to every active CRTC.");
            return -1;
        }
        (void)snprintf(response, response_size,
                       "Blue-channel ceiling set to %.0f%%; it multiplies every preset, temperature and tint.", value);
        return 0;
    }
    if (strcasecmp(tok[0], "noise") == 0 && ntok >= 2U) {
        double value;
        char nerr[192];
        if (strcasecmp(tok[1], "off") == 0) {
            value = 0.0;
        } else if (parse_number(tok[1], 0.0, 100.0, &value) != 0) {
            (void)snprintf(response, response_size, "Noise intensity must be 0..100 percent (0 disables).");
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
                           "Golden-ratio noise at %.0f%% (13 frames, 89 ms cadence, server-side tiling).", value);
        }
        return 0;
    }

    (void)snprintf(response, response_size,
                   "Unknown command. Try: status, list, normal, preset NAME, color RRGGBB [strength], "
                   "rgb R G B [strength], temp KELVIN, blue PERCENT, blue-limit PERCENT, "
                   "strength PERCENT, brightness PERCENT, backlight PERCENT, noise PERCENT, stop.");
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

static void handle_client(App *app, int server_fd)
{
    char command[MAX_COMMAND];
    char response[MAX_RESPONSE];
    char wire[MAX_RESPONSE + 8];
    struct timeval tv;
    size_t used = 0U;
    ssize_t n;
    int wn;
    int result;
    int client_fd;

    client_fd = accept(server_fd, NULL, NULL);
    if (client_fd < 0) return;

    /* A stuck or hostile client must never block the daemon forever. */
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    (void)setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, (socklen_t)sizeof(tv));
    (void)setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, (socklen_t)sizeof(tv));

    while (used + 1U < sizeof(command)) {
        n = read(client_fd, command + used, sizeof(command) - used - 1U);
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (n == 0) break;
        used += (size_t)n;
        if (memchr(command, '\n', used) != NULL) break;
    }
    command[used] = '\0';
    result = handle_command(app, command, response, sizeof(response));
    wn = snprintf(wire, sizeof(wire), "%s %s\n", result == 0 ? "OK" : "ERR", response);
    if (wn > 0 && (size_t)wn < sizeof(wire)) {
        (void)write_all(client_fd, wire, (size_t)wn);
    }
    (void)close(client_fd);
}

static int daemon_loop(int notify_fd)
{
    App app;
    int server_fd = -1;
    int randr_error_base = 0;
    int major = 0, minor = 0;
    int recovery_result;
    struct sigaction sa;
    struct pollfd pfd[2];

    memset(&app, 0, sizeof(app));
    app.lock_fd = -1;
    reset_color_state(&app.state);
    g_app = &app;
    umask(0077);

    if (make_runtime_paths(app.socket_path, sizeof(app.socket_path),
                           app.recovery_path, sizeof(app.recovery_path),
                           app.lock_path, sizeof(app.lock_path)) != 0) {
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

    /* Recover exact pre-modification state after a prior unclean exit. */
    recovery_result = restore_from_recovery_file(app.dpy, app.root,
                                                 app.recovery_path, NULL);
    if (recovery_result > 0) (void)unlink(app.recovery_path);

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
        (void)restore_original(&app);
        (void)unlink(app.recovery_path);
        daemon_report(notify_fd, "ESocket\n");
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

    pfd[0].fd = server_fd;
    pfd[0].events = POLLIN;
    pfd[1].fd = ConnectionNumber(app.dpy);
    pfd[1].events = POLLIN;

    while (!g_stop_requested) {
        int pr;
        int timeout_ms = -1;
        uint64_t now;

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
        if (app.state.modified) {
            timeout_ms = deadline_delta(app.next_reassert_ms, now, timeout_ms);
        }

        pfd[0].revents = 0;
        pfd[1].revents = 0;
        pr = poll(pfd, 2U, timeout_ms);
        if (pr < 0) {
            if (errno == EINTR) continue;
            break;
        }
        now = now_ms();
        if (app.noise.intensity > 0 && now >= app.noise.next_frame_ms) {
            noise_frame_tick(&app);
        }
        if (app.state.modified && now >= app.next_reassert_ms) {
            (void)apply_color_state(&app);
        }
        if (pr > 0) {
            if (pfd[1].revents != 0) service_x_events(&app, 1);
            if (pfd[0].revents & POLLIN) handle_client(&app, server_fd);
        }
    }

    (void)restore_original(&app);
    (void)unlink(app.socket_path);
    (void)unlink(app.recovery_path);
    (void)close(server_fd);
    if (app.lock_fd >= 0) (void)close(app.lock_fd);
    free_saved_crtcs(&app);
    g_app = NULL;
    XCloseDisplay(app.dpy);
    return EXIT_OK;
}

static int start_daemon(const char *socket_path, char *err, size_t err_size)
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
        _exit(daemon_loop(pipefd[1]));
    }

    close(pipefd[1]);
    while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
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
    ROW_BLUE_LIMIT,
    ROW_BLUE,
    ROW_BRIGHT,
    ROW_STRENGTH,
    ROW_NOISE,
    ROW_BACKLIGHT,
    ROW_COUNT
} TuiRow;

typedef struct {
    double kelvin;
    double blue_limit;
    double blue;
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
    { "Blue limit",           0.0,   100.0,   5.0,  20.0 },
    { "Blue channel",         0.0,   100.0,   5.0,  20.0 },
    { "Brightness (soft)",    1.0,   100.0,   5.0,  20.0 },
    { "Tint strength",        0.0,   100.0,   5.0,  20.0 },
    { "Noise",                0.0,   100.0,   5.0,  20.0 },
    { "Backlight (hw)",       1.0,   100.0,   5.0,  20.0 },
};

static TuiTerm *g_tui_term = NULL;
static volatile sig_atomic_t g_tui_winch = 0;

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

/* Ctrl+Z: hand a sane terminal back to the shell before actually stopping. */
static void on_tstp(int sig)
{
    (void)sig;
    tui_raw_leave(g_tui_term);
    (void)signal(SIGTSTP, SIG_DFL);
    (void)raise(SIGTSTP);
}

static void on_cont(int sig)
{
    (void)sig;
    if (g_tui_term != NULL && g_tui_term->saved_valid) {
        (void)tui_raw_enter(g_tui_term);
    }
    (void)signal(SIGTSTP, on_tstp);
    g_tui_winch = 1;
}

static int tui_read_key(void)
{
    unsigned char c;
    ssize_t n;

    n = read(STDIN_FILENO, &c, 1U);
    if (n == 0) return -1;
    if (n < 0) return errno == EINTR ? TKEY_NONE : -1;
    if (c != 0x1bU) return (int)c;

    {
        struct pollfd p;
        unsigned char seq0 = 0U, seq1 = 0U, seq2 = 0U;

        p.fd = STDIN_FILENO;
        p.events = POLLIN;
        p.revents = 0;
        if (poll(&p, 1U, 50) <= 0) return TKEY_NONE; /* lone ESC: ignore */
        if (read(STDIN_FILENO, &seq0, 1U) != 1) return TKEY_NONE;
        if (seq0 != '[' && seq0 != 'O') return TKEY_NONE;
        if (read(STDIN_FILENO, &seq1, 1U) != 1) return TKEY_NONE;
        switch (seq1) {
        case 'A': return TKEY_UP;
        case 'B': return TKEY_DOWN;
        case 'C': return TKEY_RIGHT;
        case 'D': return TKEY_LEFT;
        case '5':
        case '6':
            if (read(STDIN_FILENO, &seq2, 1U) != 1) return TKEY_NONE;
            (void)seq2;
            return seq1 == '5' ? TKEY_PGUP : TKEY_PGDN;
        default:
            return TKEY_NONE;
        }
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

    if (send_daemon_command(socket_path, "status", resp, sizeof(resp)) != 0) return -1;
    if (strncmp(resp, "OK ", 3U) != 0) return -1;

    v->kelvin = 6500.0;
    v->blue_limit = 100.0;
    v->blue = 100.0;
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
        p += 4;
        (void)strtod(p, &end);
        if (end != p && *end == '/') {
            p = end + 1;
            (void)strtod(p, &end);
            if (end != p && *end == '/') {
                double b;
                p = end + 1;
                b = strtod(p, &end);
                if (end != p) v->blue = b;
            }
        }
    }
    return 0;
}

static double tui_row_value(const TuiVals *v, int row)
{
    switch (row) {
    case ROW_TEMP: return v->kelvin;
    case ROW_BLUE_LIMIT: return v->blue_limit;
    case ROW_BLUE: return v->blue;
    case ROW_BRIGHT: return v->brightness;
    case ROW_STRENGTH: return v->strength;
    case ROW_NOISE: return v->noise;
    case ROW_BACKLIGHT: return v->backlight;
    default: return 0.0;
    }
}

static void tui_row_command(int row, double value, char *cmd, size_t size)
{
    switch (row) {
    case ROW_TEMP: (void)snprintf(cmd, size, "temp %.0f", value); break;
    case ROW_BLUE_LIMIT: (void)snprintf(cmd, size, "blue-limit %.0f", value); break;
    case ROW_BLUE: (void)snprintf(cmd, size, "blue %.0f", value); break;
    case ROW_BRIGHT: (void)snprintf(cmd, size, "brightness %.0f", value); break;
    case ROW_STRENGTH: (void)snprintf(cmd, size, "strength %.0f", value); break;
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
        (void)snprintf(msg, msg_size, "%s", "Could not reach the daemon.");
        return;
    }
    p = resp;
    if (strncmp(p, "OK ", 3U) == 0) p += 3;
    else if (strncmp(p, "ERR ", 4U) == 0) p += 4;
    /* The message line is one row tall: keep only the first 200 bytes. */
    (void)snprintf(msg, msg_size, "%.200s", p);
    msg[strcspn(msg, "\r\n")] = '\0';
}

static void tui_draw(const TuiVals *v, int sel_row, const char *msg, int utf8, int bl_available)
{
    char out[8192];
    size_t used = 0U;
    const char *hz = utf8 ? "\xe2\x94\x80" : "-";       /* ─ */
    const char *fill = utf8 ? "\xe2\x96\x88" : "#";     /* █ */
    const char *empty = utf8 ? "\xe2\x96\x91" : ".";    /* ░ */
    const char *arrow = utf8 ? "\xe2\x86\x92" : ">";    /* → */
    int row, i;

    buf_append(out, sizeof(out), &used, "\x1b[H");
    buf_append(out, sizeof(out), &used, " \x1b[1m%s %s\x1b[0m - total display control\x1b[K\r\n",
               APP_NAME, APP_VERSION);
    buf_append(out, sizeof(out), &used, " ");
    for (i = 0; i < 60; ++i) buf_append(out, sizeof(out), &used, "%s", hz);
    buf_append(out, sizeof(out), &used, "\x1b[K\r\n");
    buf_append(out, sizeof(out), &used, " Mode: %s\x1b[K\r\n\x1b[K\r\n", v->mode);

    for (row = 0; row < ROW_COUNT; ++row) {
        double val, span;
        int filled;
        char valstr[32];

        if (row == ROW_BACKLIGHT && !bl_available) continue;
        val = tui_row_value(v, row);
        span = TUI_ROWS[row].max - TUI_ROWS[row].min;
        filled = (int)llround((val - TUI_ROWS[row].min) / span * 24.0);
        if (filled < 0) filled = 0;
        if (filled > 24) filled = 24;

        if (row == ROW_TEMP) {
            (void)snprintf(valstr, sizeof(valstr), "%5.0f K", val);
        } else if (row == ROW_NOISE && val <= 0.0) {
            (void)snprintf(valstr, sizeof(valstr), "%s", "   off");
        } else {
            (void)snprintf(valstr, sizeof(valstr), "%4.0f %%", val);
        }

        buf_append(out, sizeof(out), &used, " %s %s%-18s ",
                   row == sel_row ? arrow : " ",
                   row == sel_row ? "\x1b[7m" : "",
                   TUI_ROWS[row].label);
        for (i = 0; i < 24; ++i) {
            buf_append(out, sizeof(out), &used, "%s", i < filled ? fill : empty);
        }
        buf_append(out, sizeof(out), &used, " %s%s\x1b[K\r\n",
                   valstr, row == sel_row ? "\x1b[0m" : "");
    }

    buf_append(out, sizeof(out), &used, "\x1b[K\r\n");
    buf_append(out, sizeof(out), &used,
               " Presets: 1 green  2 green-soft  3 amber  4 red  5 pink\x1b[K\r\n");
    buf_append(out, sizeof(out), &used,
               "          6 sepia  7 warm  8 low-blue  9 ultra-low  0 zero-blue\x1b[K\r\n");
    buf_append(out, sizeof(out), &used,
               " Keys: %s select   %s adjust   PgUp/PgDn coarse   n normal   q quit\x1b[K\r\n",
               utf8 ? "\xe2\x86\x91\xe2\x86\x93" : "Up/Down",
               utf8 ? "\xe2\x86\x90\xe2\x86\x92" : "Left/Right");
    buf_append(out, sizeof(out), &used, " ");
    for (i = 0; i < 60; ++i) buf_append(out, sizeof(out), &used, "%s", hz);
    buf_append(out, sizeof(out), &used, "\x1b[K\r\n");
    buf_append(out, sizeof(out), &used, " %.76s\x1b[K\r\n", msg);
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
    if (start_daemon(socket_path, err, sizeof(err)) != 0) {
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
    sa.sa_handler = on_tstp;
    (void)sigaction(SIGTSTP, &sa, NULL);
    sa.sa_handler = on_cont;
    (void)sigaction(SIGCONT, &sa, NULL);

    memset(&term, 0, sizeof(term));
    g_tui_term = &term;
    if (tui_raw_enter(&term) != 0) {
        g_tui_term = NULL;
        fprintf(stderr, "Could not switch the terminal to raw mode.\n");
        return EXIT_RUNTIME;
    }
    (void)snprintf(msg, sizeof(msg), "%s",
                   "Every adjustment applies instantly. 'q' leaves the look active.");

    for (;;) {
        int bl_available = vals.backlight >= 0.0 ? 1 : 0;
        int vis[ROW_COUNT];
        int nvis = 0;
        int key;
        int row;

        for (row = 0; row < ROW_COUNT; ++row) {
            if (row == ROW_BACKLIGHT && !bl_available) continue;
            vis[nvis++] = row;
        }
        if (sel >= nvis) sel = nvis - 1;
        if (sel < 0) sel = 0;

        tui_draw(&vals, vis[sel], msg, utf8, bl_available);
        key = tui_read_key();
        if (g_stop_requested || key == -1) break;
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
            tui_row_command(r2, nv, cmd, sizeof(cmd));
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
    printf("  %s backlight PERCENT      hardware backlight, 1..100 (if writable)\n", argv0);
    printf("  %s noise PERCENT          golden-ratio CRT noise, 0..100 (needs a compositor)\n", argv0);
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
    printf("  %s normal\n\n", argv0);
    printf("Notes:\n");
    printf("  * Commands automatically start the background daemon when needed.\n");
    printf("  * 'normal' restores the exact gamma ramps captured at daemon startup.\n");
    printf("  * Monitor hotplug and mode changes re-apply the current tint automatically.\n");
    printf("  * 'backlight' only writes sysfs devices already writable by your user.\n");
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

static int emergency_reset(const char *socket_path, const char *recovery_path)
{
    Display *dpy;
    Window root = None;
    char response[MAX_RESPONSE];
    int recovered;
    int backlights = 0;
    int identity = 0;

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

    recovered = restore_from_recovery_file(dpy, root, recovery_path, &backlights);
    if (recovered > 0 || backlights > 0) {
        printf("Recovered %d CRTC gamma ramp(s) and %d backlight value(s) from the recovery file.\n",
               recovered > 0 ? recovered : 0, backlights);
        (void)unlink(recovery_path);
        if (dpy != NULL) XCloseDisplay(dpy);
        return EXIT_OK;
    }

    if (dpy != NULL) {
        identity = set_identity_gamma(dpy, root);
        XCloseDisplay(dpy);
    }
    if (identity > 0) {
        printf("Recovery file was unavailable; set %d active CRTC(s) to identity gamma.\n", identity);
        printf("Note: identity gamma may not reproduce a custom ICC calibration.\n");
        return EXIT_OK;
    }
    fprintf(stderr, "Could not reset any XRandR CRTC or backlight device.\n");
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

    if (start_daemon(socket_path, err, sizeof(err)) != 0) {
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
    int rc;

    /* A peer disappearing during IPC must become an error, never SIGPIPE. */
    (void)signal(SIGPIPE, SIG_IGN);

    if (make_runtime_paths(socket_path, sizeof(socket_path),
                           recovery_path, sizeof(recovery_path),
                           lock_path, sizeof(lock_path)) != 0) {
        fprintf(stderr, "Could not create runtime paths.\n");
        return EXIT_RUNTIME;
    }

    if (argc < 2) {
        print_help(argv[0]);
        return EXIT_OK;
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
        return emergency_reset(socket_path, recovery_path);
    }
    if (strcmp(argv[1], "foreground") == 0) {
        return daemon_loop(-1);
    }
    if (strcmp(argv[1], "tui") == 0) {
        return run_tui(socket_path);
    }
    if (strcmp(argv[1], "interactive") == 0) {
        return run_interactive(socket_path);
    }
    if (strcmp(argv[1], "start") == 0) {
        if (start_daemon(socket_path, err, sizeof(err)) != 0) {
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
    if (start_daemon(socket_path, err, sizeof(err)) != 0) {
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
