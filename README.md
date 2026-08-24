# PhosTint

Total display control for X11 — tint, color temperature, blue-light blocking,
brightness, and a golden-ratio CRT noise overlay. Built for i3wm and any other
X11 window manager, on any Linux distribution.

Single C11 source file. Runtime dependencies: `libX11`, `libXrandr`, `libXext`.
No configuration files, no root, no sudo, no DDC/CI, no direct hardware access.

## Features

- **Full-screen TUI** (`phostint tui`): arrow-key sliders for temperature,
  blue limit, blue channel, brightness, strength, noise and backlight, plus
  one-key presets. Pure VT100 — no ncurses dependency, works in any terminal,
  ASCII fallback when the locale is not UTF-8.
- **Blue-light control, three independent ways**:
  - `temp 1000..10000` — Kelvin white point (blackbody curve), like
    redshift/gammastep but with exact state restoration;
  - `blue 0..100` — direct digital blue-channel level, down to a true 0%;
  - `blue-limit 0..100` — a multiplicative blue ceiling that **composes with
    every other mode** (preset, temperature, custom tint), so you can cap blue
    on top of any look. Neither redshift, gammastep nor f.lux offers this.
- **Presets**: green, green-soft, amber, red, pink, sepia, warm, low-blue,
  ultra-low-blue, zero-blue — freely combinable with `noise`, `brightness`,
  `strength` and `blue-limit` (e.g. a green-phosphor CRT:
  `phostint preset green && phostint noise 35`).
- **Custom tints**: any hex color (`color FF66CC 80`) or per-channel
  percentages (`rgb 100 60 10 90`).
- **Golden-ratio noise overlay** (`noise 0..100`): animated CRT grain built
  entirely from the golden ratio — 377×233 Fibonacci golden-rectangle tiles
  (377/233 = 1.618), 13 frames, 89 ms cadence (both Fibonacci), pixels placed
  by Knuth's multiplicative hash `0x9E3779B9 = floor(2^32/phi)`, densities and
  alphas as powers of phi (0.382 = 1/phi², 0.618 = 1/phi). Tiles are rendered
  once and animated **server-side**: the per-frame cost is three tiny X
  requests. Measured: **0.2% of one core** at 50% intensity, 0% idle.
- **Software brightness** (1..100%, gamma-based) and **hardware backlight**
  (only via `/sys/class/backlight`, and only when your user already has write
  permission — PhosTint never escalates privileges).
- **Hotplug-aware**: monitor connect/disconnect and mode switches re-apply the
  current look automatically; re-enabled monitors are restored from their
  pristine capture.
- **Self-healing**: gamma is re-asserted once a minute (drivers that reset
  ramps on suspend/resume recover automatically); the noise overlay shuts
  itself down within ~1.2 s if the compositor disappears.
- **Crash-safe**: exact original gamma ramps and backlight values are
  persisted before the first modification; `emergency-reset` recovers them
  even after `kill -9` or an X restart.

## Build

```bash
./build.sh
```

The script uses `pkg-config` when available, probes hardening flags
(`-fstack-protector-strong`, `-fstack-clash-protection`, `_FORTIFY_SOURCE=2`)
and only applies the ones your compiler accepts. Development packages:

| Distribution | Packages |
|---|---|
| Debian / Ubuntu | `build-essential pkg-config libx11-dev libxrandr-dev libxext-dev` |
| Fedora | `gcc pkgconf-pkg-config libX11-devel libXrandr-devel libXext-devel` |
| Arch Linux | `base-devel pkgconf libx11 libxrandr libxext` |
| openSUSE | `gcc pkg-config libX11-devel libXrandr-devel libXext-devel` |
| Alpine | `build-base pkgconf libx11-dev libxrandr-dev libxext-dev` |

Verify the internal math without any X server (useful for packaging/CI):

```bash
./phostint selftest
```

## Usage

The easiest path is the panel:

```bash
./phostint tui
```

Any mutating command auto-starts the background daemon; you rarely need
`start` explicitly.

```bash
./phostint preset warm        # warm low-blue tint
./phostint temp 3400          # candle-ish white point in Kelvin
./phostint blue 0             # digital blue channel fully off
./phostint blue-limit 50      # cap blue at 50% of any current/future mode
./phostint color 33FF66 80    # custom hex tint at 80% strength
./phostint rgb 100 60 10 90   # per-channel tint
./phostint noise 35           # golden-ratio CRT grain (needs a compositor)
./phostint brightness 45      # software brightness (gamma-based)
./phostint backlight 60       # hardware backlight, only if writable
./phostint status             # one-line state (machine-readable key=value)
./phostint list               # outputs, gamma sizes, backlight devices
./phostint normal             # restore the exact captured original state
./phostint stop               # restore and stop the daemon
./phostint interactive        # plain question-and-answer menu
./phostint emergency-reset    # recovery without a running daemon
```

Exit codes: `0` success, `2` usage error, `3` runtime failure.

### The noise overlay needs a compositor

True per-pixel transparency requires a compositing manager. On i3wm the usual
choice is picom:

```
exec --no-startup-id picom -b
```

Without one, `noise` refuses cleanly and explains why (everything else works
compositor-free). If the compositor dies while noise is active, PhosTint
detects it within ~1.2 s and removes the overlay automatically, so the screen
can never stay black. The overlay is click-through (empty SHAPE input region):
it never steals clicks, scrolling or focus. It will appear in screenshots and
recordings — that is inherent to overlays; turn it off when capturing.

If your picom config uses background blur, exclude the overlay:
`blur-background-exclude = [ "name = 'PhosTint noise overlay'" ];`

## i3wm integration

```
exec --no-startup-id picom -b
exec --no-startup-id /path/to/phostint start

bindsym $mod+F9  exec --no-startup-id /path/to/phostint preset warm
bindsym $mod+F10 exec --no-startup-id /path/to/phostint temp 3400
bindsym $mod+F11 exec --no-startup-id /path/to/phostint brightness 40
bindsym $mod+F12 exec --no-startup-id /path/to/phostint normal
bindsym $mod+Shift+F12 exec --no-startup-id /path/to/phostint emergency-reset
bindsym $mod+F8  exec alacritty -e /path/to/phostint tui
```

(Replace `alacritty -e` with your terminal's exec flag.) A mode block keeps
all display controls behind one prefix key:

```
set $display_mode Display: [w]arm [t]emp [g]reen [c]rt [b]lue-cut [n]ormal [Esc]
mode "$display_mode" {
    bindsym w exec --no-startup-id phostint preset warm,           mode "default"
    bindsym t exec --no-startup-id phostint temp 3400,             mode "default"
    bindsym g exec --no-startup-id phostint preset green,          mode "default"
    bindsym c exec --no-startup-id sh -c 'phostint preset green && phostint noise 35', mode "default"
    bindsym b exec --no-startup-id phostint preset ultra-low-blue, mode "default"
    bindsym n exec --no-startup-id phostint normal,                mode "default"
    bindsym Escape mode "default"
}
bindsym $mod+d mode "$display_mode"
```

## Safety and recovery model

- On startup the daemon captures the **existing** per-CRTC gamma ramps (which
  include any ICC/xcalib calibration already loaded) and the current backlight
  values, then writes them to a private recovery file **before** the first
  modification. All tints are computed from that pristine capture, never
  stacked on previous tints, so repeated commands cannot drift or accumulate
  rounding error.
- `normal`, `stop`, SIGINT/SIGTERM/SIGHUP/SIGQUIT, and daemon shutdown restore
  the exact captured state (ramps, backlight, and the noise overlay).
- If the X server dies, the IO error handler still restores any backlight
  values this process changed.
- `emergency-reset` tries, in order: a running daemon (via socket, 10 s
  timeout), the recovery file (gamma **and** backlight), and finally identity
  gamma. It works even when X is unreachable (backlight-only recovery).
- A per-`(uid, DISPLAY)` `fcntl` lock guarantees a single daemon instance;
  concurrent `start` races resolve to one daemon; the lock dies with the
  process, so a crashed daemon never blocks a new one.
- Software brightness is floored at 1% and hex `000000` is rejected, so the
  screen cannot be made fully black by mistake.
- All client commands time out after 10 s — a wedged daemon can never hang
  your shell or the TUI.

Runtime files live in `XDG_RUNTIME_DIR` (or a private `0700` fallback
directory `/tmp/phostint-<uid>`), mode `0600`:

```
phostint-<uid>-<display-hash>.sock    control socket
phostint-<uid>-<display-hash>.state   recovery file (gamma + backlight)
phostint-<uid>-<display-hash>.lock    instance lock (never deleted)
```

The recovery file is only trusted when it is a regular file owned by the
current user (opened with `O_NOFOLLOW`); backlight device names from the file
are validated and rebuilt under `/sys/class/backlight/` so a corrupt file can
never redirect writes elsewhere.

## Performance

Measured on this codebase (Intel laptop, 1920×1080):

- Idle daemon: **0% CPU** — fully event-driven `poll()` loop, no polling.
- Noise at 50%: **0.2% of one core** — tiles are pre-rendered server-side;
  each 89 ms frame costs three X requests regardless of screen size.
- Tint/temperature changes: one `XRRSetCrtcGamma` per CRTC, plus a once-a-
  minute re-assertion (a single request per CRTC) while a look is active.

## Conflicts with other tools

Do **not** run PhosTint at the same time as redshift, gammastep, xflux,
GNOME Night Light, or KDE Night Color: all of them write the same XRandR gamma
ramps and will fight each other. Load ICC calibration (xcalib, colord, argyll)
**before** starting PhosTint — the calibration becomes part of the captured
"original", is preserved inside every tint, and is restored by `normal`.

## Honest physical limitations

- XRandR gamma ramps scale the R, G, B channels independently; they cannot mix
  channels. Custom colors are therefore **tints**, not a true monochrome
  matrix transform.
- `temp` uses a standard blackbody curve fit (Tanner Helland, 2012) — ideal
  for comfort, not a colorimetric CCT conversion.
- **Digital blue at 0% does not mean zero physical blue light.** Panel
  backlights (WLED, quantum dot), subpixel filters and spectral leakage keep
  emitting some energy in blue wavelengths. `blue 0` and `zero-blue` remove
  every digitally addressable trace of blue; verifying physical output
  requires a spectroradiometer.
- Identical digital settings do not produce identical physical color or
  luminance on different panels.
- Hardware backlight writability depends on your distribution's udev/logind
  policy. If `list` shows `(read-only)`, your user lacks write permission to
  `/sys/class/backlight/*/brightness`; many distributions grant it via the
  `video` group or `systemd-logind`. PhosTint deliberately never changes
  permissions itself.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `cannot open the X11 display` | Not an X11 session, or `DISPLAY` unset. Wayland is not supported (its compositors own gamma). |
| `no active CRTC with readable gamma ramps` | Driver/virtual display without RandR gamma (some VMs, Xvfb). Check `list` / `xrandr --verbose`. |
| `Noise unavailable: a compositing manager is required` | Start picom (`picom -b`) or any EWMH compositor, then retry. |
| Screen stuck tinted after a crash | `./phostint emergency-reset` |
| `backlight` says not writable | See the udev/logind note above; software `brightness` always works. |
| Colors fight/flicker | Another gamma tool is running (redshift, night light) — stop one of them. |
| TUI shows `+`/`#` instead of arrows/blocks | Non-UTF-8 locale; the ASCII fallback is automatic and fully functional. |
