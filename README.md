# PhosTint

Total display control for X11 — tint, color temperature, blue-light blocking,
brightness, and a golden-ratio CRT noise overlay. Built for i3wm and any other
X11 window manager, on any Linux distribution.

Single C11 source file. Runtime dependencies: `libX11`, `libXrandr`, `libXext`
(plus `libXrender` if present, only to detect ARGB visuals exactly).
No configuration files, no root, no sudo, no DDC/CI, no direct hardware access.

## Features

- **Full-screen TUI** (`phostint tui`): arrow-key sliders for temperature,
  the red/green/blue channels, tint strength, blue limit, brightness, noise
  and backlight, plus one-key presets. The R/G/B rows show and edit the
  **live per-channel percentages** (exactly what the display receives), so
  you can dial in any color mix directly — start from a preset and tweak one
  channel without disturbing the others. Pure VT100 — no ncurses dependency,
  works in any terminal, ASCII fallback when the locale is not UTF-8, and
  modified keys (Ctrl/Shift/Alt + arrows, Home, Delete, F-keys) are parsed
  or ignored safely.
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
  permission — PhosTint never escalates privileges). Exactly one device is
  driven per command: the kernel-preferred one (`raw` over `platform` over
  `firmware`), or the one you name with `backlight 60 intel_backlight`.
- **Per-monitor control**: prefix any color command with `output NAME` to aim
  it at a single connector (`phostint output HDMI-1 temp 3000`). Each monitor
  keeps its own look; commands without the prefix apply to all of them and
  become the default for monitors plugged in later.
- **Hotplug-aware**: monitor connect/disconnect and mode switches re-apply the
  current look automatically. Saved ramps are keyed by connector name, not
  only by CRTC id, so a monitor that the X server moves to a different CRTC
  keeps its own pristine capture and its own look.
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
./phostint backlight 60 acpi_video0   # a specific backlight device
./phostint output HDMI-1 temp 3000    # one monitor only
./phostint output HDMI-1 normal       # restore just that monitor
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
- The recovery journal is parsed and validated **completely** before any of it
  is applied, so a truncated or corrupted file is refused as a whole instead of
  being half-written to your monitors. It is deleted only after a fully
  successful restoration.
- Ramps in the journal are keyed by **EDID hash and connector name**, not by
  CRTC id, so they are matched to the physical monitor they came from even
  after the X server renumbers everything. Matching is strictly one-to-one and
  runs strongest-evidence-first (EDID, then connector, then CRTC id), so two
  identical panels can never both claim the same saved ramp. Swapping a
  different monitor into the same port re-captures instead of inheriting the
  previous monitor's ramp. In a clone/mirror setup the identity folds in every
  output on the CRTC, not just the first.
- A ramp that cannot be applied yet — its monitor is unplugged right now — is
  **carried forward** into the new journal and re-applied automatically the
  moment that monitor comes back. Starting a new daemon can never discard a
  pristine ramp that is still owed to some screen.
- The journal is stamped with a boot identifier, so a stale file left in the
  `/tmp` fallback by a previous boot is refused instead of replayed.
- Backlight changes are **write-ahead**: the journal records the level you had
  *before* the hardware is touched, so a crash in between still restores it.
  Only devices PhosTint actually changed are recorded, the "original" level is
  re-read immediately before the first change (so another tool's adjustment is
  not undone), and a device whose `max_brightness` changed is left alone
  because the stored raw level would no longer mean the same thing. A level
  still owed by a crashed instance outranks the current one: it is the value
  from before PhosTint ever touched that device, and it survives you moving
  the brightness by hand in the meantime.
- The journal is sized to hold everything the program can hold at once — every
  managed output plus every ramp owed to a disconnected monitor. If recovery
  data still could not be written in full, the write **fails** rather than
  silently producing a safety net with a hole in it, and every caller then
  declines to touch the hardware.
- A failed RandR enumeration is never mistaken for "no monitors are on". That
  distinction is what stops `normal` from reporting a clean restore while the
  screen is still tinted.
- Applying a look to several monitors is a transaction: every ramp is built
  and validated first, the batch is committed together, and if the X server
  rejects any part of it the whole batch is rolled back — to the look you had
  *before* the command, not to the untinted original, and the in-memory state
  is rolled back with it. A rejected command leaves no trace: no half-changed
  screen, and no status line that disagrees with what you see.
- An output that appears between the last journal write and an apply is
  journalled before anything is painted on it, so a crash can never turn a
  tint into the new "original".
- If the journal exists but cannot be read or parsed, the daemon **refuses to
  start**. That is the one case where it cannot know whether the ramps on
  screen are pristine or already tinted, and guessing would silently make a
  tint permanent. The file is kept (renamed `.corrupt`), and you choose:
  `phostint emergency-reset --identity` for a neutral baseline, or
  `phostint start --accept-current` to accept what is on screen. A journal
  that merely does not exist is *not* this case — "unreadable" and "absent"
  are distinguished, including permission errors and symlink substitution.
- An over-long control command is rejected outright, never executed
  truncated: `brightness 100000…` cut short would otherwise become a valid,
  completely different command.
- `emergency-reset` tries, in order: a running daemon (via socket, 10 s
  timeout), then the journal (gamma **and** backlight). It works even when X
  is unreachable (backlight-only recovery). If neither is usable it stops and
  tells you so: forcing a neutral ramp on **every** connected monitor would
  also wipe an ICC calibration loaded by another tool, so that needs an
  explicit `phostint emergency-reset --identity`.
- A silent or hostile client cannot stall anything: connections are
  non-blocking, live in the same event loop as everything else, and are
  dropped after 5 s. The noise animation and the periodic re-assert keep
  running throughout.
- Every operation reports honestly. `normal` and `stop` only claim success
  when every active output and every changed backlight really went back;
  otherwise they return a non-zero exit code and say what failed. Non-fatal
  problems appear in `phostint status` as `warning="..."`.
- A per-`(uid, DISPLAY)` `fcntl` lock guarantees a single daemon instance;
  concurrent `start` races resolve to one daemon; the lock dies with the
  process, so a crashed daemon never blocks a new one.
- The screen can never be made fully black by mistake: software brightness is
  floored at 1%, hex `000000` is rejected, and `rgb` refuses to drop all
  three channels below 1% at once.
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

## What is verified, and on what

Being precise about this matters more than claiming universal support.

**Automatically verified on every build** — `./phostint selftest` runs 113
hardware-free checks: the Kelvin curve and its monotonicity; hex/number
parsing and rejection; the noise generator (determinism, premultiplied alpha,
golden-ratio density); the command table's arity rules; the monitor-identity
rules (EDID beats connector name beats CRTC id); the hotplug decision policy
(keep / transfer / re-capture); the journal loader against every truncation
offset plus bad magic, wrong version, foreign boot id, unterminated strings,
out-of-range counts and trailing garbage; the whole backlight subsystem
against a simulated sysfs tree (type preference, one-device-per-command,
write-ahead journalling, re-reading the original level, refusing a changed
`max_brightness`, refusing to touch hardware when the journal is unwritable);
the colour-factor math; and the poll/scheduling helpers.

**Verified on real hardware**: Intel (i915, `intel_backlight`, eDP-1,
1024-entry gamma ramps), Xorg 21 with i3wm, single monitor, with and without
picom.

Stress-tested clean, both natively and under AddressSanitizer +
UndefinedBehaviorSanitizer + LeakSanitizer, with **zero sanitizer findings**:
520 sequential commands, 85 concurrent clients, 300 random/malformed command
lines, 140 hostile socket sessions (raw binary payloads, over-long commands,
truncated writes, connect-and-abort), plus the full command surface. The
daemon stayed responsive throughout and exited cleanly every time. The TUI
layout is checked at 108 terminal geometries from 16x3 to 200x40 — no line
ever exceeds the window. Carrying a pristine ramp forward for a disconnected
monitor is verified with a synthetic journal containing an absent display, and
the corrupt-journal refusal is verified end to end.

**Not verified by the author** (the code is written for it, but no hardware
was available): AMD and NVIDIA drivers, multi-monitor and multi-GPU setups,
physical hotplug, non-1024 gamma ramp sizes, big-endian machines, musl libc.
The multi-monitor *policy* and the recovery *format* are unit-tested, but the
X server interaction on more than one screen is not. If you hit a problem
there, `phostint status` ends with a `warning="..."` field and `phostint list`
shows every output and backlight device — please include both.

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
  for comfort, not a colorimetric CCT conversion. Its result is normalized so
  the strongest channel is 1.0, which is **not** luminance preservation: a
  warm white point genuinely lowers measured luminance because blue energy is
  removed. Use `brightness` when you want an explicit luminance control.
- Wayland is not supported and is not planned here: its compositors own the
  colour pipeline, and doing this properly needs a different backend
  (`wlr-gamma-control` / `color-management-v1`) rather than a port of this
  XRandR code.
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
| `Too many arguments. Usage: ...` | Arity is strict on purpose — a typo is reported instead of being silently ignored. |
| `status` ends with `warning="..."` | A non-fatal problem was recorded (for example a recovery-file write failure or a missing output). |
| `backlight` changes the wrong device | Run `list`: the `*` marks the default target (firmware is preferred, per the kernel ABI). Name the one you want: `backlight 60 intel_backlight`. |
| `emergency-reset` says there is nothing to restore | No journal survived. Add `--identity` only if you accept a neutral ramp on every monitor. |
| `status` shows `pending_ramps=N` | N monitors are disconnected and still owed their pristine ramp; plug them back and it is applied automatically. |
| `status` shows `attention="..."` | A condition only you can resolve — normally an unreadable journal after a crash. It is never cleared automatically. |
| Window too small to draw | The TUI adapts down to 24x4 and shows `window too small` below that. |
