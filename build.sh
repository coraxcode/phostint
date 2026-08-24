#!/bin/sh
set -eu

APP="phostint"
SRC="phostint.c"

show_deps() {
    cat >&2 <<'MSG'
Install a C compiler plus the X11/XRandR/Xext development packages.
Common examples:
  Debian / Ubuntu: sudo apt install build-essential pkg-config libx11-dev libxrandr-dev libxext-dev
  Fedora:          sudo dnf install gcc pkgconf-pkg-config libX11-devel libXrandr-devel libXext-devel
  Arch Linux:      sudo pacman -S --needed base-devel pkgconf libx11 libxrandr libxext
  openSUSE:        sudo zypper install gcc pkg-config libX11-devel libXrandr-devel libXext-devel
  Alpine:          sudo apk add build-base pkgconf libx11-dev libxrandr-dev libxext-dev

This script never installs packages automatically and never uses sudo itself.
MSG
}

if [ ! -f "$SRC" ]; then
    echo "Error: $SRC was not found in the current directory." >&2
    echo "Put build.sh and phostint.c in the same folder, then run ./build.sh" >&2
    exit 1
fi

if ! command -v cc >/dev/null 2>&1; then
    echo "Error: no C compiler ('cc') was found." >&2
    show_deps
    exit 1
fi

CFLAGS=""
LIBS="-lX11 -lXrandr -lXext"

# pkg-config is preferred, but it is not mandatory if the normal development
# headers/libraries are already installed in standard system locations.
if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists x11 xrandr xext 2>/dev/null; then
    CFLAGS="$(pkg-config --cflags x11 xrandr xext)"
    LIBS="$(pkg-config --libs x11 xrandr xext)"
fi

# Hardening flags are probed one by one: each is used only when the local
# compiler accepts it, so the build never fails on an unusual toolchain.
HARDEN=""
for flag in -fstack-protector-strong -fstack-clash-protection \
            '-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=2'; do
    if printf 'int main(void){return 0;}\n' | \
       cc -std=c11 -O2 $flag -x c - -o /dev/null 2>/dev/null; then
        HARDEN="$HARDEN $flag"
    fi
done

# Intentionally no -Werror: compiler-version-specific warnings should not make
# an otherwise portable build fail on a different Linux distribution.
if ! cc -std=c11 -O2 \
   -D_POSIX_C_SOURCE=200809L \
   -Wall -Wextra -Wpedantic -Wshadow \
   -Wstrict-prototypes -Wconversion -Wsign-conversion \
   -Wformat=2 \
   $HARDEN \
   $CFLAGS "$SRC" $LIBS -lm -o "$APP"; then
    echo >&2
    echo "Build failed. The most common cause is missing X11/XRandR development files." >&2
    show_deps
    exit 1
fi

chmod 0755 "$APP"

echo "Build complete: ./$APP"
echo "Easy setup:     ./$APP interactive"
echo "Restore colors: ./$APP normal"
echo "Stop + restore: ./$APP stop"
