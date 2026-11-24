#!/bin/sh
# Install the built DKMS package next to a pinned kernel-headers package and
# have DKMS build the module for that kernel -- the same path an install on a
# board takes. Usage: dkms-test.sh <package.deb>
# Env: KHDR_URL, KHDR_SHA256 (the kernel headers .deb, arm64)
set -eu
export DEBIAN_FRONTEND=noninteractive
PKG="$(realpath "$1")"
apt-get update -qq
apt-get install -y -qq --no-install-recommends ca-certificates curl dkms make gcc libc6-dev bc kmod libelf1 >/dev/null

cd "$(mktemp -d)"
curl -fsSLo headers.deb "$KHDR_URL"
echo "$KHDR_SHA256  headers.deb" | sha256sum -c -
# The vendor headers depend on packages (e.g. a specific libssl) a stock
# container may not carry; only the kbuild tree is needed here.
dpkg -i --force-depends headers.deb >/dev/null
KVER="$(dpkg-deb -f headers.deb Package | sed 's/^linux-headers-//')"
[ -d "/usr/src/linux-headers-$KVER" ] || { echo "headers not at /usr/src/linux-headers-$KVER" >&2; exit 1; }
mkdir -p "/lib/modules/$KVER"
[ -e "/lib/modules/$KVER/build" ] || ln -s "/usr/src/linux-headers-$KVER" "/lib/modules/$KVER/build"

# The postinst tries the running (CI host) kernel, whose headers are absent; that is expected.
dpkg -i "$PKG" || true
VER="$(dpkg-deb -f "$PKG" Version)"
dkms status
dkms build "aipu/$VER" -k "$KVER"
KO="$(find "/var/lib/dkms/aipu/$VER/$KVER" -name 'aipu.ko*' | head -1)"
[ -n "$KO" ] || { echo "dkms produced no aipu.ko" >&2; exit 1; }
modinfo "$KO" | grep -E '^(filename|version|vermagic|license):'
