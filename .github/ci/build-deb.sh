#!/bin/sh
# Build the DKMS package (cix-npu-driver-dkms_*.deb) from the repository root.
# Runs in a Debian container; writes the .deb to ${OUT:-dist}/.
set -eu
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq --no-install-recommends build-essential debhelper dh-exec dh-dkms >/dev/null
OUT="${OUT:-dist}"
dpkg-buildpackage -us -uc -b
mkdir -p "$OUT"
mv ../cix-npu-driver-dkms_*.deb "$OUT"/
ls -l "$OUT"
