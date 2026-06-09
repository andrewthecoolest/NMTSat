#!/bin/bash
# setup.sh — install all dependencies for nmtsat-sdr on a Debian/Ubuntu host.
# Alternative to Docker (init.sh). Run once: ./setup.sh
#
# By default uses distro packages for librtlsdr/bladeRF (fast, includes udev
# rules). Pass --from-source to build them from upstream git instead
set -euo pipefail

FROM_SOURCE=0
[[ "${1:-}" == "--from-source" ]] && FROM_SOURCE=1

if ! command -v apt-get >/dev/null 2>&1; then
  echo "This script targets Debian/Ubuntu (apt). For other distros, install:"
  echo "  build-essential cmake pkg-config git libusb-1.0 libfftw3 libcurl libncurses"
  echo "  librtlsdr-dev libbladerf-dev python3 numpy matplotlib"
  exit 1
fi

echo ">> Installing build tools and libraries via apt..."
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake pkg-config git \
  libusb-1.0-0-dev \
  libfftw3-dev \
  libcurl4-openssl-dev \
  libncurses-dev \
  python3 python3-pip python3-dev \
  usbutils \
  curl ca-certificates

if [[ "$FROM_SOURCE" -eq 0 ]]; then
  echo ">> Installing librtlsdr + bladeRF from distro packages..."
  sudo apt-get install -y librtlsdr-dev rtl-sdr libbladerf-dev bladerf
else
  echo ">> Building librtlsdr from source..."
  tmp="$(mktemp -d)"
  git clone https://github.com/librtlsdr/librtlsdr.git "$tmp/rtl-sdr"
  cmake -S "$tmp/rtl-sdr" -B "$tmp/rtl-sdr/build" \
    -DINSTALL_UDEV_RULES=ON -DDETACH_KERNEL_DRIVER=ON
  sudo cmake --build "$tmp/rtl-sdr/build" --target install -j"$(nproc)"

  echo ">> Building bladeRF from source..."
  git clone https://github.com/Nuand/bladeRF.git "$tmp/bladeRF"
  cmake -S "$tmp/bladeRF/host" -B "$tmp/bladeRF/host/build"
  sudo cmake --build "$tmp/bladeRF/host/build" --target install -j"$(nproc)"

  sudo ldconfig
  rm -rf "$tmp"
fi

echo ">> Installing Python deps..."
pip3 install --user --no-cache-dir numpy matplotlib

echo ""
echo ">> Done. Build with:"
echo "     cmake -S . -B build -DCMAKE_BUILD_TYPE=Release"
echo "     cmake --build build -j\$(nproc)"
echo ""
echo "   Note: USB devices need udev rules + group membership (plugdev/dialout)."
echo "   You may need to unplug/replug SDRs, or run the build with sudo, after install."
