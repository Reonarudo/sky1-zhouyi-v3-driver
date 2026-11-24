# sky1-zhouyi-v3-driver

Linux kernel driver (`aipu`) for the **Zhouyi V3 NPU** (X2_1204MP3: 3 cores × 4 TECs) in the
**CIX Sky1 / P1** SoC (CD8180 / CD8160).

It is derived from CIX's open-source driver,
[cix_opensource__npu_driver](https://github.com/cixtech/cix_opensource__npu_driver), release
**6.2.0-1** (branch `cix_mainline_dev`, the "P1 KERNEL 7.0" release). CIX's history is kept as-is
up to that release; the commits after it are ours. The code for other NPU generations and other
SoCs is removed, and a set of fixes and additions sits on top (see below).

This project is not affiliated with CIX or Arm China.

> ⚠️ **Work in progress.** This is the driver we develop the NPU examples against, published as it
> stands. It is in daily use on one board and kernel (below), but its interface additions, defaults
> and kernel support may still change, and it has not been tested beyond that setup. Please report
> what you find in an [issue](https://github.com/Reonarudo/sky1-zhouyi-v3-driver/issues).

## Kernel support

| Kernel | Status |
| --- | --- |
| `7.0.0-41-cix` (CIX P1 kernel 7.0 BSP) | in daily use |
| `6.6.89-cix` (vendor 6.6 images) | builds, and DKMS-installs in CI; **not yet run on a 6.6 kernel** — reports welcome |

## Install

Download `cix-npu-driver-dkms_<version>_all.deb` from the
[Releases](https://github.com/Reonarudo/sky1-zhouyi-v3-driver/releases) page (or the `package`
artifact of a CI run). DKMS builds the module for every kernel whose headers are installed, so
install the headers for your running kernel first.

```sh
sudo apt install ./cix-npu-driver-dkms_<version>_all.deb
```

**If another `aipu` driver is already installed** (the vendor images ship one as DKMS module
`aipu/<version>`), remove it first so the two don't install the same `aipu.ko`:

```sh
dkms status aipu                     # list installed versions
dpkg -S /usr/src/aipu-*              # which package owns each one
sudo apt remove <that-package>       # or: sudo dkms remove aipu/<version> --all
```

Then reboot (or `sudo modprobe -r aipu && sudo modprobe aipu`) and check that the loaded module
is this one: `dkms status aipu` lists only this package's version, `modinfo -F filename aipu`
points into `updates/dkms/`, and `dmesg | grep -i aipu` shows the probe.

To go back to the vendor driver, remove `cix-npu-driver-dkms` and reinstall the vendor package.

## Build from source

The Debian package (needs `debhelper`, `dh-exec`, `dh-dkms`):

```sh
dpkg-buildpackage -us -uc -b         # writes ../cix-npu-driver-dkms_<version>_all.deb
```

Only the module, against the running kernel's headers (`/lib/modules/$(uname -r)/build`):

```sh
cd driver && make                    # writes driver/aipu.ko
make COMPASS_DRV_BTENVAR_KPATH=/path/to/linux-headers   # another kernel
```

## What differs from CIX 6.2.0-1

- **Zhouyi V3 on Sky1 only.** Android, other SoC backends, the V1 / V2 / V3_2 architecture
  backends, `CONFIG_COMPAT` and unused config knobs are removed. The two crash-dump ioctls
  return `-EINVAL`.
- **Hardening.** Probe and teardown unwind correctly; the hard-IRQ path no longer sleeps;
  ioctls validate their inputs; every caller-supplied address must belong to a buffer the
  calling fd owns; exported dma-bufs have a proper lifecycle; register accesses the hardware
  cannot service are refused; the command-pool interrupt mask is restored after runtime resume.
- **Job completion.** A job whose own completion interrupt was not seen is retired only once
  the command pool has drained. The vendor rule (retire every earlier job when a later one
  completes) retired jobs that were still running when groups ran in parallel on several cores.
- **Job deadline.** A job that neither completes nor faults within `job_deadline_s` is reported
  to userspace as an exception instead of leaving the caller waiting forever.
- **Cache maintenance.** The cache ioctls operate on the pages' linear-map alias, the one that
  can hold stale lines; the per-completion whole-fd flush is gone (callers invalidate what
  they read).
- **Zero-copy weights.** New WBUF / SLOT ioctls (see below).
- **Faster lookups.** Buffer and TCB lookups use rb-trees instead of walking every buffer on
  each submit; IOVA allocation for ASIDs 1–3 is fixed.

## Userspace interface

`driver/armchina-npu/include/armchina_aipu.h` is CIX 6.2.0's header (identical to CIX 6.0.1's)
plus five ioctls:

| Ioctl | Nr | Purpose |
| --- | --- | --- |
| `AIPU_IOCTL_WBUF_ALLOC` / `WBUF_FREE` | 30 / 31 | host memory for weights, in 2 MiB units outside the NPU window, `mmap()`ed write-combine |
| `AIPU_IOCTL_SLOT_ALLOC` / `SLOT_FREE` | 32 / 33 | a 2 MiB-aligned, initially unmapped range of ASID 0's window |
| `AIPU_IOCTL_SLOT_MAP` | 34 | point a slot at a range of a weight buffer with SMMU block mappings, replacing a copy (a 411 MB layer: ~16 µs against ~40 ms) |

⚠️ **Userspace built against the 5.11.x header will not work with this driver.** That header
(the driver on the vendor 6.6 images) has `asid_base[32]` in `struct aipu_cap`, so its
`AIPU_IOCTL_QUERY_CAP` encodes a different size and the call fails. Userspace built for CIX
6.0.1 or 6.2.0 is unaffected.

**Address layout** (CIX 6.2.0's, unchanged): four ASIDs, each a 4 GiB slot of which the lower
3 GiB is usable; ASID *n*'s NPU-view base is *n* × 4 GiB. Pointers inside a TCB are 32-bit
offsets from the ASID 0 base.

## Module parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `job_deadline_s` | `30` | seconds before an outstanding job is reported as an exception; `0` disables |
| `wbuf_force_small` | `0` | weight buffers always use 4 KiB pages (tests the fallback) |

## CI and releases

`.github/workflows/build.yml` builds the Debian package, then installs it in containers carrying
the CIX `6.6.89-cix` and `7.0.0` kernel headers (pinned by checksum) and has DKMS build the
module for each. Pushing a tag `v<package version>` (e.g. `v6.2.0-1+zv3.1`, matching
`debian/changelog`) publishes a release with the `.deb` attached.

## License

GPL-2.0 — see `driver/LICENSE.TXT`. Original code © Arm Technology (China) Co. Ltd. and Cix
Technology Group Co., Ltd.; changes © Leonardo Marques.
