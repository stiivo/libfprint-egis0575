# libfprint driver for EgisTec EH575 (1c7a:0575)

A working [libfprint](https://gitlab.freedesktop.org/libfprint/libfprint) driver for the **EgisTec EH575** swipe fingerprint sensor (USB ID `1c7a:0575`), found e.g. in the Acer Swift SF114-34. Upstream libfprint lists this sensor as unsupported.

The driver is based on the work of topni1 in [libfprint MR !357](https://gitlab.freedesktop.org/libfprint/libfprint/-/merge_requests/357), with fixes that make matching reliable and keep fprintd from hanging.

> **Status:** works for me with fprintd (enroll, verify, sudo/login via PAM). Tested on one device only. Use at your own risk. A fingerprint on a swipe sensor is convenient, but no replacement for a good password.

## Branches

| Branch | Content |
|---|---|
| `main` | This README and Fedora packaging (`fedora/`) |
| `egis0575` | MR !357 + fixes, on top of current libfprint `master` |
| `egis0575-fedora` | The same driver on top of libfprint `v1.94.100` (the version in Fedora 44) |

## Changes compared to MR !357

- Movement estimation in the driver (high-pass filter + normalized cross correlation) instead of `fpi_do_movement_estimation()`, which got confused by the brightness gradient of the sensor
- Strips are captured without delay and short swipes are processed instead of discarded
- Only the high-contrast central rows of each strip are used, with normalized contrast
- Images are upscaled 2× before minutiae detection, the ridges are too close for NBIS otherwise
- 8 enroll stages
- Sensor reset before reading the calibration, so the device can be opened again after a session
- Fixed hangs: the capture loop never completed, and a stop or an error left the device stuck in fprintd
- USB interface claim is checked and released again if opening fails
- `EGIS0575_DUMP_DIR=/some/dir` dumps the raw strips of each swipe for debugging

On my device, with 8 enrolled swipes, the same finger scored 15–43 (threshold 15) and all other fingers 0–4.

## Installation on Fedora

```sh
git clone https://github.com/stiivo/libfprint-egis0575.git
cd libfprint-egis0575
./fedora/build-rpm.sh

sudo rpm -Uvh ~/rpmbuild/RPMS/x86_64/libfprint-1.94.100-1.3.egis0575.fc*.x86_64.rpm
sudo systemctl restart fprintd
fprintd-enroll      # swipe the finger 8 times, slowly and centered
fprintd-verify
```

The release `1.3.egis0575` is newer than Fedora's `1.94.100-1`, so dnf treats it as an update. A later Fedora update of libfprint would replace it again. To prevent that:

```sh
sudo dnf config-manager setopt excludepkgs=libfprint
```

With this exclude, dnf filters out the local package too, so updates of this package have to be installed with `sudo rpm -Uvh`.

To use the fingerprint for sudo and login:

```sh
sudo authselect enable-feature with-fingerprint
```

To go back to the official package:

```sh
sudo dnf config-manager unsetopt excludepkgs
sudo dnf distro-sync libfprint
```

## Other distributions

Build libfprint from the `egis0575` branch (or apply `fedora/libfprint-egis0575.patch` to libfprint 1.94.100) with meson as usual, e.g.:

```sh
git checkout egis0575
meson setup build -Ddrivers=all
ninja -C build
sudo ninja -C build install
```

## License

The driver is part of libfprint and licensed under the LGPL-2.1-or-later, see the libfprint branches.
