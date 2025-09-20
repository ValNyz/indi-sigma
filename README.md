# indi-sigma

INDI CCD driver for Sigma cameras backed by a custom PTP for Sigma camera library : [sigma-driver](https://github.com/ValNyz/sigma-driver).

## Features
- USB PTP connect using libptp_sigma
- Discrete exposures (no BULB)
- ISO and exposure preset control
- Image download to client or local disk
- RAW path: DNG → 16‑bit FITS with libraw + cfitsio

## Build

Dependencies:
- libindi
- libusb-1.0
- libjpeg
- libraw
- cfitsio
- libptp_sigma (no installation needed, source code obtained from git)

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
sudo make install
```

## Run

```bash
indiserver -v indi_sigma_ccd
```

Connect from KStars/Ekos and select **Sigma DSLR**.

## Implementation notes

- The driver publishes pixel buffers for JPEG and can publish FITS BLOBs for RAW.
- `CCD_DOWNLOAD_TIMEOUT` adds extra seconds to wait for the file after exposure.
- LiveView hooks exist but require support in libptp_sigma.

## Troubleshooting

- Check udev permissions for the Sigma VID/PID so the process can access the USB device.
- Enable driver debug in Control Panel and run `indiserver -vvv indi_sigma_ccd` for logs.
- Don't hesitate to open an issue and i will try to fix any problem.
