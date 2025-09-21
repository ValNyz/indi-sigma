# indi-sigma

INDI CCD driver for Sigma cameras backed by a custom PTP for Sigma camera library : [sigma-driver](https://github.com/ValNyz/sigma-driver).

## Features
- [x] USB PTP connection
- [x] Discrete exposures (no BULB), nearest-time clamping
- [x] ISO and exposure preset control
- [x] JPEG path: decode to 8-bit mono and publish via `ExposureComplete`
- [x] RAW path: DNG → 16-bit FITS with libraw + cfitsio
- [ ] Error states and logs compatible with Ekos
- [ ] Capture target selection: SD Card / RAM / Both
- [x] Live View streaming
- [ ] Bulb mode

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

## Troubleshooting

- Check udev permissions for the Sigma VID/PID so the process can access the USB device.
- Enable driver debug in Control Panel and run `indiserver -vvv indi_sigma_ccd` for logs.
- Don't hesitate to open an issue and i will try to fix any problem.
