#include "sigma_ccd.h"
#include <algorithm>
#include <cstring>
#include <indilogger.h>
#include <indipropertyswitch.h>
#include <jpeglib.h>
#include <setjmp.h>
#include <libraw/libraw.h>

#include <indilogger.h>

// avoid macro clashes within this TU only
#undef LOG_DEBUG
#undef LOG_INFO
#undef LOG_WARN
#undef LOG_ERROR

#include "utils/log.h"

// Build items, bind to existing PropertySwitch, define it.
// Assumes you already called: prop.fill(dev, name, label, group, IP_RW, ISR_1OFMANY, 60, IPS_IDLE)
static inline ISwitchVectorProperty *
bindSwitchVector(INDI::PropertySwitch &prop,
                 std::vector<ISwitch> &items,
                 const std::vector<std::string> &labels)
{
  ISwitchVectorProperty *svp = prop.getSwitch();

  items.resize(labels.size());
  for (size_t i = 0; i < labels.size(); ++i)
  {
    std::string key = "S_" + std::to_string(i);
    IUFillSwitch(&items[i], key.c_str(), labels[i].c_str(), ISS_OFF);
  }

  // Rebind using the metadata already present on svp
  IUFillSwitchVector(svp, items.data(), items.size(),
                     svp->device, svp->name, svp->label,
                     svp->group, svp->p, svp->r, svp->timeout, svp->s);
  return svp;
}

// Convenience wrapper: build from values + formatter // fmt(T)->std::string
template <class T, class F>
static inline ISwitchVectorProperty *bindSwitchVectorFormat(INDI::PropertySwitch &prop,
                                                            std::vector<ISwitch> &items,
                                                            const std::vector<T> &vals, F fmt)
{
  std::vector<std::string> labs;
  labs.reserve(vals.size());
  for (const auto &v : vals)
    labs.push_back(fmt(v));
  return bindSwitchVector(prop, items, labs);
}

// index of ON item, or -1
inline int switchSelectedIndex(const INDI::PropertySwitch &p)
{
  ISwitchVectorProperty *svp = const_cast<INDI::PropertySwitch &>(p).getSwitch();
  return svp ? IUFindOnSwitchIndex(svp) : -1;
}

// label of ON item, or empty
inline std::string switchSelectedLabel(const INDI::PropertySwitch &p)
{
  ISwitchVectorProperty *svp = const_cast<INDI::PropertySwitch &>(p).getSwitch();
  int i = svp ? IUFindOnSwitchIndex(svp) : -1;
  return (i >= 0) ? std::string(svp->sp[i].label) : std::string();
}

// map ON index → value vector (ISO, speeds). Returns fallback if none.
template <class T>
inline T switchSelectedValue(const INDI::PropertySwitch &p,
                             const std::vector<T> &vals,
                             T fallback)
{
  int i = switchSelectedIndex(p);
  return (i >= 0 && static_cast<size_t>(i) < vals.size()) ? vals[i] : fallback;
}

// JPEG error handler
struct jpeg_error_mgr_wrapper
{
  struct jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
};

static void jpeg_error_exit(j_common_ptr cinfo)
{
  jpeg_error_mgr_wrapper *myerr = (jpeg_error_mgr_wrapper *)cinfo->err;
  longjmp(myerr->setjmp_buffer, 1);
}

static std::unique_ptr<SigmaCCD> sigmaCameraInstance(new SigmaCCD());

SigmaCCD::SigmaCCD() : exposureRetries(0), InExposure(false)
{
  cam_ = std::make_unique<SigmaBackend>();
  bind_logger();
}

const char *SigmaCCD::getDefaultName() { return "Sigma DSLR"; }

bool SigmaCCD::initProperties()
{
  INDI::CCD::initProperties();

  ISOSP.fill(getDeviceName(), "CCD_ISO", "ISO", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
  ExposurePresetSP.fill(getDeviceName(), "CCD_EXPOSURE_PRESETS", "Presets", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60,
                        IPS_IDLE);

  CaptureTargetSP[InCamera].fill("SD Card", "SD Card", ISS_ON);
  CaptureTargetSP[InComputer].fill("RAM", "RAM", ISS_OFF);
  CaptureTargetSP[Both].fill("Both", "Both", ISS_OFF);
  CaptureTargetSP.fill(getDeviceName(), "CCD_CAPTURE_TARGET", "Capture Target", IMAGE_SETTINGS_TAB, IP_RW, ISR_1OFMANY, 0,
                       IPS_IDLE);
  CaptureTargetSP.load();

  // Download Timeout
  DownloadTimeoutNP[0].fill("VALUE", "Seconds", "%.f", 0, 30, 5, 5);
  DownloadTimeoutNP.fill(getDeviceName(), "CCD_DOWNLOAD_TIMEOUT", "Download Timeout", OPTIONS_TAB, IP_RW, 60, IPS_IDLE);
  DownloadTimeoutNP.load();

  PrimaryCCD.setMinMaxStep("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", 0.001, 30.0, 1, false);

  // Most cameras have this by default, so let's set it as default.
  BayerTP[2].setText("RGGB");

  // Set capabilities
  uint32_t cap = CCD_CAN_ABORT | CCD_HAS_STREAMING | CCD_HAS_BAYER; // TODO Not sure about this, should check each capability.
  SetCCDCapability(cap);

  PrimaryCCD.getCCDInfo().setPermission(IP_RW);

  setDriverInterface(getDriverInterface());

  // Add Debug, Simulator, and Configuration controls
  addAuxControls();

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SigmaCCD::ISGetProperties(const char *dev)
{
  INDI::CCD::ISGetProperties(dev);

  if (isConnected())
    return;

  // Read Image Info if we have not connected yet.
  double pixel = 0, pixel_x = 0, pixel_y = 0;
  IUGetConfigNumber(getDeviceName(), "CCD_INFO", "CCD_PIXEL_SIZE", &pixel);
  IUGetConfigNumber(getDeviceName(), "CCD_INFO", "CCD_PIXEL_SIZE_X", &pixel_x);
  IUGetConfigNumber(getDeviceName(), "CCD_INFO", "CCD_PIXEL_SIZE_Y", &pixel_y);

  auto nvp = PrimaryCCD.getCCDInfo();

  if (!nvp.isValid())
    return;

  // Load the necessary pixel size information
  // The maximum resolution and bits per pixel depend on the capture itself.
  // while the pixel size data remains constant.
  if (pixel > 0)
    nvp[INDI::CCDChip::CCD_PIXEL_SIZE].setValue(pixel);
  if (pixel_x > 0)
    nvp[INDI::CCDChip::CCD_PIXEL_SIZE_X].setValue(pixel_x);
  if (pixel_y > 0)
    nvp[INDI::CCDChip::CCD_PIXEL_SIZE_Y].setValue(pixel_y);
}

bool SigmaCCD::updateProperties()
{
  INDI::CCD::updateProperties();

  if (isConnected())
  {
    if (cam_->supportedExposures().size() > 0)
    {
      auto svp = bindSwitchVectorFormat(ExposurePresetSP, ExposurePresetItems, cam_->supportedExposures(), formatExposure);
      defineProperty(svp);
    }
    if (cam_->supportedISOs().size() > 0)
    {
      auto svp = bindSwitchVectorFormat(ISOSP, ISOItems, cam_->supportedISOs(), formatISO);
      defineProperty(svp);
    }
    defineProperty(CaptureTargetSP);

    imageBP = getBLOB("CCD1");

    defineProperty(DownloadTimeoutNP);

    CaptureFormat jpeg = {"JPEG", "JPEG", 8, false};
    CaptureFormat dng12bit = {"DNG12bit", "DNG 12bit", 12, false};
    CaptureFormat dng14bit = {"DNG14bit", "DNG 14bit", 14, true};
    addCaptureFormat(jpeg);
    addCaptureFormat(dng12bit);
    addCaptureFormat(dng14bit);

    // Update camera info
    if (auto res = cam_->sensorResolution())
    {
      // TODO: False, pixel size is 6 for FP and i don't know for FP L
      SetCCDParams(res->first, res->second, 12, 6, 6);
    }
    else
      // Dummy values until first capture is done
      SetCCDParams(1280, 1024, 8, 5.4, 5.4);

    defineProperty(UploadSP);
    defineProperty(UploadSettingsTP);

    LOGF_INFO("Connected to %s (S/N: %s, FW: %s)", cam_->model().c_str(),
              cam_->serialNumber().c_str(), cam_->firmwareVersion().c_str());

    Streamer->setPixelFormat(INDI_MONO, 8);

    // Set FPS limits if needed
    INumberVectorProperty *streamExp = getNumber("STREAM_EXPOSURE");
    if (streamExp)
    {
      streamExp->np[0].min = 0.04;
      streamExp->np[0].max = 1;
      streamExp->np[0].value = 0.1;
    }
  }
  else
  {
    deleteProperty(ExposurePresetSP);
    deleteProperty(ISOSP);
    deleteProperty(CaptureTargetSP);
    deleteProperty(DownloadTimeoutNP);
  }

  return true;
}

bool SigmaCCD::Connect()
{
  char **options;
  int max_opts;

  if (isSimulation())
  {
    LOGF_INFO("Simulation mode - connected to virtual %s", getDefaultName());
    return true;
  }

  if (!cam_->open(getDefaultName()))
  {
    LOG_ERROR("Failed to connect to Sigma camera");
    return false;
  }

  if (isSimulation())
  {
    PrimaryCCD.setMinMaxStep("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", 0.001, 3600, 1, true);
  }
  else
  {
    auto exposures = cam_->supportedExposures();
    double min_exp = *std::min_element(exposures.begin(), exposures.end());
    double max_exp = *std::max_element(exposures.begin(), exposures.end());
    PrimaryCCD.setMinMaxStep("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", min_exp, max_exp, 1, true);
  }

  const char *fmts[] = {"Custom"};
  // setidx             = 0;
  max_opts = 1;
  options = const_cast<char **>(fmts);

  if (!isSimulation())
  {
    // cache the BLOB vector handle
    imageBP = getBLOB("CCD1");
  }

  SetTimer(getCurrentPollingPeriod());

  return true;
}

bool SigmaCCD::Disconnect()
{
  if (!isSimulation())
  {
    cam_->close();
  }

  LOGF_INFO("%s disconnected.", getDeviceName());
  return true;
}

bool SigmaCCD::StartExposure(float duration)
{
  if (InExposure)
  {
    LOG_ERROR("Camera is already exposing");
    return false;
  }

  // Clamp to nearest supported exposure
  double clamped = clampToSupported(duration);
  if (std::abs(clamped - duration) > 0.01)
  {
    LOGF_INFO("Adjusted exposure from %.3fs to nearest supported %.3fs",
              duration, clamped);
  }

  ExposureRequest = duration;
  PrimaryCCD.setExposureDuration(clamped);

  // Configure camera settings
  if (!isSimulation())
  {
    // Set ISO
    int iso = switchSelectedValue(ISOSP, cam_->supportedISOs(), defaultISO);
    cam_->setISO(iso);
    // Set exposure time
    cam_->setExposureSeconds(clamped);
    cam_->setDownloadTimeout(DownloadTimeoutNP[0].value);
    // Set image format
    int formatIndex = IUFindOnSwitchIndex(CaptureFormatSP);
    bool useJpeg = (formatIndex == 0);
    cam_->setImageQuality(useJpeg);
    if (useJpeg)
      PrimaryCCD.setBPP(8);
    else
      PrimaryCCD.setBPP(16);
    PrimaryCCD.setFrameType(INDI::CCDChip::LIGHT_FRAME);
    cam_->setDestination(IUFindOnSwitchIndex(CaptureTargetSP));

    // Trigger capture
    if (!cam_->triggerCapture())
    {
      LOG_ERROR("Failed to trigger capture");
      return false;
    }
  }

  InExposure = true;
  gettimeofday(&ExpStart, nullptr);

  int formatIndex = IUFindOnSwitchIndex(CaptureFormatSP);
  LOGF_INFO("Started %.3fs exposure (ISO %d, %s format)", clamped,
            switchSelectedValue(ISOSP, cam_->supportedISOs(), defaultISO),
            formatIndex == 0 ? "JPEG" : "RAW");

  return true;
}

bool SigmaCCD::AbortExposure()
{
  if (!InExposure)
    return true;

  InExposure = false;

  if (!isSimulation())
  {
    cam_->abortCapture();
  }

  LOG_INFO("Exposure aborted");
  return true;
}

bool SigmaCCD::UpdateCCDFrame(int x, int y, int w, int h)
{
  // Store the subframe for later use
  PrimaryCCD.setFrame(x, y, w, h);
  return true;
}

void SigmaCCD::TimerHit()
{
  if (!isConnected())
    return;

  if (InExposure)
  {
    // Check if exposure is complete
    double timeLeft = calcTimeLeft();
    LOGF_DEBUG("Timer hit: %.2f seconds left", timeLeft);

    if (timeLeft <= 0)
    {
      LOG_INFO("Exposure complete, downloading image...");
      // Download the image
      InExposure = false;

      if (isSimulation())
      {
        // Generate simulated data
        int w = PrimaryCCD.getXRes();
        int h = PrimaryCCD.getYRes();
        int bpp = PrimaryCCD.getBPP() / 8;
        int size = w * h * bpp;
        uint8_t *data = new uint8_t[size];

        // Simple gradient pattern
        for (int i = 0; i < size; i++)
        {
          data[i] = (i * 255) / size;
        }

        ExposureComplete(&PrimaryCCD);
        delete[] data;
      }
      else
      {
        // Download real image
        grabImage();
      }
    }
    else
    {
      // Update progress
      PrimaryCCD.setExposureLeft(timeLeft);
    }
  }

  SetTimer(getCurrentPollingPeriod());
}

bool SigmaCCD::grabImage()
{
  SigmaImage img;

  if (!cam_->downloadLastCapture(img))
  {
    LOG_ERROR("Failed to download image");
    return false;
  }

  if (img.data.empty())
  {
    LOG_ERROR("Downloaded image is empty");
    return false;
  }

  // Handle JPEG images
  int formatIndex = IUFindOnSwitchIndex(CaptureFormatSP);
  bool sizeSet = false;
  if (formatIndex == 0)
    return decodeJPEG(img, false, sizeSet);
  else
    return decodeDNG(img, formatIndex == 2 ? 14 : 12);
}

double SigmaCCD::clampToSupported(double seconds)
{
  auto list = cam_->supportedExposures();
  if (list.empty())
    return seconds;

  // Find nearest value
  auto it =
      std::min_element(list.begin(), list.end(), [seconds](double a, double b)
                       { return std::abs(a - seconds) < std::abs(b - seconds); });

  return *it;
}

double SigmaCCD::calcTimeLeft()
{
  struct timeval now;
  gettimeofday(&now, nullptr);

  double timeSince =
      (now.tv_sec - ExpStart.tv_sec) + (now.tv_usec - ExpStart.tv_usec) / 1e6;

  return ExposureRequest - timeSince;
}

bool SigmaCCD::StartStreaming()
{
  if (Streamer->isBusy())
    return true;

  if (!cam_->startLiveView())
  {
    LOG_ERROR("Failed to start live view");
    return false;
  }

  // Reset the primary CCD frame buffer size for streaming
  // This is important to clear any previous capture sizes
  PrimaryCCD.setFrameBufferSize(0);

  Streamer->setPixelFormat(INDI_RGB, 8);

  isStreaming = true;

  // Start streaming thread
  int rc = pthread_create(&streamThread, nullptr, &SigmaCCD::streamVideoHelper, this);
  if (rc != 0)
  {
    LOG_ERROR("Failed to create streaming thread");
    cam_->stopLiveView();
    isStreaming = false;
    return false;
  }

  LOG_INFO("Live view started");
  return true;
}

bool SigmaCCD::StopStreaming()
{
  if (!isStreaming)
    return true;

  isStreaming = false;

  // Wait for thread to finish
  pthread_join(streamThread, nullptr);

  cam_->stopLiveView();

  // Restore full sensor resolution for captures
  if (auto res = cam_->sensorResolution())
  {
    PrimaryCCD.setFrame(0, 0, res->first, res->second);
    PrimaryCCD.setFrameBufferSize(res->first * res->second * 2); // 16-bit for RAW
  }

  LOG_INFO("Live view stopped");
  return true;
}

void *SigmaCCD::streamVideo(void *arg)
{
  bool sizeSet = false;

  while (isStreaming)
  {
    std::vector<uint8_t> jpegData;

    if (cam_->readLiveViewFrame(jpegData) && !jpegData.empty())
    {
      // Decode JPEG to RGB
      SigmaImage img;
      img.data = jpegData;

      if (!decodeJPEG(img, true, sizeSet))
        LOG_ERROR("Failed to decode current jpeg image");
    }

    // Control frame rate (e.g., 10 FPS)
    // Use INDI's streaming exposure property for timing
    INumberVectorProperty *streamExp = getNumber("STREAMING_EXPOSURE");
    double exposureTime = streamExp ? streamExp->np[0].value : 0.1; // seconds
    usleep(exposureTime * 1000000);                                 // Convert to microseconds
  }

  return nullptr;
}

// ISNewNumber: handle it
bool SigmaCCD::ISNewNumber(const char *dev, const char *name,
                           double values[], char *names[], int n)
{
  if (!strcmp(dev, getDeviceName()) && !strcmp(name, DownloadTimeoutNP.getName()))
  {
    auto *np = DownloadTimeoutNP.getNumber();
    IUUpdateNumber(np, values, names, n);
    double seconds = np->np[0].value;
    cam_->setDownloadTimeout(seconds);
    np->s = IPS_OK;
    IDSetNumber(np, nullptr);
    return true;
  }
  return INDI::CCD::ISNewNumber(dev, name, values, names, n);
}

bool SigmaCCD::ISNewSwitch(const char *dev, const char *name,
                           ISState *states, char *names[], int n)
{
  if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
  {
    // --- Capture format (if you still use legacy C structs FormatSP/FormatS) ---
    if (!strcmp(name, CaptureFormatSP.getName()))
    {
      auto *sp = CaptureFormatSP.getSwitch();
      IUUpdateSwitch(sp, states, names, n);
      int idx = sp->findOnSwitchIndex(); // assumes item[0]=JPEG, item[1]=DNG12, item[2]=DNG14
      cam_->setImageQuality(idx == 0);
      if (idx > 0)
        cam_->setDNGQuality(idx == 2);
      sp->s = IPS_OK;
      IDSetSwitch(sp, nullptr);
      LOGF_INFO("Image format set to %s", idx == 0 ? "JPEG" : (idx == 2 ? "DNG 14 bits" : "DNG 12 bits"));
      return true;
    }

    // --- ISO (PropertySwitch + items) ---
    if (!strcmp(name, ISOSP.getName()))
    {
      auto *svp = ISOSP.getSwitch();
      IUUpdateSwitch(svp, states, names, n);
      int idx = svp->findOnSwitchIndex();
      if (idx >= 0 && (size_t)idx < ISOItems.size())
      {
        int iso = cam_->supportedISOs()[idx]; // your parallel ISO table
        cam_->setISO(iso);
        svp->s = IPS_OK;
        IDSetSwitch(svp, nullptr);
        LOGF_INFO("ISO set to %d", iso);
      }
      else
      {
        svp->s = IPS_ALERT;
        IDSetSwitch(svp, "Invalid ISO selection");
      }
      return true;
    }

    // --- Capture Target (InCamera / InComputer / Both) ---
    if (!strcmp(name, CaptureTargetSP.getName()))
    {
      auto *svp = CaptureTargetSP.getSwitch();
      IUUpdateSwitch(svp, states, names, n);
      int idx = svp->findOnSwitchIndex(); // 0..2
      cam_->setDestination(idx);
      svp->s = IPS_OK;

      LOGF_INFO("Save destination set to %s", idx == 0 ? "In camera" : (idx == 1 ? "In computer" : "Both"));
      IDSetSwitch(svp, nullptr);
      return true;
    }

    // --- Exposure presets (discrete shutter speeds) ---
    if (!strcmp(name, ExposurePresetSP.getName()))
    {
      auto *svp = ExposurePresetSP.getSwitch();
      IUUpdateSwitch(svp, states, names, n);
      int idx = svp->findOnSwitchIndex();
      if (idx >= 0 && (size_t)idx < ExposurePresetItems.size())
      {
        double t = cam_->supportedExposures()[idx]; // your shutter table from backend
        cam_->setExposureSeconds(t);
        svp->s = IPS_OK;
        IDSetSwitch(svp, nullptr);
        LOGF_INFO("Exposure preset set to %s", ExposurePresetItems[idx].label);
      }
      else
      {
        svp->s = IPS_ALERT;
        IDSetSwitch(svp, "Invalid exposure preset");
      }
      return true;
    }
  }

  return INDI::CCD::ISNewSwitch(dev, name, states, names, n);
}

bool SigmaCCD::decodeJPEG(const SigmaImage &img, bool forStreaming, bool &sizeSet)
{
  // Decode JPEG to raw pixels
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr_wrapper jerr;

  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = jpeg_error_exit;

  if (setjmp(jerr.setjmp_buffer))
  {
    jpeg_destroy_decompress(&cinfo);
    LOG_ERROR("JPEG decompression failed");
    return false;
  }

  jpeg_create_decompress(&cinfo);
  jpeg_mem_src(&cinfo, img.data.data(), img.data.size());

  if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK)
  {
    jpeg_destroy_decompress(&cinfo);
    return false;
  }

  int width = cinfo.image_width;
  int height = cinfo.image_height;
  int channels = 3;
  size_t frameSize = width * height * channels;
  LOGF_INFO("LiveView size: %dx%dx%d", width, height, channels);

  // Set size only once when we know it
  if (!sizeSet)
  {
    // Reset PrimaryCCD dimensions for streaming
    PrimaryCCD.setFrame(0, 0, width, height);
    PrimaryCCD.setFrameBufferSize(frameSize);

    Streamer->setSize(width, height);
    sizeSet = true;
    LOGF_INFO("LiveView size: %dx%d", width, height);
  }

  cinfo.out_color_space = JCS_RGB;
  jpeg_start_decompress(&cinfo);

  if (forStreaming)
  {
    // For streaming, use a local buffer
    std::vector<uint8_t> buffer(frameSize);

    int row_stride = width * channels;
    while (cinfo.output_scanline < cinfo.output_height)
    {
      JSAMPROW row = &buffer[cinfo.output_scanline * row_stride];
      jpeg_read_scanlines(&cinfo, &row, 1);
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    // Send to streamer, NOT ExposureComplete
    Streamer->newFrame(buffer.data(), buffer.size());
  }
  else
  {
    // For capture, use PrimaryCCD buffer
    PrimaryCCD.setFrame(0, 0, width, height);
    PrimaryCCD.setFrameBufferSize(frameSize);
    PrimaryCCD.setNAxis(channels == 1 ? 2 : 3);
    PrimaryCCD.setBPP(8);

    uint8_t *buffer = PrimaryCCD.getFrameBuffer();

    int row_stride = width * channels;
    while (cinfo.output_scanline < cinfo.output_height)
    {
      JSAMPROW row = &buffer[cinfo.output_scanline * row_stride];
      jpeg_read_scanlines(&cinfo, &row, 1);
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    // Only call ExposureComplete for actual captures
    ExposureComplete(&PrimaryCCD);
    LOGF_INFO("Captured JPEG: %dx%d, %d channels", width, height, channels);
  }

  return true;
}

bool SigmaCCD::decodeDNG(const SigmaImage &img, int bitDepth)
{
  LibRaw processor;

  int ret = processor.open_buffer(img.data.data(), img.data.size());
  if (ret != LIBRAW_SUCCESS)
  {
    LOGF_ERROR("Cannot open DNG: %s", libraw_strerror(ret));
    return false;
  }

  // Set processing parameters for raw extraction
  processor.imgdata.params.use_auto_wb = 0;
  processor.imgdata.params.no_auto_bright = 1;
  processor.imgdata.params.output_bps = 16;

  ret = processor.unpack();
  if (ret != LIBRAW_SUCCESS)
  {
    LOGF_ERROR("Cannot unpack DNG: %s", libraw_strerror(ret));
    return false;
  }

  // Get raw dimensions
  int width = processor.imgdata.sizes.width;
  int height = processor.imgdata.sizes.height;

  const char *pattern = processor.imgdata.idata.cdesc;
  if (pattern[0] != 0)
  {
    char bayerPattern[5] = {0};
    for (int i = 0; i < 4; i++)
    {
      bayerPattern[i] = pattern[processor.COLOR(i / 2, i % 2)];
    }
    // Only update if different from your default
    if (strcmp(bayerPattern, "RGGB") != 0)
    {
      BayerTP[2].setText(bayerPattern);
      LOGF_INFO("Detected Bayer pattern: %s", bayerPattern);
    }
  }

  // Setup frame buffer for raw Bayer data
  PrimaryCCD.setFrame(0, 0, width, height);
  PrimaryCCD.setFrameBufferSize(width * height * 2);
  PrimaryCCD.setBPP(16);
  PrimaryCCD.setNAxis(2);

  // Copy raw sensor data
  uint16_t *buffer = reinterpret_cast<uint16_t *>(PrimaryCCD.getFrameBuffer());
  memcpy(buffer, processor.imgdata.rawdata.raw_image, width * height * sizeof(uint16_t));

  LOGF_INFO("Decoded DNG: %dx%d, %d-bit Bayer", width, height, bitDepth);

  ExposureComplete(&PrimaryCCD);

  LOGF_INFO("Downloaded %s image: %zu bytes (%dx%d)", img.mime.c_str(),
            img.data.size(), img.width, img.height);

  return true;
}

void SigmaCCD::bind_logger()
{
  log_set_sink([this](LogLevel lvl, const char *msg)
               {
  switch (lvl) {
    case LogLevel::Debug:   INDI::Logger::getInstance().print(getDeviceName(), INDI::Logger::DBG_DEBUG, __FILE__, __LINE__, msg); break; // LOGF_DEBUG("%s", msg ? msg : "");   break;
    case LogLevel::Info:    INDI::Logger::getInstance().print(getDeviceName(), INDI::Logger::DBG_SESSION, __FILE__, __LINE__, msg); break; // OGF_INFO ("%s", msg ? msg : "");   break;
    case LogLevel::Warning: INDI::Logger::getInstance().print(getDeviceName(), INDI::Logger::DBG_WARNING, __FILE__, __LINE__, msg); break; // OGF_WARN ("%s", msg ? msg : "");   break;
    case LogLevel::Error:   INDI::Logger::getInstance().print(getDeviceName(), INDI::Logger::DBG_ERROR, __FILE__, __LINE__, msg); break; // OGF_ERROR("%s", msg ? msg : "");   break;
  } });
}
