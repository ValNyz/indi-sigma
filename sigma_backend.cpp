#include "sigma_backend.h"

// Include actual sigma_c_driver headers
#include "ptp/usb_transport.h"
#include "sigma/enum.h"
#include "sigma/schema.h"
#include "sigma/sigma_ptp.h"
#include "utils/apex.h"

#include <chrono>
#include <cmath>
#include <thread>

#include "utils/log.h"

class SigmaBackend::Impl
{
public:
  std::unique_ptr<USBTransport> transport;
  std::unique_ptr<SigmaCamera> camera;
  ApiConfig config;
  bool isOpen{false};
  uint8_t lastImageId{0};

  // Convert seconds to APEX shutter speed code
  uint8_t secondsToApex(double seconds)
  {
    return ShutterSpeed3Converter.encode_uint8(seconds);
  }

  // Convert ISO to APEX code
  uint8_t isoToApex(int iso) { return ISOSpeedConverter.encode_uint8(iso); }

  // Convert f-number to APEX code
  uint8_t apertureToApex(double fnumber)
  {
    return Aperture3Converter.encode_uint8(fnumber);
  }
};

SigmaBackend::SigmaBackend() : pImpl(std::make_unique<Impl>()), downloadTimeout(1)
{
  LOG_SET_LEVEL(LogLevel::Debug);
}

SigmaBackend::~SigmaBackend()
{
  if (isOpen())
    close();
}

bool SigmaBackend::open(const char *device_name)
{
  try
  {
    pImpl->transport = std::make_unique<USBTransport>();
    pImpl->transport->open_first(); // Opens first PTP device found

    pImpl->camera = std::make_unique<SigmaCamera>(*pImpl->transport);
    pImpl->camera->open_session(1);

    // Initialize camera with ConfigApi
    pImpl->config = pImpl->camera->config_api();

    // Set manual exposure mode. Necessary to set shutterspeed and iso.
    CamDataGroup2 g2;
    g2.exposureMode = ExposureMode::Manual;
    pImpl->camera->set_group(g2);

    // Set destination to computer for image transfer // TOCHECK
    // TODO Need to align to default value in sigma_ccd property
    // TODO Also check if it works.
    CamDataGroup3 g3;
    g3.destToSave = DestToSave::InCamera;
    pImpl->camera->set_group(g3);

    pImpl->isOpen = true;
    LOG_INFO("Connected to %s", pImpl->config.camera_model().c_str());
    return true;
  }
  catch (const std::exception &e)
  {
    LOG_ERROR("Failed to open camera: %s", e.what());
    return false;
  }
}

void SigmaBackend::close()
{
  if (!pImpl->isOpen)
    return;

  try
  {
    if (pImpl->camera)
    {
      pImpl->camera->close_application();
      pImpl->camera->close_session();
      pImpl->camera.reset();
    }
    if (pImpl->transport)
    {
      pImpl->transport->close();
      pImpl->transport.reset();
    }
  }
  catch (...)
  {
  }

  pImpl->isOpen = false;
}

bool SigmaBackend::isOpen() const
{
  return pImpl->isOpen && pImpl->transport && pImpl->transport->is_open();
}

std::string SigmaBackend::model() const { return pImpl->config.camera_model(); }

std::string SigmaBackend::serialNumber() const
{
  return pImpl->config.serial_number();
}

std::string SigmaBackend::firmwareVersion() const
{
  return pImpl->config.firmware_version();
}

std::optional<std::pair<int, int>> SigmaBackend::sensorResolution()
{
  // Sigma FP: 6000x4000, FP L: 9520x6328
  if (model().find("fp L") != std::string::npos)
  {
    return std::make_pair(9520, 6328);
  }
  else if (model().find("fp") != std::string::npos)
  {
    return std::make_pair(6064, 4042);
  }
  return std::nullopt;
}

double SigmaBackend::nearestSupportedExposure(double s) const
{
  const auto &v = speeds_;
  if (v.empty())
    return s;

  // clamp to range
  if (s <= v.back())
    return v.back();
  if (s >= v.front())
    return v.front();

  // exact match fast path
  for (double x : v)
    if (std::fabs(x - s) < EPS)
      return x;

  // nearest
  int best = 0;
  double bestd = std::fabs(v[0] - s);
  for (int i = 1; i < (int)v.size(); ++i)
  {
    double d = std::fabs(v[i] - s);
    if (d < bestd)
    {
      bestd = d;
      best = i;
    }
  }
  return v[best];
}

bool SigmaBackend::setExposureSeconds(double seconds)
{
  try
  {
    CamDataGroup1 g1;
    g1.shutterSpeed = pImpl->secondsToApex(seconds);
    pImpl->camera->set_group(g1);
    LOG_INFO("Set exposure to %.3f seconds (APEX: %d)", seconds,
             *g1.shutterSpeed);
    debug(1);
    return true;
  }
  catch (const std::exception &e)
  {
    LOG_ERROR("setExposureSeconds failed: %s", e.what());
    return false;
  }
}

bool SigmaBackend::setISO(int iso)
{
  try
  {
    CamDataGroup1 g1;
    g1.isoAuto = ISOAuto::Manual;
    g1.isoSpeed = pImpl->isoToApex(iso);
    pImpl->camera->set_group(g1);
    LOG_INFO("Set iso to %d seconds (APEX: %d)", iso, *g1.isoSpeed);
    debug(1);
    return true;
  }
  catch (const std::exception &e)
  {
    LOG_ERROR("setISO failed: %s", e.what());
    return false;
  }
}

bool SigmaBackend::setAperture(double fnumber)
{
  try
  {
    CamDataGroup1 g1;
    g1.aperture = pImpl->apertureToApex(fnumber);
    pImpl->camera->set_group(g1);
    LOG_INFO("Set iso to %d (APEX: %d)", fnumber, *g1.aperture);
    debug(1);
    return true;
  }
  catch (const std::exception &e)
  {
    LOG_ERROR("setAperture failed: %s", e.what());
    return false;
  }
}

bool SigmaBackend::setImageQuality(bool jpeg)
{
  try
  {
    CamDataGroup2 g2;
    g2.imageQuality = jpeg ? ImageQuality::JPEGNormal : ImageQuality::DNG;
    pImpl->camera->set_group(g2);
    LOG_INFO("setImageQuality to %s", jpeg ? "JPEG" : "DNG");
    debug(2);
    return true;
  }
  catch (const std::exception &e)
  {
    LOG_ERROR("setImageQuality failed: %s", e.what());
    return false;
  }
}

bool SigmaBackend::setDNGQuality(bool is14bit)
{
  try
  {
    setImageQuality(false);
    CamDataGroup4 g4;
    g4.dngQuality = is14bit ? DNGQuality::Q14bit : DNGQuality::Q12bit;
    pImpl->camera->set_group(g4);
    LOG_INFO("setDNGQuality to %s", is14bit ? "14 bits" : "12 bits");
    debug(4);
    return true;
  }
  catch (const std::exception &e)
  {
    LOG_ERROR("setDNGQuality failed: %s", e.what());
    return false;
  }
}

bool SigmaBackend::setDestination(int value)
{
  if (value > 2)
    return false;
  try
  {
    CamDataGroup3 g3;
    if (value == 0)
      g3.destToSave = DestToSave::InCamera;
    else if (value == 1)
      g3.destToSave = DestToSave::InComputer;
    else
      g3.destToSave = DestToSave::Both;
    pImpl->camera->set_group(g3);
    LOG_INFO("set save destination to %s", value == 0 ? "In camera" : (value == 1 ? "In computer" : "Both"));
    debug(3);
    return true;
  }
  catch (const std::exception &e)
  {
    LOG_ERROR("setDestination failed: %s", e.what());
    return false;
  }
}

bool SigmaBackend::setDownloadTimeout(int seconds)
{
  try
  {
    downloadTimeout = seconds;
    return true;
  }
  catch (const std::exception &e)
  {
    LOG_ERROR("setDownloadTimeout failed: %s", e.what());
    return false;
  }
}

void SigmaBackend::debug(int value)
{
  if (log_get_level() == LogLevel::Debug)
  {
    if (value == 0 or value == 1)
    {
      CamDataGroup1 g1 = pImpl->camera->get_group<CamDataGroup1>();
      LOG_INFO("CamDataGroup1: iso mode=0x%01X iso=%d shutter speed=%d", g1.isoAuto.value(), g1.isoSpeed.value(), g1.shutterSpeed.value());
    }
    if (value == 0 or value == 2)
    {
      CamDataGroup2 g2 = pImpl->camera->get_group<CamDataGroup2>();
      LOG_INFO("CamDataGroup2: exposure mode=0x%01X resolution=0x%01X image_quality=0x%02X", g2.exposureMode.value(), g2.resolution.value(), g2.imageQuality.value());
    }
    if (value == 0 or value == 3)
    {
      CamDataGroup3 g3 = pImpl->camera->get_group<CamDataGroup3>();
      LOG_INFO("CamDataGroup3: destToSave=0x%01X", g3.destToSave.value());
    }
    if (value == 0 or value == 4)
    {
      CamDataGroup4 g4 = pImpl->camera->get_group<CamDataGroup4>();
      LOG_INFO("CamDataGroup4: dngQuality=0x%01X", g4.dngQuality.value());
    }
  }
}

bool SigmaBackend::triggerCapture()
{
  try
  {
    debug(0);
    // Single capture
    uint16_t resp = pImpl->camera->snap(CaptureMode::NonAFCapt, 1);
    LOG_INFO("Capture triggered, response: 0x%04X", resp);

    return resp == PTP_RESP_OK;
  }
  catch (const std::exception &e)
  {
    LOG_ERROR("triggerCapture failed: %s", e.what());
    return false;
  }
}

bool SigmaBackend::downloadLastCapture(SigmaImage &out)
{
  int timeout = lround(downloadTimeout * 1000 / 30);
  LOG_INFO("Downloading last capture. Timeout is 30x%d", timeout);
  try
  {
    // Wait for capture completion
    CamCaptStatus status = pImpl->camera->wait_completion(pImpl->lastImageId, 30, timeout);
    LOG_INFO("CamCaptStatus: id=%u head=%u tail=%u code=0x%04X dest=0x%02X",
             status.ImageId, status.ImageDBHead, status.ImageDBTail, status.Status, status.Dest);

    pImpl->lastImageId++;
    if (status.Status != CaptStatus::ImageGenCompleted &&
        status.Status != CaptStatus::ImageDataStorageCompleted)
    {
      LOG_WARN("Capture not complete, status: 0x%04X",
               static_cast<uint16_t>(status.Status));
      return false;
    }
    else
      LOG_INFO("Capture complete, status: 0x%04X", static_cast<uint16_t>(status.Status));

    // If destination is computer, wait for ObjectAdded event and download
    // if (status.Dest == DestToSave::InComputer)
    // {
    //   LOG_INFO("Start capture loading in computer, dest: 0x%04X", static_cast<uint16_t>(status.Dest));
    //   if (auto handle = pImpl->camera->wait_object_added(5000, 200)) // This is not working...
    //   {
    //     out.data = pImpl->camera->get_object(*handle);

    //     // Get object info for metadata
    //     auto info_bytes = pImpl->camera->get_object_info(*handle);
    //     // Parse info for dimensions and format (simplified)
    //     out.mime = "image/jpeg";
    //     if (auto res = sensorResolution())
    //     {
    //       out.width = res->first;
    //       out.height = res->second;
    //     }
    //     out.filesize = out.data.size();
    //     pImpl->camera->clear_image_db_single(status.ImageId);

    //     LOG_INFO("Downloaded image: %zu bytes", out.data.size());
    //     return true;
    //   }
    // }
    // else
    // {
    LOG_INFO("Start capture loading in camera, dest: 0x%04X", static_cast<uint16_t>(status.Dest));
    // Use vendor-specific download for camera storage
    auto info = pImpl->camera->get_pict_file_info2();
    LOG_INFO("PictFileInfo2: %s/%s size=%u bytes addr=0x%08X", info.PathName.c_str(),
             info.FileName.c_str(), info.FileSize, info.FileAddress);

    out.filename = info.FileName;
    out.width = info.SizeX;
    out.height = info.SizeY;

    BigPartialPictFile img = pImpl->camera->get_big_partial_pict_file(info.FileAddress, 0, info.FileSize);
    LOG_INFO("BigPartialPictFile: size=%u bytes dataSize=%u bytes", img.AcquiredSize,
             img.PartialData.size());
    out.data = img.PartialData;
    out.filesize = out.data.size();
    pImpl->camera->clear_image_db_single(status.ImageId);

    LOG_INFO("Downloaded image: %zu bytes", out.data.size());
    return true;
    // }

    return false;
  }
  catch (const std::exception &e)
  {
    LOG_ERROR("downloadLastCapture failed: %s", e.what());
    return false;
  }
}

bool SigmaBackend::abortCapture()
{
  // Not directly supported in discrete mode
  // Could potentially send terminate_open_capture for BULB mode later
  return true;
}

bool SigmaBackend::startLiveView()
{
  try
  {
    CamDataGroup2 g2;
    g2.specialMode = SpecialMode::LiveView;
    pImpl->camera->set_group(g2);
    LOG_INFO("Live view started");
    return true;
  }
  catch (const std::exception &e)
  {
    LOG_ERROR("startLiveView failed: %s", e.what());
    return false;
  }
}

bool SigmaBackend::stopLiveView()
{
  try
  {
    CamDataGroup2 g2;
    g2.specialMode = SpecialMode::Null;
    pImpl->camera->set_group(g2);
    LOG_INFO("Live view stopped");
    return true;
  }
  catch (const std::exception &e)
  {
    LOG_ERROR("stopLiveView failed: %s", e.what());
    return false;
  }
}

bool SigmaBackend::readLiveViewFrame(std::vector<uint8_t> &jpeg)
{
  try
  {
    ViewFrame frame = pImpl->camera->get_view_frame();
    jpeg = frame.Data;
    return !jpeg.empty();
  }
  catch (const std::exception &e)
  {
    LOG_ERROR("readLiveViewFrame failed: %s", e.what());
    return false;
  }
}

bool SigmaBackend::isCaptureComplete()
{
  try
  {
    CamCaptStatus status = pImpl->camera->get_cam_capt_status(pImpl->lastImageId);
    return status.Status == CaptStatus::ImageGenCompleted ||
           status.Status == CaptStatus::ImageDataStorageCompleted;
  }
  catch (...)
  {
    return false;
  }
}

int SigmaBackend::getBatteryLevel()
{
  try
  {
    CamDataGroup1 g1 = pImpl->camera->get_group<CamDataGroup1>();
    return g1.batteryState.value_or(0);
  }
  catch (...)
  {
    return 0;
  }
}