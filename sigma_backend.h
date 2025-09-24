#pragma once
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <iomanip>

struct SigmaImage
{
  std::vector<uint8_t> data;
  int width{0}, height{0}, bpp{8};
  std::string mime{"image/jpeg"};
  std::string filename;
  uint32_t filesize{0};
};

class SigmaBackend
{
public:
  SigmaBackend();
  ~SigmaBackend();

  // Connection management
  bool open(const char* device_name);
  void close();
  bool isOpen() const;

  // Camera information
  std::string model() const;
  std::string serialNumber() const;
  std::string firmwareVersion() const;
  std::optional<std::pair<int, int>> sensorResolution();

  // Exposure settings
  const std::vector<double>& supportedExposures() const { return speeds_; }
  double nearestSupportedExposure(double s) const;
  bool setExposureSeconds(double seconds);

  // ISO settings
  const std::vector<int>& supportedISOs() const { return isos_; }
  bool setISO(int iso);

  // Aperture settings
  bool setAperture(double fnumber);

  // Image quality
  bool setImageQuality(bool jpeg);
  bool setDNGQuality(bool is14bit);
  bool setDestination(int value);

  // Timeout
  bool setDownloadTimeout(int seconds);

  // Capture control
  bool triggerCapture();
  bool downloadLastCapture(SigmaImage &out);
  bool abortCapture();

  // Live view
  bool startLiveView();
  bool stopLiveView();
  bool readLiveViewFrame(std::vector<uint8_t> &jpeg);

  // Status
  bool isCaptureComplete();
  int getBatteryLevel();

private:
  void debug(int value);

  static constexpr double EPS = 1e-9;
  const std::vector<double> speeds_{
    300,30,25,20,15,13,10, 8,6,5,4,3.2,2.5, 2,1.6,1.3,1,0.8,0.6,
    0.5,0.4,0.3, 1.0/4,1.0/5,1.0/6, 1.0/8,1.0/10,1.0/13,1.0/15,1.0/20,1.0/25,
    1.0/30,1.0/60,1.0/125,1.0/250,1.0/500,1.0/1000,1.0/2000,1.0/4000,1.0/8000,
  };
  const std::vector<int> isos_{
    6,8,10,12,16,20,25,32,40,50,64,80,100,125,160,200,250,320,400,500,640,
    800,1000,1250,1600,2000,2500,3200,4000,5000,6400,8000,10000,12800,
    16000,20000,25600,32000,40000,51200,64000,80000,102400,
  };
  int downloadTimeout;
  class Impl;
  std::unique_ptr<Impl> pImpl;
};
