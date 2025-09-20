#pragma once

#include "sigma_backend.h"
#include <indiccd.h>
#include <memory>
#include <sys/time.h>

static inline std::string formatISO(int v)
{
  return std::to_string(v);
};

static inline std::string formatExposure(double s)
{
  std::ostringstream os;
  if (s >= 1.0)
  {
    // whole seconds like 30, 25, … print "30s"
    if (std::fabs(s - std::round(s)) < 1e-9)
      os << int(std::lround(s)) << "s";
    // fractional seconds like 3.2, 2.5, 1.6, 1.3 print "3.2s"
    else
      os << std::fixed << std::setprecision(1) << s << "s";
  }
  else
    // sub-second values print as reciprocal: 1/60s, 1/125s, …
    os << "1/" << int(std::lround(1.0 / s)) << "s";
  return os.str();
}

class SigmaCCD : public INDI::CCD
{
public:
  SigmaCCD();
  virtual ~SigmaCCD() = default;

  // INDI interface
  const char *getDefaultName() override;
  bool initProperties() override;
  void ISGetProperties(const char *dev) override;
  bool updateProperties() override;

  bool Connect() override;
  bool Disconnect() override;

  // CCD specific
  bool StartExposure(float duration) override;
  bool AbortExposure() override;
  bool UpdateCCDFrame(int x, int y, int w, int h) override;
  void TimerHit() override;

  // Property handlers
  bool ISNewNumber(const char *dev, const char *name, double values[],
                   char *names[], int n) override;
  bool ISNewSwitch(const char *dev, const char *name, ISState *states,
                   char *names[], int n) override;

protected:
  // Helper functions
  double clampToSupported(double seconds);
  double calcTimeLeft();
  bool grabImage();

private:
  void bind_logger();

  bool decodeJPEG(const SigmaImage& img);
  bool decodeDNG(const SigmaImage& img, int bitDepth);

  std::unique_ptr<SigmaBackend> cam_;

  // ISO List
  INDI::PropertySwitch ISOSP{0};
  std::vector<ISwitch> ISOItems;
  int defaultISO{400};

  // Capture Target selection
  INDI::PropertySwitch CaptureTargetSP{3};
  enum
  {
    InCamera,
    InComputer,
    Both,
  };

  // Exposure Presets
  INDI::PropertySwitch ExposurePresetSP{0};
  std::vector<ISwitch> ExposurePresetItems;

  // Wait this many seconds before giving up on exposure download
  INDI::PropertyNumber DownloadTimeoutNP{1};

  INDI::PropertyBlob imageBP{INDI::Property()};

  char name[MAXINDIDEVICE];

  // Exposure tracking
  double ExposureRequest;
  struct timeval ExpStart;
  bool InExposure;
  int exposureRetries;
};
