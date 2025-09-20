%global         tag %{?tag}%{!?tag:v%{version}}
%global         debug_package %{nil}
Name:           indi-sigma
Version:        %{?version}%{!?version:0.1.0}
Release:        1%{?dist}
Summary:        INDI driver for Sigma cameras
License:        MIT
URL:            https://github.com/ValNyz/indi-sigma
Source0:        %{url}/archive/refs/tags/%{tag}.tar.gz#/%{name}-%{version}.tar.gz
BuildRequires:  cmake, gcc-c++, libindi-devel, libusb1-devel, LibRaw-devel, cfitsio-devel, libjpeg-turbo-devel
Requires:       libindi, libusb1, libraw, cfitsio, libjpeg-turbo

%description
INDI CCD driver for Sigma cameras using libptp_sigma.

%prep
%autosetup -n %{name}-%{version}

%build
%cmake -DCMAKE_BUILD_TYPE=Release
%cmake_build

%install
%cmake_install

%files
%license LICENSE
%doc README.md
%{_bindir}/indi_sigma_ccd
%{_datadir}/indi/indi_sigma.xml

%changelog
* Sat Sep 20 2025 Val Nyz <valentin.nyzam@gmail.com> - 0.3.0-1
- Add optional RAW path (DNG → FITS) via libraw + cfitsio
- JPEG decode to gray8 pixel path; ExposureComplete used for Ekos preview
- Use RPM CMake macros (%cmake, %cmake_build, %cmake_install)

* Fri Sep 19 2025 Val Nyz <valentin.nyzam@gmail.com> - 0.2.0-1
- Implement capture target switches and ISO/exposure presets
- Emit CCD1 BLOBs for native files when not using pixel path
- Install indi_sigma.xml to %{_datadir}/indi

* Fri Sep 12 2025 Val Nyz <valentin.nyzam@gmail.com> - 0.1.0-1
- Initial RPM packaging for indi-sigma
- Build with libindi and libusb; install indi_sigma_ccd binary