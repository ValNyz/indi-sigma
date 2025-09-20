cmake_minimum_required(VERSION 3.14)  # FetchContent requires 3.14+
project(indi-sigma CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use pkg-config for dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(INDI REQUIRED libindi)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
pkg_check_modules(CFITSIO REQUIRED cfitsio)
pkg_check_modules(JPEG REQUIRED libjpeg)
pkg_check_modules(LIBRAW REQUIRED libraw)

# FetchContent for sigma_c_driver
include(FetchContent)
FetchContent_Declare(
  sigma_driver
  GIT_REPOSITORY https://github.com/ValNyz/sigma-driver.git
  GIT_TAG        main
  GIT_SHALLOW    TRUE  # Only fetch the latest commit
  GIT_PROGRESS   TRUE  # Show download progress
  SOURCE_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/external/sigma-driver
  # Build configuration options for sigma-driver
  CMAKE_ARGS     -DBUILD_TESTING=OFF
                 -DBUILD_EXAMPLES=OFF
)

# Set options before FetchContent_MakeAvailable
set(BUILD_TESTS OFF CACHE INTERNAL "")
set(BUILD_EXAMPLES OFF CACHE INTERNAL "")

FetchContent_MakeAvailable(sigma_driver)

# Set INDI data dir manually for Fedora ?
set(INDI_DATA_DIR "/usr/share/indi")

# Create the INDI driver executable
add_executable(indi_sigma_ccd
  sigma_ccd.cpp
  sigma_backend.cpp
)

# Include directories
target_include_directories(indi_sigma_ccd PRIVATE
  ${INDI_INCLUDE_DIRS}
  ${CFITSIO_INCLUDE_DIRS}
  ${sigma_driver_SOURCE_DIR}/include
  ${LIBUSB_INCLUDE_DIRS}
  ${JPEG_INCLUDE_DIRS}
  ${LIBRAW_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(indi_sigma_ccd PRIVATE
  indidriver
  ptp_sigma
  ${CFITSIO_LIBRARIES}
  ${LIBUSB_LIBRARIES}
  ${JPEG_LIBRARIES}
  ${LIBRAW_LIBRARIES}
  pthread
)

# Set RPATH for finding libraries at runtime
set_target_properties(indi_sigma_ccd PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Installation
install(TARGETS indi_sigma_ccd
  RUNTIME DESTINATION bin
)

install(FILES indi_sigma.xml
  DESTINATION ${INDI_DATA_DIR}
)