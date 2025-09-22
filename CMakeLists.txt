cmake_minimum_required(VERSION 3.14) # FetchContent requires 3.14+
project(indi-sigma CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

set(PTP_SIGMA_PREFIX ${CMAKE_BINARY_DIR}/ptp_sigma/prefix)
set(PTP_SIGMA_SO "${CMAKE_SHARED_LIBRARY_PREFIX}ptp_sigma${CMAKE_SHARED_LIBRARY_SUFFIX}")
set(PTP_SIGMA_BYPRODUCT "${PTP_SIGMA_PREFIX}/${CMAKE_INSTALL_LIBDIR}/${PTP_SIGMA_SO}")
set(BUILD_TESTS OFF CACHE INTERNAL "")
set(BUILD_EXAMPLES OFF CACHE INTERNAL "")
# Set INDI data dir manually for Fedora ?
set(INDI_DATA_DIR "/usr/share/indi")

# Use pkg-config for dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(INDI REQUIRED libindi>=2.0)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
pkg_check_modules(CFITSIO REQUIRED cfitsio)
pkg_check_modules(JPEG REQUIRED libjpeg)
pkg_check_modules(LIBRAW REQUIRED libraw)

file(MAKE_DIRECTORY
  "${PTP_SIGMA_PREFIX}/include"
  "${PTP_SIGMA_PREFIX}/lib"
)

include(ExternalProject)
ExternalProject_Add(PTP_SIGMA_EP
  GIT_REPOSITORY https://github.com/ValNyz/sigma-driver.git
  GIT_TAG v1.0.6
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=${PTP_SIGMA_PREFIX}
  -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
  -DBUILD_SHARED_LIBS=ON
  -DBUILD_TESTS=OFF
  INSTALL_DIR ${PTP_SIGMA_PREFIX}
  UPDATE_DISCONNECTED 1
  BUILD_BYPRODUCTS ${PTP_SIGMA_BYPRODUCT}
)

add_library(ptp_sigma SHARED IMPORTED GLOBAL)
set_target_properties(ptp_sigma PROPERTIES
  IMPORTED_LOCATION ${PTP_SIGMA_BYPRODUCT}
  INTERFACE_INCLUDE_DIRECTORIES ${PTP_SIGMA_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}
  INTERFACE_LINK_LIBRARIES ${LIBUSB_LIBRARIES}
)
add_dependencies(ptp_sigma PTP_SIGMA_EP)

# Create the INDI driver executable
add_executable(indi_sigma_ccd
  sigma_ccd.cpp
  sigma_backend.cpp
)
add_dependencies(indi_sigma_ccd PTP_SIGMA_EP)

# Include directories
target_include_directories(indi_sigma_ccd PRIVATE
  ${INDI_INCLUDE_DIRS}
  ${CFITSIO_INCLUDE_DIRS}
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
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Install the ptp_sigma deliverables into this package
# Headers
install(DIRECTORY "${PTP_SIGMA_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}/"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

# Libraries (.so and optionally .a), respecting lib vs lib64
install(DIRECTORY "${PTP_SIGMA_PREFIX}/${CMAKE_INSTALL_LIBDIR}/"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        FILES_MATCHING
          REGEX ".*/libptp_sigma\\.so(\\..*)?$"
          REGEX ".*/libptp_sigma\\.a$")

install(TARGETS indi_sigma_ccd
  RUNTIME DESTINATION bin
)

install(FILES indi_sigma.xml
  DESTINATION ${INDI_DATA_DIR}
)
