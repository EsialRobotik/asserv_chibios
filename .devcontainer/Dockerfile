FROM mcr.microsoft.com/devcontainers/cpp:ubuntu-24.04

#ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="none"

# Optionally install the cmake for vcpkg
#COPY ./reinstall-cmake.sh /tmp/

# RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
#         chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
#     fi \
#     && rm -f /tmp/reinstall-cmake.sh

# [Optional] Uncomment this section to install additional vcpkg ports.
# RUN su vscode -c "${VCPKG_ROOT}/vcpkg install <your-port-name-here>"

 RUN apt-get update \
     && apt-get -y install openocd binutils-arm-none-eabi=2.42-1ubuntu1+23 gcc-arm-none-eabi=15:13.2.rel1-2 libnewlib-arm-none-eabi=4.4.0.20231231-2 libstdc++-arm-none-eabi-newlib=15:13.2.rel1-2+26 gdb-multiarch