FROM --platform=linux/arm64 ros:iron as base

SHELL ["/bin/bash", "-l", "-c"]

RUN apt update && apt install -y libboost-system-dev qtbase5-dev libqt5serialport5-dev libwiringpi-dev wiringpi

FROM --platform=linux/amd64 ros:iron as dev

SHELL ["/bin/bash", "-l", "-c"]

RUN mkdir -p /sysroot/usr /sysroot/opt /sysroot/lib
COPY --from=base /lib/ /sysroot/lib/
COPY --from=base /usr/include/ /sysroot/usr/include/
COPY --from=base /usr/lib/ /sysroot/usr/lib/
COPY --from=base /usr/bin/ /sysroot/usr/bin
COPY --from=base /opt/ros/ /sysroot/opt/ros

RUN apt update && apt install -y vim git tree curl cmake make openssh-client sshpass rsync \
    libboost-system-dev gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
    qtbase5-dev libqt5serialport5-dev libwiringpi-dev

# Это какой то баг ROS: https://stackoverflow.com/questions/75529286/crosscompile-with-cmake-and-cannot-generate-a-safe-runtime
RUN ln -s /sysroot/usr/lib/aarch64-linux-gnu/libpython3.10.so /usr/lib/aarch64-linux-gnu/libpython3.10.so

ENV DOCKER_ENV=1
WORKDIR /workspace
