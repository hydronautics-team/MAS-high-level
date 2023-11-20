#!/usr/bin/env python3

import argparse
import os
import platform
import shutil
import subprocess as sp
import sys
from pathlib import Path

RPI_TOOLCHAIN = "cmake/toolchains/aarch64-linux.cmake"


def get_source_dir() -> Path:
    return Path(__file__).parent.resolve()


def build_ros2(args, build_suffix) -> None:
    ros2_ws = get_source_dir() / "ros2"

    if args.rpi:
        setup_bash_file = "/sysroot/opt/ros/iron/setup.bash"
        toolchain_arg = f"--cmake-args -DCMAKE_TOOLCHAIN_FILE={get_source_dir() / RPI_TOOLCHAIN}"
    else:
        setup_bash_file = "/opt/ros/iron/setup.bash"
        toolchain_arg = ""

    build_dir = f"build-{build_suffix}"
    install_dir = f"install-{build_suffix}"

    if args.clean:
        shutil.rmtree(ros2_ws / build_dir, ignore_errors=True)
        shutil.rmtree(ros2_ws / install_dir, ignore_errors=True)

    sp.check_call(f"source {setup_bash_file} && colcon build --cmake-force-configure "
                  f"--build-base {build_dir} --install-base {install_dir} {toolchain_arg}",
                  cwd=ros2_ws, shell=True, executable='/bin/bash')


# def build_qt(args, build_suffix) -> None:
#     build_dir = Path(f"build-{build_suffix}").resolve()
#     if args.clean:
#         shutil.rmtree(build_dir, ignore_errors=True)
#     build_dir.mkdir(exist_ok=True)

#     if args.rpi:
#         command = f"cmake {get_source_dir()} --toolchain {get_source_dir() / RPI_TOOLCHAIN}"
#     else:
#         command = f"cmake {get_source_dir()}"

#     sp.check_call(command, cwd=build_dir, shell=True)
#     sp.check_call("make", cwd=build_dir)


def main(args) -> None:
    if args.rpi:
        build_suffix = "aarch64-linux"
    else:
        build_suffix = f"{platform.processor()}-{platform.system().lower()}"

    # build_qt(args, build_suffix)
    build_ros2(args, build_suffix)


if __name__ == "__main__":
    if not os.getenv("DOCKER_ENV"):
        print("You are not in docker!")
        sys.exit(0)

    parser = argparse.ArgumentParser(description='Build options')
    parser.add_argument('--rpi', default=False, action='store_true',
                        help='Собрать проект под Raspberry')
    parser.add_argument('--clean', default=False, action='store_true',
                        help='Пересборка таргетов')
    main(parser.parse_args())
