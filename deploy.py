#!/usr/bin/env python3

import argparse
import subprocess as sp
from pathlib import Path

RPI_DEPLOY_PATH = "/home/agent2"


def get_source_dir() -> Path:
    return Path(__file__).parent.resolve()


def main(args) -> None:
    # qt_built_binary = get_source_dir() / "build-aarch64-linux" / "Bort_MAS"
    ros_ws = get_source_dir() / "ros2" / "install-aarch64-linux"

    # sp.check_call(f"sshpass -p {args.password} scp -o StrictHostKeyChecking=no "
    #               f"{qt_built_binary} {args.ip}:{RPI_DEPLOY_PATH}", shell=True)
    sp.check_call(f"sshpass -p {args.password} scp -o StrictHostKeyChecking=no "
                  f"-r {ros_ws} {args.ip}:{RPI_DEPLOY_PATH}", shell=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Build options')
    parser.add_argument('ip', help="Raspberry IP or user@IP")
    parser.add_argument('password', help="Raspberry user password")
    main(parser.parse_args())
