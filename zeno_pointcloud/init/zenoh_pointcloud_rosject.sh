#!/bin/bash
trap "killall zenohd; exit" SIGINT SIGTERM

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
zenoh-bridge-ros2dds -c $SCRIPT_DIR/../config/zenoh-plugin-dds/rosject_config.json5
