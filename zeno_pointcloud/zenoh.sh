#!/bin/bash

cleanup() {
  pkill zenohd
  pkill zenoh-bridge-dds
}

trap cleanup SIGINT
zenohd -e tcp/79.153.77.129:7447 &
zenoh-bridge-dds -e tcp/79.153.77.129:7447 --fwd-discovery
