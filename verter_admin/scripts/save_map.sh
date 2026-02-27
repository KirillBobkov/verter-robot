#!/usr/bin/env bash
# save_map.sh — Save current Nav2 map to a YAML file.
#
# Usage:
#   ./save_map.sh <output_path_without_extension>
#
# Example:
#   ./save_map.sh ~/maps/my_map
#
# The script exits with the map_saver exit code so CI can detect failures.

set -e

if [ -z "$1" ]; then
  echo "Usage: $0 <output_path_without_extension>" >&2
  exit 1
fi

ros2 run nav2_map_server map_saver_cli -f "$1"
exit $?
