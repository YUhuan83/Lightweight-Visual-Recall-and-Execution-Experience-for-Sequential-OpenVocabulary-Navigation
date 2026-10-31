#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/smoothing_boundary_repair_20261018/install/local_setup.bash
cd /home/ly/terrain_contract_audit_20261019
g++ -std=c++17 -O1 -I/usr/include/eigen3 -I/home/ly/smoothing_boundary_repair_20261018/candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper/include terrain_metrics_probe.cpp -o terrain_metrics_probe
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python review_maps.py
