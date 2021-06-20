#!/bin/bash
set -eux
STATE=$1
rostopic pub -r 1 \
  "${VEHICLE_NAME}/fsm_node/mode" \
  duckietown_msgs/FSMState \
  "{header: {}, state: \"${STATE}\"}"

