# TAUVER Link Kit · tauver_env.sh
# The ONE place that sets up ROS 2 for this kit. It is loaded by:
#   - every new terminal (setup.sh adds one line to ~/.bashrc)
#   - the auto-start bridge service on the Jetson
#   - the tv command itself
# Change settings in tauver.conf (or with: tv set ...), not here.
# shellcheck shell=bash

TAUVER_KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export TAUVER_KIT
if [ -f "$TAUVER_KIT/tauver.conf" ]; then
  # shellcheck source=/dev/null
  . "$TAUVER_KIT/tauver.conf"
fi

# ROS 2: Humble first (Jetson JetPack 6 = Ubuntu 22.04 = Humble), then newer ones.
if [ -z "${TAUVER_ROS_DISTRO:-}" ]; then
  for _d in humble jazzy; do
    if [ -f "/opt/ros/$_d/setup.bash" ]; then TAUVER_ROS_DISTRO=$_d; break; fi
  done
  unset _d
fi
if [ -n "${TAUVER_ROS_DISTRO:-}" ] && [ -f "/opt/ros/$TAUVER_ROS_DISTRO/setup.bash" ]; then
  # shellcheck source=/dev/null
  . "/opt/ros/$TAUVER_ROS_DISTRO/setup.bash"
fi

# Machines only see each other if this number is the SAME on all of them.
export ROS_DOMAIN_ID="${TAUVER_DOMAIN_ID:-27}"

# Talk to the whole local network, not just this machine.
if [ "${ROS_DISTRO:-}" = "humble" ]; then
  export ROS_LOCALHOST_ONLY=0
else
  unset ROS_LOCALHOST_ONLY
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
fi

# Plan B (tv peers): name the other machines directly, for networks that block multicast.
if [ -f "$TAUVER_KIT/peers.xml" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$TAUVER_KIT/peers.xml"
  export FASTDDS_DEFAULT_PROFILES_FILE="$TAUVER_KIT/peers.xml"
else
  unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
fi

# The one command to remember.
alias tv='bash "$TAUVER_KIT/tv"'
