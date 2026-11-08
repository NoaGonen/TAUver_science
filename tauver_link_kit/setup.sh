#!/usr/bin/env bash
# TAUVER Link Kit · setup.sh
# Run ONCE on every machine: laptop A, laptop B, the CMD station, the Jetson.
#   bash ~/tauver_link_kit/setup.sh
# Safe to run again: it only adds what is missing.

KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -t 1 ]; then G=$'\033[32m'; Y=$'\033[33m'; B=$'\033[1m'; N=$'\033[0m'; else G=""; Y=""; B=""; N=""; fi
relogin=0
USER="${USER:-$(id -un)}"

echo "${B}== TAUVER link kit setup on $(hostname) ==${N}"
echo "kit folder: $KIT"

# 1. scripts executable
chmod +x "$KIT/tv" "$KIT/setup.sh" "$KIT"/host/*.py 2>/dev/null
echo "${G}✔${N} scripts are executable"

# 2. settings file
if [ ! -f "$KIT/tauver.conf" ]; then
  printf 'TAUVER_DOMAIN_ID=27\nJETSON_IP=192.168.55.1\nJETSON_USER=\nPEER_IP=\n' > "$KIT/tauver.conf"
  echo "${G}✔${N} created tauver.conf (ROS_DOMAIN_ID 27)"
else
  echo "${G}✔${N} tauver.conf already there"
fi

# 3. every new terminal loads ROS 2 + the tv command
if grep -qs "tauver_env.sh" "$HOME/.bashrc"; then
  echo "${G}✔${N} ~/.bashrc already loads the kit"
else
  printf '\n# TAUVER link kit: ROS 2 settings + the "tv" command\nsource "%s/tauver_env.sh"\n' "$KIT" >> "$HOME/.bashrc"
  echo "${G}✔${N} added one line to ~/.bashrc (new terminals load ROS 2 + tv)"
fi
if grep -qsE "^[^#]*ROS_LOCALHOST_ONLY=1" "$HOME/.bashrc"; then
  echo "${Y}!${N} ~/.bashrc contains ROS_LOCALHOST_ONLY=1, which keeps ROS 2 on this machine only. Delete that line."
fi

# 4. ROS 2
if ls /opt/ros/*/setup.bash >/dev/null 2>&1; then
  echo "${G}✔${N} ROS 2 found: $(ls /opt/ros | tr '\n' ' ')"
else
  echo "${Y}!${N} ROS 2 is not installed."
  read -r -p "  Install it now (needs internet, 10-30 min)? [y/N] " yn
  if [[ "$yn" =~ ^[Yy] ]]; then bash "$KIT/tv" install-ros </dev/tty; else
    echo "  later: tv install-ros"; fi
fi

# 5. pyserial (talks to the USB port)
if python3 -c "import serial" 2>/dev/null; then
  echo "${G}✔${N} pyserial installed"
else
  echo "installing pyserial ..."
  sudo apt install -y python3-serial && echo "${G}✔${N} pyserial installed"
fi

# 6. permission to open serial ports
if id -nG "$USER" | grep -qw dialout; then
  echo "${G}✔${N} $USER is in the dialout group"
else
  sudo usermod -aG dialout "$USER" && relogin=1 && echo "${G}✔${N} added $USER to dialout (serial port permission)"
fi

echo
echo "${B}Done.${N} Next:"
if [ "$relogin" = 1 ]; then
  echo "  ${Y}1. Log out and back in (or reboot)${N}, so the serial permission takes effect."
else
  echo "  1. Open a NEW terminal (so it loads the kit)."
fi
echo "  2. Type:  ${B}tv doctor${N}"
[ -f /etc/nv_tegra_release ] && echo "  On the Jetson, later:  tv service install   (the bridge then starts at every boot)"
exit 0
