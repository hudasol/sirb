#!/bin/sh
# Start or stop N PX4 SIH quadrotor instances (simulation only). Usage: sitl_instances.sh start N | stop
# PX4_DIR is a built PX4-Autopilot checkout (make px4_sitl_sih); see docs/SITL.md.
PX4_DIR=${PX4_DIR:-/home/claude/px4work/PX4-Autopilot}
WORK=${SITL_WORK:-/home/claude/px4work}
case "$1" in
  start)
    for i in $(seq 0 $(( $2 - 1 ))); do
      mkdir -p "$WORK/run$i"
      (cd "$WORK/run$i" && rm -rf parameters* dataman *.bson 2>/dev/null
       PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=sihsim_quadx nohup "$PX4_DIR/build/px4_sitl_sih/bin/px4" \
         -i "$i" -d "$PX4_DIR/build/px4_sitl_sih/etc" > "$WORK/run$i.log" 2>&1 &)
    done ;;
  stop) pkill -x px4; sleep 2 ;;
esac
