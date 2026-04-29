#!/bin/bash
# Lifecycle helper for the runner. Runs wherever the file lives — no hardcoded
# paths. Usage:
#   ./serve.sh start [scene]   # scene defaults to the indicator-check example
#   ./serve.sh stop
#   ./serve.sh status
#   ./serve.sh logs [N]
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
export PATH="$HOME/.local/bin:$PATH"

LOG="$SCRIPT_DIR/runner.log"
PIDFILE="$SCRIPT_DIR/runner.pid"
DEFAULT_SCENE="examples.scenes.mobile_aloha_piper_indicator_check"

case "${1:-status}" in
  start)
    SCENE="${2:-$DEFAULT_SCENE}"
    if [ -f "$PIDFILE" ] && kill -0 "$(cat $PIDFILE)" 2>/dev/null; then
      echo "already running (pid $(cat $PIDFILE))"; exit 0
    fi
    pkill -f 'mwb run|mujoco_workbench.cli' 2>/dev/null || true
    sleep 1
    nohup uv run mwb run "$SCENE" --host 127.0.0.1 --port 8080 \
      > "$LOG" 2>&1 < /dev/null &
    echo $! > "$PIDFILE"
    sleep 6
    if kill -0 "$(cat $PIDFILE)" 2>/dev/null; then
      echo "started pid=$(cat $PIDFILE), scene=$SCENE, log=$LOG"
      tail -8 "$LOG"
    else
      echo "failed to start; see $LOG"; tail -20 "$LOG"; exit 1
    fi
    ;;
  stop)
    if [ -f "$PIDFILE" ]; then kill "$(cat $PIDFILE)" 2>/dev/null || true; fi
    pkill -f 'mwb run|mujoco_workbench.cli' 2>/dev/null || true
    rm -f "$PIDFILE"
    echo stopped
    ;;
  status)
    if pgrep -af 'mwb run|mujoco_workbench.cli' >/dev/null; then
      echo running:; pgrep -af 'mwb run|mujoco_workbench.cli'
      ss -tlnp 2>/dev/null | grep :8080 || true
    else
      echo stopped
    fi
    ;;
  logs)
    tail -n "${2:-50}" "$LOG"
    ;;
  *)
    echo "usage: serve.sh {start [scene]|stop|status|logs [N]}"; exit 2
    ;;
esac
