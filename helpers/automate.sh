#!/usr/bin/env bash
# price_watcher.sh
# Poll the price every POLL_INTERVAL seconds and call start/stop scripts
# Configurable via environment variables documented below.

set -u
# Non-fatal network errors are handled inside loop; do not 'set -e'

# ---------- Configuration (override with env vars) ----------
POLL_INTERVAL="${POLL_INTERVAL:-60}"        # seconds between checks (default 300s = 5min)
CHARGE_THRESHOLD="${CHARGE_THRESHOLD:-22.0}" # start charging when price <= this (units = same as API)
DISCHARGE_THRESHOLD="${DISCHARGE_THRESHOLD:-30.0}" # start discharging when price >= this
HYSTERESIS="${HYSTERESIS:-0.5}"              # hysteresis in same units to avoid flapping

# Commands to run (set full paths if needed). These commands are invoked when a state change is required.
START_CHARGING_CMD="${START_CHARGING_CMD:-./start_charging.sh}"
STOP_CHARGING_CMD="${STOP_CHARGING_CMD:-./stop_charging.sh}"
START_DISCHARGING_CMD="${START_DISCHARGING_CMD:-./start_discharging.sh}"
STOP_DISCHARGING_CMD="${STOP_DISCHARGING_CMD:-./stop_discharging.sh}"

# Where we persist last-known state (charging/discharging). Default is /tmp so no root needed.
STATE_FILE="${STATE_FILE:-/tmp/homebattery_price_watcher.state}"

# If DRY_RUN=1, we echo actions rather than executing commands.
DRY_RUN="${DRY_RUN:-0}"

# Path to your price fetcher (must produce the JSON like your sample).
PRICE_FETCH_CMD="${PRICE_FETCH_CMD:-./helpers/get_prices.sh}"

# ---------- Helper functions ----------
log() { printf '%s %s\n' "$(date -u +"%Y-%m-%dT%H:%M:%SZ")" "$*"; }

# load state from file or initialize
load_state() {
  if [[ -f "$STATE_FILE" ]]; then
    # file format: charging=0|1 discharging=0|1
    # shell-parse it safely:
    # shellcheck disable=SC1090
    source "$STATE_FILE"
    CHARGING="${CHARGING:-0}"
    DISCHARGING="${DISCHARGING:-0}"
  else
    CHARGING=0
    DISCHARGING=0
  fi
}

save_state() {
  cat >"$STATE_FILE" <<EOF
# state file for price_watcher.sh
CHARGING=${CHARGING}
DISCHARGING=${DISCHARGING}
EOF
}

# Compare floats using awk. returns 0 if true.
float_le() { awk -v a="$1" -v b="$2" 'BEGIN{print (a<=b)?0:1}'; }
float_lt() { awk -v a="$1" -v b="$2" 'BEGIN{print (a<b)?0:1}'; }
float_ge() { awk -v a="$1" -v b="$2" 'BEGIN{print (a>=b)?0:1}'; }
float_gt() { awk -v a="$1" -v b="$2" 'BEGIN{print (a>b)?0:1}'; }

# Run a command (or echo if dry-run). Log return code.
run_cmd() {
  local cmd="$1"
  if [[ "$DRY_RUN" == "1" ]]; then
    log "[DRY-RUN] would run: $cmd"
    return 0
  fi
  log "running: $cmd"
  # use 'bash -c' so strings with args work; caller should set full path if necessary
  bash -c "$cmd"
  local rc=$?
  log "command exited with code: $rc"
  return $rc
}

# ---------- Main loop ----------
trap 'log "exiting"; exit 0' SIGINT SIGTERM

load_state
save_state  # ensure file exists

log "starting price_watcher: poll=${POLL_INTERVAL}s charge<=${CHARGE_THRESHOLD} discharge>=${DISCHARGE_THRESHOLD} hyst=${HYSTERESIS} dry_run=${DRY_RUN}"

while true; do
  price="$(./get_current_price.sh)"
  if [[ -z "$price" || "$price" == "null" ]]; then
    log "no current price found; skipping this iteration"
    sleep "$POLL_INTERVAL"
    continue
  fi

  # Normalize price to numeric form (jq already outputs numeric but be defensive)
  # trim whitespace
  price="$(printf '%s' "$price" | tr -d '[:space:]')"

  log "current price = ${price}"

  # Derived thresholds with hysteresis
  charge_stop_threshold=$(awk -v t="$CHARGE_THRESHOLD" -v h="$HYSTERESIS" 'BEGIN{printf("%.6f", t + h)}')
  discharge_stop_threshold=$(awk -v t="$DISCHARGE_THRESHOLD" -v h="$HYSTERESIS" 'BEGIN{printf("%.6f", t - h)}')

  # ---------- Charging logic ----------
  # Start charging if price <= CHARGE_THRESHOLD and not already charging
  if [[ "$(float_le "$price" "$CHARGE_THRESHOLD")" -eq 0 ]] && [[ "$CHARGING" -eq 0 ]]; then
    # ensure we aren't discharging
    if [[ "$DISCHARGING" -eq 1 ]]; then
      log "stopping discharging before starting charging"
      run_cmd "$STOP_DISCHARGING_CMD" || log "warning: stop discharging command failed"
      DISCHARGING=0
    fi
    log "price <= ${CHARGE_THRESHOLD} -> starting charging"
    run_cmd "$START_CHARGING_CMD" || log "warning: start charging command failed"
    CHARGING=1
    save_state
  fi

  # Stop charging if price >= charge_stop_threshold and currently charging
  if [[ "$(float_ge "$price" "$charge_stop_threshold")" -eq 0 ]] && [[ "$CHARGING" -eq 1 ]]; then
    log "price >= ${charge_stop_threshold} -> stopping charging"
    run_cmd "$STOP_CHARGING_CMD" || log "warning: stop charging command failed"
    CHARGING=0
    save_state
  fi

  # ---------- Discharging logic ----------
  # Start discharging if price >= DISCHARGE_THRESHOLD and not already discharging
  if [[ "$(float_ge "$price" "$DISCHARGE_THRESHOLD")" -eq 0 ]] && [[ "$DISCHARGING" -eq 0 ]]; then
    # ensure we aren't charging
    if [[ "$CHARGING" -eq 1 ]]; then
      log "stopping charging before starting discharging"
      run_cmd "$STOP_CHARGING_CMD" || log "warning: stop charging command failed"
      CHARGING=0
    fi
    log "price >= ${DISCHARGE_THRESHOLD} -> starting discharging"
    run_cmd "$START_DISCHARGING_CMD" || log "warning: start discharging command failed"
    DISCHARGING=1
    save_state
  fi

  # Stop discharging if price <= discharge_stop_threshold and currently discharging
  if [[ "$(float_le "$price" "$discharge_stop_threshold")" -eq 0 ]] && [[ "$DISCHARGING" -eq 1 ]]; then
    log "price <= ${discharge_stop_threshold} -> stopping discharging"
    run_cmd "$STOP_DISCHARGING_CMD" || log "warning: stop discharging command failed"
    DISCHARGING=0
    save_state
  fi

  sleep "$POLL_INTERVAL"
done
