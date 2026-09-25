#!/usr/bin/env bash
set -Eeuo pipefail
export RAICOM_ALL_START_NAV=false
export RAICOM_ALL_CLEAN_START=false
export RAICOM_KEEP_NAV_AFTER_ALL=true
exec "$(dirname "$0")/run_national_all_tasks_sim.sh" "$@"
