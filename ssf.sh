#!/usr/bin/env bash

set -euo pipefail

USER_NAME="selamicetin"
HOST_NAME="220609012"

MACHINE_ID="$(cat /etc/machine-id 2>/dev/null || echo N/A)"
PRODUCT_UUID="$(cat /sys/class/dmi/id/product_uuid 2>/dev/null || echo N/A)"

MACS="$(
  for p in /sys/class/net/*; do
    IF="$(basename "$p")"
    case "$IF" in lo|docker*|veth*|br*|virbr*|zt*) continue ;; esac
    [[ -f "$p/address" ]] && cat "$p/address"
  done | awk 'NF' | paste -sd, -
)"

NOW_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

STUDENT_ID="selamicetin220609012"

RAW="${STUDENT_ID}|${USER_NAME}|${HOST_NAME}|${MACHINE_ID}|${PRODUCT_UUID}|${MACS}|${NOW_UTC}"
HASH="$(printf "%s" "$RAW" | sha256sum | awk '{print $1}')"

printf "\n===== SSF RAW =====\n%s\n" "$RAW"
printf "\n===== SSF SHA256 =====\n%s\n" "$HASH"

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
printf "%s\n" "$HASH" > "${PROJECT_ROOT}/SSF_HASH.txt"

echo "Kaydedildi: ${PROJECT_ROOT}/SSF_HASH.txt"


