#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONNECTION="${CONNECTION:-/dev/serial0}"
BAUD="${BAUD:-57600}"
WIDTH="${WIDTH:-320}"
HEIGHT="${HEIGHT:-180}"
QUALITY="${QUALITY:-30}"
INTERVAL="${INTERVAL:-5}"
CHUNKS_PER_LOOP="${CHUNKS_PER_LOOP:-2}"
EXTRA_ARGS=("$@")

cd "${PROJECT_DIR}"

python3 src/image_stream_sender.py \
  --connection "${CONNECTION}" \
  --baud "${BAUD}" \
  --width "${WIDTH}" \
  --height "${HEIGHT}" \
  --quality "${QUALITY}" \
  --interval "${INTERVAL}" \
  --chunks-per-loop "${CHUNKS_PER_LOOP}" \
  "${EXTRA_ARGS[@]}"
