#!/usr/bin/env bash
set -euo pipefail

# Config (override with env vars if you want)
INFLUX_TOKEN="${INFLUX_TOKEN:-${TOKEN:-}}"
INFLUX_ORG="${INFLUX_ORG:-TomOrg}"
INFLUX_BUCKET="${INFLUX_BUCKET:-metrics}"
MEASUREMENT="${MEASUREMENT:-charger}"
PRECISION="${PRECISION:-s}" # s, ms, ns, etc
INFLUX_URL="${INFLUX_URL:-http://influx.localdomain:8086}"

if [ -z "$INFLUX_TOKEN" ]; then
  echo "ERROR: set INFLUX_TOKEN (or TOKEN) environment variable" >&2
  exit 2
fi

# Read full stdin
json="$(cat -)"

# Build field set from JSON:
# - numbers are emitted as numbers
# - booleans as true/false
# - strings are JSON-encoded (quoted)
fields="$(jq -r '.measurements
	| to_entries
	| map(
	if (.value|type) == "string" then "\(.key)=\(.value|@json)"
	elif (.value|type) == "number" then "\(.key)=\(.value)"
	elif (.value|type) == "boolean" then "\(.key)=\(.value)"
	else "\(.key)=\(.value|@json)"
	end
	)
	| join(",")' <<<"$json")"

if [ -z "$fields" ] || [ "$fields" = "null" ]; then
  echo "ERROR: no fields extracted from JSON" >&2
  exit 3
fi

# Compose line-protocol (no timestamp -> server time)
line="${MEASUREMENT} ${fields}"

# Debug: print the line to stderr (disable if noisy)
# echo "LINE: $line" >&2

# Send to Influx
curl -sS -i -XPOST "${INFLUX_URL}/api/v2/write?org=${INFLUX_ORG}&bucket=${INFLUX_BUCKET}&precision=${PRECISION}" \
  -H "Authorization: Token ${INFLUX_TOKEN}" \
  -H "Content-Type: text/plain; charset=utf-8" \
  --data-binary "$line"
