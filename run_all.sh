#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "$0")" && pwd)"
JSON="$ROOT/images/test_cases.json"
OUTDIR="$ROOT/out_validate"
SCRIPT="$ROOT/Model/pipeline.py"
ENV="sittingducks"

mkdir -p "$OUTDIR"

# emit "id<TAB>prompt<TAB>image" lines
python - "$JSON" <<'PY' | while IFS=$'\t' read -r id prompt img; do
import json, sys
for t in json.load(open(sys.argv[1])):
    print(f"{t['test_id']}\t{t['prompt']}\t{t['image_file']}")
PY
    IMG="$ROOT/$img"
    OUT="$OUTDIR/out_${id}.jpg"
    echo "=== test $id : $prompt ==="
    conda run -n "$ENV" --no-capture-output python "$SCRIPT" "$prompt" "$IMG" --out "$OUT"
done
