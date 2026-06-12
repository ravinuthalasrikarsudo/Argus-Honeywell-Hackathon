#!/usr/bin/env bash
# Pack the LEAN ARGUS project for native-Ubuntu migration:
#  - argus_code.tar.gz : code + scripts + docs + models + patched third_party + plots
#                        + venv requirements + MIGRATE_SETUP.md   (~0.5 GB)
#  - (optional) argus_bags.tar.gz : the pre-recorded demo bags    (run with BAGS=1)
# Excludes build/install/log + raw bags + venvs (rebuilt/re-recorded on target).
set -uo pipefail
WS=/home/vittal/argus
OUT=/mnt/c/Users/vitta/argus_migrate            # NOT under OneDrive (avoid sync churn)
mkdir -p "$OUT"

echo "[pack] freezing venv requirements..."
~/.venvs/argus-eval/bin/pip freeze > "$WS/requirements-eval.txt" 2>/dev/null || echo "(eval venv freeze failed)"
~/.venvs/argus-sp/bin/pip   freeze > "$WS/requirements-sp.txt"   2>/dev/null || echo "(sp venv freeze failed)"

echo "[pack] building argus_code.tar.gz (lean code set)..."
cd /home/vittal
tar czf "$OUT/argus_code.tar.gz" \
  --exclude='argus/build' --exclude='argus/install' --exclude='argus/log' \
  --exclude='argus/data/bags' \
  --exclude='*/__pycache__' --exclude='*.pyc' \
  --exclude='*/.git' \
  --exclude='argus/third_party/*/build' --exclude='argus/third_party/*/install' \
  --exclude='argus/third_party/*/log' \
  argus/src argus/scripts argus/docs argus/models argus/third_party \
  argus/data/scenarios argus/data/eval \
  argus/README.md argus/MIGRATE_SETUP.md \
  argus/requirements-eval.txt argus/requirements-sp.txt

if [ "${BAGS:-0}" = "1" ]; then
  echo "[pack] building argus_bags.tar.gz (demo bags, ~11 GB)..."
  tar czf "$OUT/argus_bags.tar.gz" \
    argus/data/bags/baseline_live_day6 argus/data/bags/map_demo argus/data/bags/baseline_ABC
fi

echo "[pack] DONE. Contents of $OUT:"
ls -lh "$OUT"
