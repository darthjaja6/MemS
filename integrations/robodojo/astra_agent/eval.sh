#!/usr/bin/env bash
set -euo pipefail
exec bash "${ROBODOJO_ROOT:?Set ROBODOJO_ROOT to your RoboDojo checkout}/scripts/robodojo.sh" eval \
  --task "$2" --policy-dir "$ROBODOJO_ROOT/XPolicyLab/policy/astra_agent" --ckpt "$3" --env-cfg "$4" \
  --action-type "$5" --seed "$6" --policy-gpu "$7" --env-gpu "$8" \
  --policy-env "$9" --eval-env "${10}"
