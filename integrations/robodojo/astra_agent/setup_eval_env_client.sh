#!/usr/bin/env bash
set -euo pipefail
export PATH="${8}/bin:$PATH"
exec bash "${ROBODOJO_ROOT:?Set ROBODOJO_ROOT to your RoboDojo checkout}/scripts/eval_policy.sh" \
  --root_dir "$ROBODOJO_ROOT" --task_name "$2" --env_cfg_type "$4" --device_id "$7" \
  --policy_name astra_agent --port "${10}" --host "${11:-localhost}" \
  --additional_info "$9" --seed "$6" --headless --enable_cameras
