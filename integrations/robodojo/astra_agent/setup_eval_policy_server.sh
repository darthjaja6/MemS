#!/usr/bin/env bash
set -euo pipefail
policy_python="${8}/bin/python"
exec "$policy_python" "${ROBODOJO_ROOT:?Set ROBODOJO_ROOT to your RoboDojo checkout}/XPolicyLab/setup_policy_server.py" \
  --config_path "${ROBODOJO_ROOT}/XPolicyLab/policy/astra_agent/deploy.yml" \
  --overrides bench_name="$1" task_name="$2" ckpt_name="$3" env_cfg_type="$4" \
  action_type="$5" seed="$6" gpu_id="$7" port="$9" host="${10:-localhost}"
