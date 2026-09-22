#!/usr/bin/env bash
# Install into an existing, working RoboDojo/Isaac Python environment.
set -euo pipefail
integration_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
: "${ROBODOJO_ROOT:?Set ROBODOJO_ROOT to the RoboDojo checkout}"
python_env="${VIRTUAL_ENV:-${CONDA_PREFIX:-}}"
: "${python_env:?Activate the working RoboDojo virtualenv or conda environment}"
adapter_target="$ROBODOJO_ROOT/XPolicyLab/policy/astra_agent"
if [[ -e "$adapter_target" || -L "$adapter_target" ]]; then
  if [[ "$(readlink -f "$adapter_target")" != "$integration_dir/astra_agent" ]]; then
    echo "An existing adapter occupies $adapter_target" >&2
    exit 1
  fi
fi
for transport_patch in "$integration_dir/transport-timeout.patch" "$integration_dir/transport-config.patch"; do
  if ! git -C "$ROBODOJO_ROOT" apply --reverse --check "$transport_patch" 2>/dev/null; then
    git -C "$ROBODOJO_ROOT" apply --check "$transport_patch"
    git -C "$ROBODOJO_ROOT" apply "$transport_patch"
  fi
done
python -m pip install -r "$integration_dir/../../requirements.txt" -r "$integration_dir/../../requirements-motion.txt" \
  Pillow==11.3.0 PyYAML==6.0.2 websockets==15.0.1 imageio-ffmpeg==0.6.0
ln -sfn "$integration_dir/astra_agent" "$adapter_target"
if [[ ! -e "$python_env/bin/ffmpeg" ]]; then
  ffmpeg_binary="$(python -c 'import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())')"
  ln -s "$ffmpeg_binary" "$python_env/bin/ffmpeg"
fi
codex --version
