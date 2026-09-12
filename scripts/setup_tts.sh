#!/usr/bin/env bash
# One-time offline Piper voices for HOUND TTS (Jarvis / Friday).
# Run inside mushr_jazzy. After this, synthesis does not need the network.
set -euo pipefail

ROOT="${ROS_WORKSPACE:-/root/colcon_ws}"
PIPER_DIR="${HOUND_PIPER_DIR:-$ROOT/src/hound_core/share/piper}"
VOICES="$PIPER_DIR/voices"
BIN_DIR="$PIPER_DIR/bin"
HF="https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0"
PIPER_TGZ="https://github.com/rhasspy/piper/releases/download/v1.2.0/piper_arm64.tar.gz"

mkdir -p "$VOICES" "$BIN_DIR"

need_cmd() {
  command -v "$1" >/dev/null 2>&1
}

if ! need_cmd curl && ! need_cmd wget; then
  echo "need curl or wget to fetch Piper + voices" >&2
  exit 1
fi

fetch() {
  local url="$1" dest="$2"
  if [[ -f "$dest" && -s "$dest" ]]; then
    echo "have $dest"
    return 0
  fi
  echo "get $url"
  if need_cmd curl; then
    curl -fL --retry 3 -o "$dest" "$url"
  else
    wget -O "$dest" "$url"
  fi
}

fetch_voice() {
  local locale="$1" name="$2" quality="$3"
  local stem="${locale}-${name}-${quality}"
  local base="$HF/en/${locale}/${name}/${quality}/${stem}"
  fetch "${base}.onnx" "$VOICES/${stem}.onnx"
  fetch "${base}.onnx.json" "$VOICES/${stem}.onnx.json"
}

if ! need_cmd aplay; then
  apt-get update -y
  apt-get install -y --no-install-recommends alsa-utils espeak-ng
fi

# Standalone aarch64 binary (works even if pip onnxruntime is messy).
if [[ ! -x "$BIN_DIR/piper" ]]; then
  tmp="$(mktemp -d)"
  fetch "$PIPER_TGZ" "$tmp/piper_arm64.tar.gz"
  tar -xzf "$tmp/piper_arm64.tar.gz" -C "$tmp"
  # archive is piper/piper + libs
  if [[ -x "$tmp/piper/piper" ]]; then
    cp -a "$tmp/piper/." "$BIN_DIR/"
  else
    echo "unexpected piper archive layout" >&2
    ls -la "$tmp"
    exit 1
  fi
  rm -rf "$tmp"
fi
chmod +x "$BIN_DIR/piper"

# In-process Python (keeps the model warm). Optional.
if ! python3 -c "import piper" >/dev/null 2>&1; then
  pip3 install --break-system-packages 'piper-tts' || \
    echo "pip piper-tts skipped (binary still works)"
fi

# Jarvis = Alan (British male). Friday = Alba (British/Scottish female).
fetch_voice en_GB alan medium
fetch_voice en_GB alba medium

echo
echo "Piper ready:"
echo "  binary  $BIN_DIR/piper"
echo "  voices  $VOICES"
ls -lh "$VOICES"/*.onnx
echo
echo "SSoT: tts.engine: piper / tts.voice: jarvis   # or friday"
echo "Test:  echo 'Systems online.' | $BIN_DIR/piper --model $VOICES/en_GB-alan-medium.onnx --output_file /tmp/jarvis.wav && aplay -D plughw:CARD=Device,DEV=0 /tmp/jarvis.wav"
