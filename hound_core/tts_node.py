"""Experiment-logistics text-to-speech for HOUND.

Speaks phrases over a local ALSA device (USB speaker on the Jetson). Triggers:

  * ``std_msgs/String`` on speak_topic (free-form)
  * ``std_srvs/Trigger`` on test_service (device check)
  * SSoT ``tts.events`` — Bool / Empty / String topic watchers

Preferred engine is offline Piper (``voice: jarvis`` / ``friday``). Falls
back to ``espeak-ng`` if the Piper model or binary is missing. Playback is
``aplay``. A worker thread serializes utterances.
"""

from __future__ import annotations

import os
import queue
import shutil
import subprocess
import tempfile
import threading
import wave
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger


_LATCH_QOS = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
_EVENT_QOS = QoSProfile(
    depth=20,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.RELIABLE,
)

# Piper neural voices (British). Not the film actors — closest offline match.
_VOICE_PRESETS: Dict[str, Dict[str, Any]] = {
    "jarvis": {
        "model": "en_GB-alan-medium.onnx",
        "length_scale": 1.08,
    },
    "friday": {
        "model": "en_GB-alba-medium.onnx",
        "length_scale": 1.0,
    },
    "alan": {
        "model": "en_GB-alan-medium.onnx",
        "length_scale": 1.08,
    },
    "alba": {
        "model": "en_GB-alba-medium.onnx",
        "length_scale": 1.0,
    },
}


def _parse_alsa_cards(text: str) -> List[Dict[str, str]]:
    """Parse ``/proc/asound/cards`` into ``[{index, id, driver, name}, ...]``."""
    cards: List[Dict[str, str]] = []
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        line = lines[i]
        if line[:2].strip().isdigit() and "[" in line:
            idx = line.split("[", 1)[0].strip()
            ident = line.split("[", 1)[1].split("]", 1)[0].strip()
            rest = line.split("]:", 1)[-1].strip()
            driver = rest.split(" - ", 1)[0].strip()
            name = rest.split(" - ", 1)[-1].strip() if " - " in rest else rest
            cards.append(
                {"index": idx, "id": ident, "driver": driver, "name": name}
            )
        i += 1
    return cards


def detect_usb_playback_device() -> str:
    """Prefer a USB-Audio card (speaker); fall back to ALSA default."""
    path = Path("/proc/asound/cards")
    if not path.is_file():
        return "default"
    try:
        cards = _parse_alsa_cards(path.read_text(encoding="utf-8"))
    except OSError:
        return "default"
    for card in cards:
        blob = f"{card['driver']} {card['name']}".lower()
        if "usb" in blob:
            return f"plughw:CARD={card['id']},DEV=0"
    return "default"


def _workspace_roots() -> List[Path]:
    roots: List[Path] = []
    env = os.environ.get("ROS_WORKSPACE", "").strip()
    if env:
        roots.append(Path(env))
    roots.extend(
        (
            Path("/root/colcon_ws"),
            Path("/home/hound/colcon_ws"),
        )
    )
    seen = set()
    out: List[Path] = []
    for root in roots:
        key = str(root)
        if key in seen:
            continue
        seen.add(key)
        out.append(root)
    return out


def default_piper_dir() -> Path:
    for root in _workspace_roots():
        candidate = root / "src" / "hound_core" / "share" / "piper"
        if candidate.is_dir():
            return candidate
    return Path("/root/colcon_ws/src/hound_core/share/piper")


class TtsNode(Node):
    def __init__(self) -> None:
        super().__init__("tts")

        self.declare_parameter("speak_topic", "/hound/speak")
        self.declare_parameter("test_service", "/hound/tts/test")
        self.declare_parameter("device", "")
        self.declare_parameter("engine", "piper")
        self.declare_parameter("voice", "jarvis")
        self.declare_parameter("rate_wpm", 175)
        self.declare_parameter("amplitude", 140)
        self.declare_parameter("length_scale", 0.0)
        self.declare_parameter("piper_dir", "")
        self.declare_parameter("piper_bin", "")
        self.declare_parameter("startup_phrase", "Hound audio ready")
        self.declare_parameter("events_yaml", "")
        self.declare_parameter("max_queue", 16)

        device = str(self.get_parameter("device").value).strip()
        self._device = device or detect_usb_playback_device()
        self._engine = str(self.get_parameter("engine").value).strip().lower() or "piper"
        self._voice = str(self.get_parameter("voice").value).strip() or "jarvis"
        self._rate = int(self.get_parameter("rate_wpm").value)
        self._amplitude = int(self.get_parameter("amplitude").value)
        self._length_scale = float(self.get_parameter("length_scale").value)
        self._max_queue = max(1, int(self.get_parameter("max_queue").value))
        piper_dir = str(self.get_parameter("piper_dir").value).strip()
        self._piper_dir = Path(piper_dir) if piper_dir else default_piper_dir()
        self._piper_bin = str(self.get_parameter("piper_bin").value).strip()
        self._piper_voice = None
        self._piper_model: Optional[Path] = None
        self._active_engine = "espeak-ng"

        # Subscribe before Piper load so --once pubs during warmup are not dropped.
        self._queue: queue.Queue[str] = queue.Queue(maxsize=self._max_queue)
        self._stop = threading.Event()
        speak_topic = str(self.get_parameter("speak_topic").value)
        self.create_subscription(String, speak_topic, self._on_speak, _EVENT_QOS)
        test_service = str(self.get_parameter("test_service").value)
        self.create_service(Trigger, test_service, self._on_test)
        self._event_seen: Dict[str, bool] = {}
        self._bind_events(self._load_events())

        self._resolve_engine()
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

        if shutil.which("aplay") is None:
            self.get_logger().error(
                "aplay not found. In mushr_jazzy: apt-get install -y alsa-utils"
            )

        self.get_logger().info(
            f"TTS online device={self._device} engine={self._active_engine} "
            f"voice={self._voice} model={self._piper_model or '-'} "
            f"speak={speak_topic} test={test_service}"
        )

        startup = str(self.get_parameter("startup_phrase").value).strip()
        if startup:
            self.enqueue(startup)

    def _voice_spec(self) -> Tuple[str, float]:
        key = self._voice.lower()
        preset = _VOICE_PRESETS.get(key)
        if preset:
            scale = float(preset["length_scale"])
            if self._length_scale > 0.0:
                scale = self._length_scale
            return str(preset["model"]), scale
        model = self._voice
        if not model.endswith(".onnx"):
            model = f"{model}.onnx"
        scale = self._length_scale if self._length_scale > 0.0 else 1.0
        return model, scale

    def _find_piper_model(self, filename: str) -> Optional[Path]:
        name = Path(filename).name
        candidates = [
            self._piper_dir / "voices" / name,
            self._piper_dir / name,
            Path(filename),
        ]
        for path in candidates:
            if path.is_file():
                return path
        return None

    def _find_piper_bin(self) -> Optional[str]:
        if self._piper_bin:
            path = Path(self._piper_bin)
            if path.is_file() and os.access(path, os.X_OK):
                return str(path)
        found = shutil.which("piper")
        if found:
            return found
        for candidate in (
            self._piper_dir / "bin" / "piper",
            self._piper_dir / "piper",
        ):
            if candidate.is_file() and os.access(candidate, os.X_OK):
                return str(candidate)
        return None

    def _load_piper_python(self, model: Path) -> bool:
        try:
            from piper import PiperVoice  # type: ignore
        except ImportError:
            return False
        try:
            self._piper_voice = PiperVoice.load(str(model))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Piper Python load failed ({model}): {exc}")
            self._piper_voice = None
            return False
        return True

    def _resolve_engine(self) -> None:
        want_piper = self._engine in ("piper", "auto")
        want_espeak = self._engine in ("espeak-ng", "espeak", "auto")
        if self._engine not in ("piper", "espeak-ng", "espeak", "auto"):
            self.get_logger().warning(
                f"unknown tts.engine={self._engine!r}; trying piper then espeak-ng"
            )
            want_piper = True
            want_espeak = True

        if want_piper:
            model_name, scale = self._voice_spec()
            self._length_scale = scale
            model = self._find_piper_model(model_name)
            if model is None:
                self.get_logger().warning(
                    f"Piper model missing ({model_name}). "
                    "Run: bash /root/colcon_ws/src/hound_core/scripts/setup_tts.sh"
                )
            else:
                self._piper_model = model
                if self._load_piper_python(model):
                    self._active_engine = "piper-python"
                    return
                if self._find_piper_bin():
                    self._active_engine = "piper-bin"
                    return
                self.get_logger().warning(
                    "Piper model found but no piper binary / python module. "
                    "Run setup_tts.sh or: pip3 install piper-tts"
                )

        if want_espeak or self._active_engine == "espeak-ng":
            if shutil.which("espeak-ng") is None and shutil.which("espeak") is None:
                self.get_logger().error(
                    "No TTS engine available. "
                    "Run setup_tts.sh (Piper) or: apt-get install -y espeak-ng alsa-utils"
                )
            self._active_engine = "espeak-ng"

    def _load_events(self) -> List[Dict[str, Any]]:
        raw = str(self.get_parameter("events_yaml").value).strip()
        if not raw:
            return []
        try:
            data = yaml.safe_load(raw)
        except yaml.YAMLError as exc:
            self.get_logger().error(f"tts.events YAML parse failed: {exc}")
            return []
        if not isinstance(data, list):
            self.get_logger().error("tts.events must be a list")
            return []
        return [e for e in data if isinstance(e, dict)]

    def _bind_events(self, events: List[Dict[str, Any]]) -> None:
        for idx, event in enumerate(events):
            topic = str(event.get("topic") or "").strip()
            kind = str(event.get("type") or "bool").strip().lower()
            if not topic:
                self.get_logger().warning(f"tts.events[{idx}] missing topic; skip")
                continue
            latch = bool(event.get("latched", kind == "bool"))
            qos = _LATCH_QOS if latch else _EVENT_QOS
            key = f"{topic}:{kind}:{idx}"
            self._event_seen[key] = False
            if kind == "bool":
                self.create_subscription(
                    Bool,
                    topic,
                    self._make_bool_cb(key, event),
                    qos,
                )
            elif kind == "empty":
                self.create_subscription(
                    Empty,
                    topic,
                    self._make_empty_cb(key, event),
                    qos,
                )
            elif kind == "string":
                self.create_subscription(
                    String,
                    topic,
                    self._make_string_cb(key, event),
                    qos,
                )
            else:
                self.get_logger().warning(
                    f"tts.events[{idx}] unknown type={kind!r}; skip"
                )
                continue
            self.get_logger().info(f"TTS event {kind} {topic}")

    def _skip_idle_latch(self, key: str, is_idle: bool) -> bool:
        """Drop only the first *idle* latched sample (False / empty current state).

        A first True must speak — TTS often starts before ArUco, so the first
        hunt/align edge is the real trigger, not leftover latch.
        """
        if self._event_seen[key]:
            return False
        self._event_seen[key] = True
        return is_idle

    def _make_bool_cb(self, key: str, event: Dict[str, Any]):
        on_true = str(event.get("on_true") or "").strip()
        on_false = str(event.get("on_false") or "").strip()

        def _cb(msg: Bool) -> None:
            if self._skip_idle_latch(key, not bool(msg.data)):
                return
            phrase = on_true if msg.data else on_false
            if phrase:
                self.enqueue(phrase)

        return _cb

    def _make_empty_cb(self, key: str, event: Dict[str, Any]):
        phrase = str(event.get("phrase") or "").strip()

        def _cb(_msg: Empty) -> None:
            if event.get("latched") and self._skip_idle_latch(key, True):
                return
            if phrase:
                self.enqueue(phrase)

        return _cb

    def _make_string_cb(self, key: str, event: Dict[str, Any]):
        template = str(event.get("phrase") or "").strip()

        def _cb(msg: String) -> None:
            if event.get("latched") and self._skip_idle_latch(key, not bool(str(msg.data or "").strip())):
                return
            text = str(msg.data or "").strip()
            if template:
                phrase = (
                    template.replace("{data}", text)
                    if "{data}" in template
                    else template
                )
            else:
                phrase = text
            if phrase:
                self.enqueue(phrase)

        return _cb

    def _on_speak(self, msg: String) -> None:
        text = str(msg.data or "").strip()
        if text:
            self.enqueue(text)

    def _on_test(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        phrase = "Audio check"
        ok = self.enqueue(phrase)
        res.success = ok
        res.message = "queued" if ok else "queue full"
        return res

    def enqueue(self, text: str) -> bool:
        phrase = " ".join(text.split())
        if not phrase:
            return False
        try:
            self._queue.put_nowait(phrase)
            return True
        except queue.Full:
            self.get_logger().warning(f"TTS queue full; drop: {phrase!r}")
            return False

    def _run_worker(self) -> None:
        while not self._stop.is_set():
            try:
                phrase = self._queue.get(timeout=0.25)
            except queue.Empty:
                continue
            try:
                self._speak(phrase)
            except Exception as exc:  # noqa: BLE001 — keep worker alive
                self.get_logger().error(f"TTS playback failed: {exc}")
            finally:
                self._queue.task_done()

    def _synth_piper_python(self, phrase: str, wav_path: Path) -> bool:
        voice = self._piper_voice
        if voice is None:
            return False
        with wave.open(str(wav_path), "wb") as wav_file:
            syn = getattr(voice, "synthesize_wav", None)
            if callable(syn):
                try:
                    syn(phrase, wav_file, length_scale=self._length_scale)
                    return True
                except TypeError:
                    syn(phrase, wav_file)
                    return True
            syn = getattr(voice, "synthesize", None)
            if callable(syn):
                try:
                    syn(phrase, wav_file, length_scale=self._length_scale)
                    return True
                except TypeError:
                    syn(phrase, wav_file)
                    return True
        return False

    def _synth_piper_bin(self, phrase: str, wav_path: Path) -> bool:
        binary = self._find_piper_bin()
        if binary is None or self._piper_model is None:
            return False
        cmd = [
            binary,
            "--model",
            str(self._piper_model),
            "--output_file",
            str(wav_path),
            "--length_scale",
            str(self._length_scale),
        ]
        synth = subprocess.run(
            cmd,
            input=phrase + "\n",
            check=False,
            capture_output=True,
            text=True,
            timeout=30.0,
        )
        if synth.returncode != 0:
            err = (synth.stderr or synth.stdout or "").strip()
            self.get_logger().error(f"piper failed (code={synth.returncode}): {err}")
            return False
        return wav_path.is_file() and wav_path.stat().st_size > 44

    def _synth_espeak(self, phrase: str, wav_path: Path) -> bool:
        engine = shutil.which("espeak-ng") or shutil.which("espeak")
        if engine is None:
            self.get_logger().error("cannot speak; espeak-ng missing")
            return False
        voice = self._voice
        if voice.lower() in _VOICE_PRESETS:
            voice = "en-gb"
        synth = subprocess.run(
            [
                engine,
                "-v",
                voice,
                "-s",
                str(self._rate),
                "-a",
                str(self._amplitude),
                "-w",
                str(wav_path),
                phrase,
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=20.0,
        )
        if synth.returncode != 0:
            err = (synth.stderr or synth.stdout or "").strip()
            self.get_logger().error(
                f"{engine} failed (code={synth.returncode}): {err}"
            )
            return False
        return True

    def _speak(self, phrase: str) -> None:
        aplay = shutil.which("aplay")
        if aplay is None:
            self.get_logger().error("cannot speak; aplay missing")
            return
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as handle:
            wav_path = Path(handle.name)
        try:
            ok = False
            if self._active_engine == "piper-python":
                ok = self._synth_piper_python(phrase, wav_path)
                if not ok:
                    ok = self._synth_piper_bin(phrase, wav_path)
            elif self._active_engine == "piper-bin":
                ok = self._synth_piper_bin(phrase, wav_path)
            if not ok:
                ok = self._synth_espeak(phrase, wav_path)
            if not ok:
                return
            play = subprocess.run(
                [aplay, "-q", "-D", self._device, str(wav_path)],
                check=False,
                capture_output=True,
                text=True,
                timeout=30.0,
            )
            if play.returncode != 0:
                err = (play.stderr or play.stdout or "").strip()
                self.get_logger().error(
                    f"aplay -D {self._device} failed (code={play.returncode}): {err}"
                )
                return
            self.get_logger().info(f"spoke: {phrase}")
        finally:
            try:
                wav_path.unlink(missing_ok=True)
            except OSError:
                pass

    def shutdown(self) -> None:
        self._stop.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)


def main() -> None:
    rclpy.init()
    node = TtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
