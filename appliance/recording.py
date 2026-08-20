"""Local record + pull-on-request (SWRS-REC-003 / UN-008)."""

from __future__ import annotations

import hashlib
import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore


def _dump(data: Dict[str, Any]) -> str:
    if yaml is not None:
        return yaml.safe_dump(data)
    return json.dumps(data, indent=2) + "\n"


def _load(text: str) -> Dict[str, Any]:
    if yaml is not None:
        return yaml.safe_load(text) or {}
    return json.loads(text) if text.strip() else {}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class RecordingStore:
    def __init__(self, root: Optional[Path] = None) -> None:
        data = Path("/data/armpi_recordings")
        fallback = Path.home() / "armpi_recordings"
        self.root = root or (data if data.parent.exists() else fallback)
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._active_id: Optional[str] = None
        self._joints_fp = None

    def start(self, experiment_id: str, extra: Optional[Dict[str, Any]] = None) -> str:
        with self._lock:
            if self._active_id:
                raise RuntimeError(f"already recording {self._active_id}")
            day = datetime.now().strftime("%Y-%m-%d")
            dest = self.root / day / experiment_id
            dest.mkdir(parents=True, exist_ok=True)
            (dest / "video").mkdir(exist_ok=True)
            meta = {
                "session_id": experiment_id,
                "start_time": _utc_now(),
                "device_id": "armpi_mini",
                **(extra or {}),
            }
            (dest / "metadata.yaml").write_text(_dump(meta), encoding="utf-8")
            self._joints_fp = (dest / "joints.jsonl").open("a", encoding="utf-8")
            self._active_id = experiment_id
            return experiment_id

    def append_joints(self, joints_deg: List[float], t_s: float) -> None:
        with self._lock:
            if self._joints_fp is None:
                return
            self._joints_fp.write(
                json.dumps({"t_s": t_s, "joints_deg": joints_deg}) + "\n"
            )
            self._joints_fp.flush()

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            if not self._active_id:
                raise RuntimeError("not recording")
            session = self._active_id
            if self._joints_fp:
                self._joints_fp.close()
                self._joints_fp = None
            dest = self._session_dir(session)
            meta_path = dest / "metadata.yaml"
            meta = _load(meta_path.read_text()) if meta_path.exists() else {}
            meta["end_time"] = _utc_now()
            files = {}
            for path in dest.rglob("*"):
                if path.is_file() and path.name != "metadata.yaml":
                    digest = hashlib.sha256(path.read_bytes()).hexdigest()
                    files[str(path.relative_to(dest))] = {
                        "bytes": path.stat().st_size,
                        "sha256": digest,
                    }
            meta["files"] = files
            meta_path.write_text(_dump(meta), encoding="utf-8")
            self._active_id = None
            return meta

    def _session_dir(self, session_id: str) -> Path:
        matches = list(self.root.glob(f"*/{session_id}"))
        if not matches:
            raise FileNotFoundError(session_id)
        return matches[0]

    def list_sessions(self) -> List[Dict[str, Any]]:
        out = []
        for meta in self.root.glob("*/*/metadata.yaml"):
            data = _load(meta.read_text())
            data["path"] = str(meta.parent)
            out.append(data)
        return sorted(out, key=lambda m: m.get("start_time", ""), reverse=True)

    def session_meta(self, session_id: str) -> Dict[str, Any]:
        dest = self._session_dir(session_id)
        return _load((dest / "metadata.yaml").read_text())

    def blob_path(self, session_id: str, rel: str = "joints.jsonl") -> Path:
        dest = self._session_dir(session_id)
        path = (dest / rel).resolve()
        if not str(path).startswith(str(dest.resolve())):
            raise ValueError("invalid path")
        return path

    @property
    def active_id(self) -> Optional[str]:
        return self._active_id
