#!/usr/bin/env python3
"""TripGuard Raspberry Pi <-> STM32 binary UART link.

The daemon is the only owner of /dev/serial0.  Camera applications and manual
tests submit READY/PERSON/CLEAR/STATUS commands through a local AF_UNIX socket.
"""

from __future__ import annotations

import argparse
import base64
import collections
import dataclasses
import enum
import json
import logging
import os
import queue
import shlex
import signal
import socket
import sqlite3
import stat
import struct
import subprocess
import tempfile
import threading
import time
import uuid
import urllib.error
import urllib.parse
import urllib.request
from typing import Deque, Dict, List, Optional, Set, Tuple

try:
    import serial  # type: ignore
except ImportError:  # Parser self-test must still work without pyserial.
    serial = None


LOG = logging.getLogger("tripguard-uart")

MAGIC = b"\x54\x50"
PROTOCOL_VERSION = 0x01
MAX_PAYLOAD = 128
FIXED_HEADER_SIZE = 9
CRC_SIZE = 2
MAX_FRAME_SIZE = FIXED_HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE
PARSER_BUFFER_LIMIT = MAX_FRAME_SIZE * 4

FLAG_ACK_REQUIRED = 0x01
FLAG_RETRY = 0x02
KNOWN_FLAGS = FLAG_ACK_REQUIRED | FLAG_RETRY

DEFAULT_PORT = "/dev/serial0"
DEFAULT_BAUD = 115200
DEFAULT_CONTROL_SOCKET = "/run/tripguard-uart/control.sock"
DEFAULT_QUEUE_DB = "/var/lib/tripguard/telemetry.db"
DEFAULT_CAMERA_SERVICE = "tripguard-camera.service"
DEFAULT_CAMERA_ENV = "/run/tripguard-uart/camera-session.env"
DEFAULT_SCAN_STATE = "/var/lib/tripguard/scan-session.json"
DEFAULT_CAMERA_STATE = "/var/lib/tripguard/camera-state.json"
DEFAULT_TB_HOST = os.environ.get(
    "TRIPGUARD_TB_HOST", "https://tb.trankhoinguyeniot.io.vn"
)
DEFAULT_TB_TOKEN = os.environ.get("TRIPGUARD_TB_ACCESS_TOKEN", "")
DEFAULT_TB_INTERFACE = os.environ.get("TRIPGUARD_TB_INTERFACE", "ppp0")
MAX_TELEMETRY_JSON = 2560
MAX_OFFLINE_ROWS = 10000
MAX_EVIDENCE_BYTES = 80 * 1024
EVIDENCE_PRIORITY = 100
ALERT_PRIORITY = 50
SERIAL_READ_TIMEOUT_S = 0.05
SERIAL_WRITE_TIMEOUT_S = 0.25
RECONNECT_DELAY_S = 1.0
HEARTBEAT_INTERVAL_S = 1.0
PEER_TIMEOUT_S = 3.0
ACK_TIMEOUT_S = 0.75
MAX_RETRIES = 3
PERSISTENT_STATE_RETRY_S = 10.0
TX_QUEUE_DEPTH = 64
DUPLICATE_WINDOW_SIZE = 64


class MessageType(enum.IntEnum):
    """Message values shared with Core/Inc/pi_link.h."""

    PI_MSG_HEARTBEAT = 0x01
    PI_MSG_READY = 0x02
    PI_MSG_STATUS_REQUEST = 0x03
    PI_MSG_CAMERA_PERSON = 0x10
    PI_MSG_CAMERA_CLEAR = 0x11
    PI_MSG_SCAN_COMPLETE = 0x12
    PI_MSG_ACK = 0x7F

    STM_MSG_HEARTBEAT = 0x81
    STM_MSG_STATUS = 0x82
    STM_MSG_EVENT = 0x83
    STM_MSG_TELEMETRY_CHUNK = 0x84
    STM_MSG_ALERT_REQUEST = 0x85
    STM_MSG_SCAN_START = 0x86
    STM_MSG_SCAN_STOP = 0x87
    STM_MSG_ACK = 0xFE
    STM_MSG_NACK = 0xFF


PI_MESSAGE_TYPES = {
    MessageType.PI_MSG_HEARTBEAT,
    MessageType.PI_MSG_READY,
    MessageType.PI_MSG_STATUS_REQUEST,
    MessageType.PI_MSG_CAMERA_PERSON,
    MessageType.PI_MSG_CAMERA_CLEAR,
    MessageType.PI_MSG_SCAN_COMPLETE,
    MessageType.PI_MSG_ACK,
}

STM_MESSAGE_TYPES = {
    MessageType.STM_MSG_HEARTBEAT,
    MessageType.STM_MSG_STATUS,
    MessageType.STM_MSG_EVENT,
    MessageType.STM_MSG_TELEMETRY_CHUNK,
    MessageType.STM_MSG_ALERT_REQUEST,
    MessageType.STM_MSG_SCAN_START,
    MessageType.STM_MSG_SCAN_STOP,
    MessageType.STM_MSG_ACK,
    MessageType.STM_MSG_NACK,
}

SCAN_RESULT_CODES = {
    "clear": 0,
    "occupied": 1,
    "fault": 2,
    "stopped": 3,
}
SCAN_RESULT_NAMES = {value: key.upper() for key, value in SCAN_RESULT_CODES.items()}

SCAN_STOP_REASON_STANDBY = 0
SCAN_STOP_REASON_RESOLVED = 1
SCAN_STOP_REASON_SHUTDOWN = 2
SCAN_STOP_REASON_NAMES = {
    SCAN_STOP_REASON_STANDBY: "STANDBY",
    SCAN_STOP_REASON_RESOLVED: "OCCUPANCY_RESOLVED",
    SCAN_STOP_REASON_SHUTDOWN: "SHUTDOWN",
}

SYSTEM_STATE_NAMES = {
    0: "SELF_TEST",
    1: "STANDBY",
    2: "ARM_DELAY",
    3: "ACTIVE",
    4: "FAULT",
    5: "WAIT_REAR_CONFIRM",
}


@dataclasses.dataclass(frozen=True)
class Frame:
    msg_type: int
    flags: int
    sequence: int
    payload: bytes


@dataclasses.dataclass
class OutboundRequest:
    msg_type: int
    payload: bytes
    ack_required: bool
    sequence: Optional[int] = None


@dataclasses.dataclass
class PendingTransmission:
    request: OutboundRequest
    retries: int
    deadline: float


class TelemetryAssembler:
    """Reassembles one ordered, ACK-protected STM32 telemetry message."""

    HEADER_SIZE = 8

    def __init__(self) -> None:
        self.message_id: Optional[int] = None
        self.total_length = 0
        self.data = bytearray()
        self.error_count = 0

    def reset(self) -> None:
        self.message_id = None
        self.total_length = 0
        self.data.clear()

    def feed(self, payload: bytes) -> Optional[Dict[str, object]]:
        if len(payload) < self.HEADER_SIZE:
            self.error_count += 1
            return None
        message_id, total_length, offset = struct.unpack_from("<IHH", payload)
        chunk = payload[self.HEADER_SIZE:]
        if not 0 < total_length <= MAX_TELEMETRY_JSON:
            self.error_count += 1
            self.reset()
            return None
        if offset + len(chunk) > total_length:
            self.error_count += 1
            self.reset()
            return None

        if self.message_id != message_id:
            self.reset()
            if offset != 0:
                self.error_count += 1
                return None
            self.message_id = message_id
            self.total_length = total_length

        if total_length != self.total_length or offset != len(self.data):
            self.error_count += 1
            self.reset()
            return None

        self.data.extend(chunk)
        if len(self.data) != self.total_length:
            return None

        raw = bytes(self.data)
        self.reset()
        try:
            decoded = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self.error_count += 1
            return None
        if not isinstance(decoded, dict):
            self.error_count += 1
            return None
        return decoded


class TelemetryStore:
    """Small SQLite FIFO so 4G outages never discard telemetry."""

    def __init__(self, path: str, max_rows: int = MAX_OFFLINE_ROWS) -> None:
        self.path = path
        self.max_rows = max_rows
        self.lock = threading.Lock()
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, mode=0o750, exist_ok=True)
        self.db = sqlite3.connect(path, check_same_thread=False)
        with self.db:
            self.db.execute("PRAGMA journal_mode=WAL")
            self.db.execute(
                "CREATE TABLE IF NOT EXISTS telemetry ("
                "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                "created_ms INTEGER NOT NULL, payload TEXT NOT NULL, "
                "priority INTEGER NOT NULL DEFAULT 0)"
            )
            columns = {
                str(row[1])
                for row in self.db.execute(
                    "PRAGMA table_info(telemetry)"
                ).fetchall()
            }
            if "priority" not in columns:
                self.db.execute(
                    "ALTER TABLE telemetry ADD COLUMN "
                    "priority INTEGER NOT NULL DEFAULT 0"
                )

    def enqueue(self, payload: Dict[str, object], priority: int = 0) -> int:
        encoded = json.dumps(payload, separators=(",", ":"),
                             ensure_ascii=True)
        created_ms = int(time.time() * 1000.0)
        with self.lock, self.db:
            self.db.execute(
                "INSERT INTO telemetry(created_ms,payload,priority) "
                "VALUES(?,?,?)",
                (created_ms, encoded, int(priority)),
            )
            self.db.execute(
                "DELETE FROM telemetry WHERE id IN ("
                "SELECT id FROM telemetry "
                "ORDER BY priority ASC,id ASC LIMIT "
                "MAX(0,(SELECT COUNT(*) FROM telemetry)-?))",
                (self.max_rows,),
            )
            row = self.db.execute(
                "SELECT COUNT(*) FROM telemetry"
            ).fetchone()
        return int(row[0]) if row else 0

    def peek(self) -> Optional[Tuple[int, str]]:
        with self.lock:
            row = self.db.execute(
                "SELECT id,payload FROM telemetry "
                "ORDER BY priority DESC,id ASC LIMIT 1"
            ).fetchone()
        return (int(row[0]), str(row[1])) if row else None

    def remove(self, row_id: int) -> None:
        with self.lock, self.db:
            self.db.execute("DELETE FROM telemetry WHERE id=?", (row_id,))

    def count(self) -> int:
        with self.lock:
            row = self.db.execute("SELECT COUNT(*) FROM telemetry").fetchone()
        return int(row[0]) if row else 0

    def close(self) -> None:
        with self.lock:
            self.db.close()


class ThingsBoardPublisher(threading.Thread):
    """Publishes the SQLite FIFO through the Pi's current network route."""

    def __init__(self, store: TelemetryStore, host: str, token: str,
                 interface: str,
                 stop_event: threading.Event) -> None:
        super().__init__(name="tripguard-thingsboard", daemon=True)
        self.store = store
        self.host = host.rstrip("/")
        self.token = token.strip()
        self.interface = interface.strip()
        self.stop_event = stop_event
        self.sent_count = 0
        self.error_count = 0

    def run(self) -> None:
        if (not self.token) or self.token.startswith("PASTE_"):
            LOG.warning(
                "ThingsBoard token is empty; telemetry remains in %s",
                self.store.path,
            )
            return

        endpoint = "%s/api/v1/%s/telemetry" % (
            self.host,
            urllib.parse.quote(self.token, safe=""),
        )
        hostname = urllib.parse.urlparse(self.host).hostname
        if not hostname:
            LOG.error("Invalid ThingsBoard host: %s", self.host)
            return
        retry_s = 1.0
        while not self.stop_event.is_set():
            row = self.store.peek()
            if row is None:
                self.stop_event.wait(0.5)
                continue
            row_id, payload = row
            request = urllib.request.Request(
                endpoint,
                data=payload.encode("utf-8"),
                headers={
                    "Content-Type": "application/json",
                    # Cloudflare Browser Integrity rejects Python's default
                    # urllib signature with error 1010.
                    "User-Agent": "curl/8.14.1",
                },
                method="POST",
            )
            try:
                self._pin_host_routes(hostname)
                with urllib.request.urlopen(request, timeout=20.0) as response:
                    status = int(getattr(response, "status", 0))
                if not 200 <= status < 300:
                    raise RuntimeError("HTTP status %d" % status)
                self.store.remove(row_id)
                self.sent_count += 1
                retry_s = 1.0
                LOG.info("ThingsBoard telemetry sent; queued=%d",
                         self.store.count())
            except (OSError, RuntimeError, urllib.error.URLError,
                    subprocess.SubprocessError) as exc:
                self.error_count += 1
                LOG.warning("ThingsBoard unavailable (%s); retry in %.0fs",
                            exc, retry_s)
                self.stop_event.wait(retry_s)
                retry_s = min(retry_s * 2.0, 60.0)

    def _pin_host_routes(self, hostname: str) -> None:
        """Force ThingsBoard IPv4 traffic over ppp0 even while Wi-Fi is up."""
        if not self.interface:
            return
        addresses = {
            item[4][0]
            for item in socket.getaddrinfo(
                hostname, 443, family=socket.AF_INET,
                type=socket.SOCK_STREAM,
            )
        }
        if not addresses:
            raise OSError("ThingsBoard DNS returned no IPv4 address")
        for address in addresses:
            result = subprocess.run(
                ["/sbin/ip", "route", "replace", address,
                 "dev", self.interface],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
                timeout=5.0,
            )
            if result.returncode != 0:
                raise OSError(
                    "cannot route %s through %s" %
                    (address, self.interface)
                )


class GatewayTelemetry:
    """Merges STM32 state with the latest Pi camera state before storage."""

    def __init__(self, store: TelemetryStore,
                 camera_state_path: str = DEFAULT_CAMERA_STATE) -> None:
        self.store = store
        self.camera_state_path = camera_state_path
        self.lock = threading.Lock()
        saved_person, saved_confidence = self._load_camera_state()
        self.camera_person = saved_person
        self.camera_confidence = saved_confidence
        self.camera_event_sequence = 0

    def set_camera(self, person: bool, confidence: int) -> None:
        with self.lock:
            self.camera_person = person
            self.camera_confidence = confidence if person else 0
            self._save_camera_state()

    def is_camera_person_latched(self) -> bool:
        with self.lock:
            return self.camera_person

    def get_camera_state(self) -> Tuple[bool, int]:
        with self.lock:
            return self.camera_person, self.camera_confidence

    def _load_camera_state(self) -> Tuple[bool, int]:
        try:
            with open(self.camera_state_path, "r", encoding="utf-8") as source:
                value = json.load(source)
            person = bool(value.get("person", False))
            confidence = int(value.get("confidence_permille", 0))
            return person, max(0, min(1000, confidence if person else 0))
        except (FileNotFoundError, OSError, TypeError, ValueError):
            return False, 0

    def _save_camera_state(self) -> None:
        parent = os.path.dirname(self.camera_state_path)
        os.makedirs(parent, mode=0o750, exist_ok=True)
        temporary = self.camera_state_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as output:
            json.dump({
                "person": self.camera_person,
                "confidence_permille": self.camera_confidence,
                "updated_ms": int(time.time() * 1000.0),
            }, output, separators=(",", ":"))
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, 0o640)
        os.replace(temporary, self.camera_state_path)

    def accept_stm32(self, telemetry: Dict[str, object]) -> None:
        with self.lock:
            telemetry["piCameraPerson"] = self.camera_person
            telemetry["piCameraConfidencePermille"] = self.camera_confidence
        telemetry["gatewayTimestampMs"] = int(time.time() * 1000.0)
        queued = self.store.enqueue(telemetry)
        LOG.info("STM32 telemetry stored; queued=%d", queued)

    def accept_alert(self, alert_kind: int, state: int, value: int) -> None:
        payload: Dict[str, object] = {
            "event": "STM32_ALERT_REQUEST",
            "alertKind": alert_kind,
            "systemState": SYSTEM_STATE_NAMES.get(state, "UNKNOWN"),
            "alertValue": value,
            "gatewayTimestampMs": int(time.time() * 1000.0),
        }
        with self.lock:
            payload["piCameraPerson"] = self.camera_person
            payload["piCameraConfidencePermille"] = self.camera_confidence
        queued = self.store.enqueue(payload, priority=ALERT_PRIORITY)
        LOG.warning("STM32 alert stored; kind=%d queued=%d", alert_kind, queued)

    def accept_camera_evidence(self, image_path: str,
                               confidence: int) -> int:
        """Store one bounded JPEG as high-priority ThingsBoard telemetry."""
        if not 0 <= confidence <= 1000:
            raise ValueError("confidence must be in range 0..1000")

        resolved_path = os.path.realpath(image_path)
        file_stat = os.stat(resolved_path, follow_symlinks=False)
        if not stat.S_ISREG(file_stat.st_mode):
            raise ValueError("evidence path must be a regular file")
        if not 4 <= file_stat.st_size <= MAX_EVIDENCE_BYTES:
            raise ValueError(
                "evidence JPEG must be 4..%d bytes" % MAX_EVIDENCE_BYTES
            )

        open_flags = os.O_RDONLY
        if hasattr(os, "O_NOFOLLOW"):
            open_flags |= os.O_NOFOLLOW
        descriptor = os.open(resolved_path, open_flags)
        with os.fdopen(descriptor, "rb") as image_file:
            image_data = image_file.read(MAX_EVIDENCE_BYTES + 1)
        if len(image_data) > MAX_EVIDENCE_BYTES:
            raise ValueError("evidence JPEG exceeds size limit")
        if not image_data.startswith(b"\xff\xd8") or not image_data.endswith(
            b"\xff\xd9"
        ):
            raise ValueError("evidence file is not a complete JPEG")

        captured_ms = int(time.time() * 1000.0)
        with self.lock:
            self.camera_event_sequence = (
                self.camera_event_sequence + 1
            ) & 0xFFFFFFFF
            event_sequence = self.camera_event_sequence

        payload: Dict[str, object] = {
            "ts": captured_ms,
            "values": {
                "eventType": "PERSON_DETECTED",
                "eventId": uuid.uuid4().hex,
                "eventSeq": event_sequence,
                "personDetected": True,
                "confidence": round(confidence / 1000.0, 3),
                "piCameraConfidencePermille": confidence,
                "evidenceContentType": "image/jpeg",
                "evidenceBytes": len(image_data),
                "evidenceImage": (
                    "data:image/jpeg;base64,"
                    + base64.b64encode(image_data).decode("ascii")
                ),
            },
        }
        queued = self.store.enqueue(payload, priority=EVIDENCE_PRIORITY)
        LOG.warning(
            "Camera evidence stored; eventSeq=%d bytes=%d queued=%d",
            event_sequence,
            len(image_data),
            queued,
        )
        return queued

    def accept_scan_event(self, session_id: int, state: str,
                          duration_seconds: int = 0,
                          result: str = "") -> None:
        payload: Dict[str, object] = {
            "event": "PI_CAMERA_SCAN",
            "scanSessionId": int(session_id),
            "scanState": state,
            "scanDurationSeconds": int(duration_seconds),
            "gatewayTimestampMs": int(time.time() * 1000.0),
        }
        if result:
            payload["scanResult"] = result
        self.store.enqueue(payload, priority=ALERT_PRIORITY)


class ScanController:
    """Idempotently starts/stops the systemd-managed CSI scan process."""

    def __init__(self, service: str = DEFAULT_CAMERA_SERVICE,
                 environment_path: str = DEFAULT_CAMERA_ENV,
                 state_path: str = DEFAULT_SCAN_STATE,
                 telemetry: Optional[GatewayTelemetry] = None) -> None:
        self.service = service
        self.environment_path = environment_path
        self.state_path = state_path
        self.telemetry = telemetry
        self.lock = threading.Lock()

    def start(self, session_id: int, duration_seconds: int) -> bool:
        if not 1 <= duration_seconds <= 3600:
            LOG.error("Rejected scan duration: %d", duration_seconds)
            return False
        with self.lock:
            if (
                self.telemetry is not None
                and self.telemetry.is_camera_person_latched()
            ):
                LOG.error(
                    "Refusing SCAN_START session=%d: OCCUPIED is latched",
                    session_id,
                )
                return False
            previous = self._read_state()
            if (
                previous.get("active") is True
                and int(previous.get("session_id", -1)) == session_id
                and self._service_is_active()
            ):
                LOG.info("Scan session %d already active", session_id)
                return True

            try:
                self._write_environment(session_id, duration_seconds)
                completed = subprocess.run(
                    ("/usr/bin/systemctl", "restart", self.service),
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=20.0,
                    check=False,
                )
            except (OSError, subprocess.SubprocessError) as exc:
                LOG.error("Cannot start camera scan: %s", exc)
                return False
            if completed.returncode != 0:
                LOG.error(
                    "Camera service start failed rc=%d: %s",
                    completed.returncode,
                    completed.stdout.strip(),
                )
                return False

            self._write_state({
                "active": True,
                "session_id": session_id,
                "duration_seconds": duration_seconds,
                "updated_ms": int(time.time() * 1000.0),
            })
            if self.telemetry is not None:
                self.telemetry.accept_scan_event(
                    session_id, "STARTED", duration_seconds
                )
            LOG.warning(
                "Camera scan started session=%d duration=%ds",
                session_id, duration_seconds,
            )
            return True

    def stop(self, session_id: int, reason: int) -> bool:
        if reason not in SCAN_STOP_REASON_NAMES:
            LOG.error("Rejected scan stop reason: %d", reason)
            return False
        with self.lock:
            previous = self._read_state()
            current_session = int(previous.get("session_id", 0))
            if session_id not in (0, current_session):
                LOG.info(
                    "Ignoring stale scan stop session=%d current=%d",
                    session_id, current_session,
                )
                return True
            try:
                completed = subprocess.run(
                    ("/usr/bin/systemctl", "stop", self.service),
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=20.0,
                    check=False,
                )
            except (OSError, subprocess.SubprocessError) as exc:
                LOG.error("Cannot stop camera scan: %s", exc)
                return False
            if completed.returncode != 0:
                LOG.error(
                    "Camera service stop failed rc=%d: %s",
                    completed.returncode,
                    completed.stdout.strip(),
                )
                return False

            self._write_state({
                "active": False,
                "session_id": current_session,
                "stop_reason": int(reason),
                "updated_ms": int(time.time() * 1000.0),
            })
            if self.telemetry is not None:
                self.telemetry.accept_scan_event(
                    current_session,
                    "STOPPED",
                    result=SCAN_STOP_REASON_NAMES.get(
                        reason, "UNKNOWN_%d" % reason
                    ),
                )
            LOG.warning(
                "Camera scan stopped session=%d reason=%d",
                current_session, reason,
            )
            return True

    def complete(self, session_id: int, result: str) -> None:
        with self.lock:
            previous = self._read_state()
            duration = int(previous.get("duration_seconds", 0))
            self._write_state({
                "active": False,
                "session_id": session_id,
                "duration_seconds": duration,
                "result": result,
                "updated_ms": int(time.time() * 1000.0),
            })
            if self.telemetry is not None:
                self.telemetry.accept_scan_event(
                    session_id, "COMPLETE", duration, result
                )

    def _service_is_active(self) -> bool:
        completed = subprocess.run(
            ("/usr/bin/systemctl", "is-active", "--quiet", self.service),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5.0,
            check=False,
        )
        return completed.returncode == 0

    def _write_environment(self, session_id: int,
                           duration_seconds: int) -> None:
        parent = os.path.dirname(self.environment_path)
        os.makedirs(parent, mode=0o770, exist_ok=True)
        temporary = self.environment_path + ".tmp"
        with open(temporary, "w", encoding="ascii") as output:
            output.write("TRIPGUARD_SCAN_SESSION_ID=%d\n" % session_id)
            output.write(
                "TRIPGUARD_SCAN_SESSION_SECONDS=%d\n" % duration_seconds
            )
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, 0o640)
        os.replace(temporary, self.environment_path)

    def _read_state(self) -> Dict[str, object]:
        try:
            with open(self.state_path, "r", encoding="utf-8") as source:
                value = json.load(source)
            return value if isinstance(value, dict) else {}
        except (FileNotFoundError, OSError, ValueError):
            return {}

    def _write_state(self, value: Dict[str, object]) -> None:
        parent = os.path.dirname(self.state_path)
        os.makedirs(parent, mode=0o750, exist_ok=True)
        temporary = self.state_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as output:
            json.dump(value, output, separators=(",", ":"))
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, 0o640)
        os.replace(temporary, self.state_path)


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: init FFFF, poly 1021, refin/refout false."""

    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_type: int, flags: int, sequence: int,
                 payload: bytes = b"") -> bytes:
    if not 0 <= int(msg_type) <= 0xFF:
        raise ValueError("message type must fit in one byte")
    if flags & ~KNOWN_FLAGS:
        raise ValueError("unsupported frame flags")
    if not 0 <= sequence <= 0xFFFF:
        raise ValueError("sequence must be uint16")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds 128 bytes")

    header = MAGIC + struct.pack(
        "<BBBHH",
        PROTOCOL_VERSION,
        int(msg_type),
        flags,
        sequence,
        len(payload),
    )
    crc = crc16_ccitt_false(header[2:] + payload)
    return header + payload + struct.pack("<H", crc)


class FrameParser:
    """Bounded streaming parser that tolerates fragmented and joined frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_error_count = 0
        self.length_error_count = 0
        self.version_error_count = 0
        self.overflow_count = 0
        self.discarded_byte_count = 0

    def reset_partial_frame(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes) -> List[Frame]:
        if data:
            self._buffer.extend(data)

        if len(self._buffer) > PARSER_BUFFER_LIMIT:
            excess = len(self._buffer) - PARSER_BUFFER_LIMIT
            del self._buffer[:excess]
            self.overflow_count += 1
            self.discarded_byte_count += excess

        frames: List[Frame] = []
        while True:
            if len(self._buffer) < len(MAGIC):
                break

            magic_index = self._buffer.find(MAGIC)
            if magic_index < 0:
                keep = 1 if self._buffer[-1] == MAGIC[0] else 0
                discard = len(self._buffer) - keep
                if discard > 0:
                    del self._buffer[:discard]
                    self.discarded_byte_count += discard
                break

            if magic_index > 0:
                del self._buffer[:magic_index]
                self.discarded_byte_count += magic_index

            if len(self._buffer) < FIXED_HEADER_SIZE:
                break

            if self._buffer[2] != PROTOCOL_VERSION:
                del self._buffer[0]
                self.version_error_count += 1
                self.discarded_byte_count += 1
                continue

            payload_length = self._buffer[7] | (self._buffer[8] << 8)
            if payload_length > MAX_PAYLOAD:
                del self._buffer[0]
                self.length_error_count += 1
                self.discarded_byte_count += 1
                continue

            frame_size = FIXED_HEADER_SIZE + payload_length + CRC_SIZE
            if len(self._buffer) < frame_size:
                break

            crc_offset = FIXED_HEADER_SIZE + payload_length
            received_crc = (
                self._buffer[crc_offset]
                | (self._buffer[crc_offset + 1] << 8)
            )
            calculated_crc = crc16_ccitt_false(
                bytes(self._buffer[2:crc_offset])
            )
            if received_crc != calculated_crc:
                del self._buffer[0]
                self.crc_error_count += 1
                self.discarded_byte_count += 1
                continue

            frames.append(
                Frame(
                    msg_type=self._buffer[3],
                    flags=self._buffer[4],
                    sequence=self._buffer[5] | (self._buffer[6] << 8),
                    payload=bytes(
                        self._buffer[FIXED_HEADER_SIZE:crc_offset]
                    ),
                )
            )
            del self._buffer[:frame_size]

        return frames


class DuplicateWindow:
    """Recent (message type, sequence) keys; safe at uint16 wrap-around."""

    def __init__(self, capacity: int = DUPLICATE_WINDOW_SIZE) -> None:
        self._capacity = capacity
        self._order: Deque[Tuple[int, int]] = collections.deque()
        self._keys: Set[Tuple[int, int]] = set()

    def is_duplicate_and_remember(self, msg_type: int,
                                  sequence: int) -> bool:
        key = (msg_type, sequence)
        if key in self._keys:
            return True
        self.remember(msg_type, sequence)
        return False

    def is_duplicate(self, msg_type: int, sequence: int) -> bool:
        return (msg_type, sequence) in self._keys

    def remember(self, msg_type: int, sequence: int) -> None:
        key = (msg_type, sequence)
        if key in self._keys:
            return
        self._keys.add(key)
        self._order.append(key)
        while len(self._order) > self._capacity:
            self._keys.discard(self._order.popleft())


class TripGuardLink:
    """Owns pyserial, parser, retry state and all UART transmission."""

    def __init__(self, port: str, baud: int,
                 stop_event: threading.Event,
                 telemetry: Optional[GatewayTelemetry] = None,
                 scan_controller: Optional[ScanController] = None) -> None:
        self.port = port
        self.baud = baud
        self.stop_event = stop_event
        self.telemetry = telemetry
        self.scan_controller = scan_controller
        self.telemetry_assembler = TelemetryAssembler()
        self.parser = FrameParser()
        self.tx_queue: queue.Queue[OutboundRequest] = queue.Queue(
            maxsize=TX_QUEUE_DEPTH
        )
        self.pending: Dict[Tuple[int, int], PendingTransmission] = {}
        self.duplicate_window = DuplicateWindow()
        self.serial_port = None
        self.deferred_request: Optional[OutboundRequest] = None
        self.next_sequence = 0
        self.ready_latched = threading.Event()
        self.persistent_state: Dict[str, Tuple[int, bytes]] = {}
        self.persistent_resync_due: Dict[str, float] = {}
        self.last_rx_time: Optional[float] = None
        self.stm_online = False

        self.rx_frame_count = 0
        self.tx_frame_count = 0
        self.crc_error_count = 0
        self.uart_error_count = 0
        self.duplicate_count = 0
        self.tx_drop_count = 0
        self.retry_exhausted_count = 0

    def set_camera_ready(self) -> bool:
        self.ready_latched.set()
        return self._enqueue_persistent_state(
            "ready",
            MessageType.PI_MSG_READY,
            struct.pack("<B", 1),
        )

    def queue_camera_person(self, confidence_permille: int) -> bool:
        if not 0 <= confidence_permille <= 1000:
            raise ValueError("confidence must be in range 0..1000")
        if self.telemetry is not None:
            self.telemetry.set_camera(True, confidence_permille)
        return self._enqueue_persistent_state(
            "camera",
            MessageType.PI_MSG_CAMERA_PERSON,
            struct.pack("<H", confidence_permille),
        )

    def queue_camera_clear(self) -> bool:
        if self.telemetry is not None:
            self.telemetry.set_camera(False, 0)
        return self._enqueue_persistent_state(
            "camera",
            MessageType.PI_MSG_CAMERA_CLEAR,
            b"",
        )

    def queue_scan_complete(self, session_id: int, result_code: int) -> bool:
        if not 0 <= session_id <= 0xFFFFFFFF:
            raise ValueError("session id must be uint32")
        if result_code not in SCAN_RESULT_NAMES:
            raise ValueError("invalid scan result")
        result_name = SCAN_RESULT_NAMES[result_code]
        if self.scan_controller is not None:
            self.scan_controller.complete(session_id, result_name)
        return self._enqueue_persistent_state(
            "scan_complete",
            MessageType.PI_MSG_SCAN_COMPLETE,
            struct.pack("<IB", session_id, result_code),
        )

    def queue_status_request(self) -> bool:
        return self._enqueue(
            MessageType.PI_MSG_STATUS_REQUEST,
            b"",
            ack_required=True,
        )

    def _enqueue(self, msg_type: MessageType, payload: bytes,
                 ack_required: bool) -> bool:
        request = OutboundRequest(
            msg_type=int(msg_type),
            payload=bytes(payload),
            ack_required=ack_required,
        )
        try:
            self.tx_queue.put_nowait(request)
            return True
        except queue.Full:
            self.tx_drop_count += 1
            LOG.error("TX queue full; dropped %s", msg_type.name)
            return False

    def _enqueue_persistent_state(self, state_key: str,
                                  msg_type: MessageType,
                                  payload: bytes) -> bool:
        """Queue the newest durable state and retain it until STM32 ACKs."""
        desired = (int(msg_type), bytes(payload))
        self.persistent_state[state_key] = desired
        queued = self._enqueue(msg_type, payload, ack_required=True)
        if queued:
            self.persistent_resync_due.pop(state_key, None)
        else:
            self.persistent_resync_due[state_key] = (
                time.monotonic() + PERSISTENT_STATE_RETRY_S
            )
        return queued

    @staticmethod
    def _persistent_state_key(request: OutboundRequest) -> Optional[str]:
        if request.msg_type == int(MessageType.PI_MSG_READY):
            return "ready"
        if request.msg_type in (
            int(MessageType.PI_MSG_CAMERA_PERSON),
            int(MessageType.PI_MSG_CAMERA_CLEAR),
        ):
            return "camera"
        if request.msg_type == int(MessageType.PI_MSG_SCAN_COMPLETE):
            return "scan_complete"
        return None

    def _request_is_current_persistent_state(
            self, state_key: str, request: OutboundRequest) -> bool:
        return self.persistent_state.get(state_key) == (
            request.msg_type,
            request.payload,
        )

    def _resync_persistent_state(self, now: float) -> None:
        """Requeue durable state after a temporary STM32 outage."""
        for state_key, due in list(self.persistent_resync_due.items()):
            if now < due:
                continue
            desired = self.persistent_state.get(state_key)
            if desired is None:
                self.persistent_resync_due.pop(state_key, None)
                continue
            msg_type, payload = desired
            if self._enqueue(
                    MessageType(msg_type), payload, ack_required=True):
                self.persistent_resync_due.pop(state_key, None)
                LOG.info("Requeued persistent STM32 state: %s", state_key)
            else:
                self.persistent_resync_due[state_key] = (
                    now + PERSISTENT_STATE_RETRY_S
                )

    def run(self) -> None:
        if serial is None:
            raise RuntimeError(
                "pyserial is not installed; run: sudo apt install python3-serial"
            )

        next_heartbeat = time.monotonic()
        while not self.stop_event.is_set():
            if self.serial_port is None:
                if not self._open_serial():
                    self.stop_event.wait(RECONNECT_DELAY_S)
                    continue
                next_heartbeat = time.monotonic()

            try:
                now = time.monotonic()
                if now >= next_heartbeat:
                    self._send_heartbeat()
                    next_heartbeat = now + HEARTBEAT_INTERVAL_S

                self._retry_expired(now)
                self._resync_persistent_state(now)
                self._send_next_queued()
                self._receive_once()
                self._update_peer_online(time.monotonic())
                self.crc_error_count = self.parser.crc_error_count
            except (OSError, serial.SerialException) as exc:
                self.uart_error_count += 1
                LOG.warning("UART error: %s", exc)
                self._close_serial()
                self.stop_event.wait(RECONNECT_DELAY_S)

        self._close_serial()

    def _open_serial(self) -> bool:
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=SERIAL_READ_TIMEOUT_S,
                write_timeout=SERIAL_WRITE_TIMEOUT_S,
                exclusive=True,
            )
            self.serial_port.reset_input_buffer()
            self.parser.reset_partial_frame()
            now = time.monotonic()
            for pending in self.pending.values():
                pending.deadline = now
            LOG.info("UART opened: %s at %d 8N1", self.port, self.baud)

            for state_key in self.persistent_state:
                self.persistent_resync_due[state_key] = now
            return True
        except (OSError, serial.SerialException) as exc:
            self.serial_port = None
            self.uart_error_count += 1
            LOG.warning("Cannot open %s: %s", self.port, exc)
            return False

    def _close_serial(self) -> None:
        port = self.serial_port
        self.serial_port = None
        self.stm_online = False
        self.last_rx_time = None
        self.parser.reset_partial_frame()
        if port is not None:
            try:
                port.close()
            except (OSError, serial.SerialException):
                pass

    def _allocate_sequence(self) -> int:
        for _ in range(0x10000):
            sequence = self.next_sequence
            self.next_sequence = (self.next_sequence + 1) & 0xFFFF
            if all(key[0] != sequence for key in self.pending):
                return sequence
        raise RuntimeError("all UART sequence values are pending")

    def _write_frame(self, frame: bytes) -> None:
        if self.serial_port is None:
            raise OSError("serial port is closed")
        written = self.serial_port.write(frame)
        if written != len(frame):
            raise OSError("short UART write")
        self.tx_frame_count += 1

    def _send_heartbeat(self) -> None:
        sequence = self._allocate_sequence()
        uptime_ms = int(time.monotonic() * 1000.0) & 0xFFFFFFFF
        frame = encode_frame(
            MessageType.PI_MSG_HEARTBEAT,
            0,
            sequence,
            struct.pack("<I", uptime_ms),
        )
        self._write_frame(frame)

    def _send_next_queued(self) -> None:
        request = self.deferred_request
        if request is None:
            try:
                request = self.tx_queue.get_nowait()
            except queue.Empty:
                return

        self.deferred_request = request
        if request.sequence is None:
            request.sequence = self._allocate_sequence()
        flags = FLAG_ACK_REQUIRED if request.ack_required else 0
        frame = encode_frame(
            request.msg_type,
            flags,
            request.sequence,
            request.payload,
        )
        self._write_frame(frame)

        if request.ack_required:
            key = (request.sequence, request.msg_type)
            self.pending[key] = PendingTransmission(
                request=request,
                retries=0,
                deadline=time.monotonic() + ACK_TIMEOUT_S,
            )

        self.deferred_request = None
        self.tx_queue.task_done()

    def _retry_expired(self, now: float) -> None:
        for key, pending in list(self.pending.items()):
            if now < pending.deadline:
                continue
            if pending.retries >= MAX_RETRIES:
                self.pending.pop(key, None)
                self.retry_exhausted_count += 1
                state_key = self._persistent_state_key(pending.request)
                if (
                    state_key is not None
                    and self._request_is_current_persistent_state(
                        state_key, pending.request
                    )
                ):
                    self.persistent_resync_due[state_key] = (
                        now + PERSISTENT_STATE_RETRY_S
                    )
                LOG.error(
                    "ACK timeout type=0x%02X seq=%d after %d retries",
                    pending.request.msg_type,
                    pending.request.sequence,
                    MAX_RETRIES,
                )
                continue

            retry_frame = encode_frame(
                pending.request.msg_type,
                FLAG_ACK_REQUIRED | FLAG_RETRY,
                int(pending.request.sequence),
                pending.request.payload,
            )
            self._write_frame(retry_frame)
            pending.retries += 1
            pending.deadline = time.monotonic() + ACK_TIMEOUT_S
            LOG.warning(
                "UART retry %d/%d type=0x%02X seq=%d",
                pending.retries,
                MAX_RETRIES,
                pending.request.msg_type,
                pending.request.sequence,
            )

    def _receive_once(self) -> None:
        if self.serial_port is None:
            return
        available = self.serial_port.in_waiting
        data = self.serial_port.read(available if available > 0 else 1)
        if not data:
            return

        for frame in self.parser.feed(data):
            self._handle_received_frame(frame)

    def _handle_received_frame(self, frame: Frame) -> None:
        self.rx_frame_count += 1
        self.last_rx_time = time.monotonic()
        self.stm_online = True

        try:
            msg_type = MessageType(frame.msg_type)
        except ValueError:
            LOG.warning(
                "Unknown STM32 message type=0x%02X seq=%d",
                frame.msg_type,
                frame.sequence,
            )
            return

        if msg_type not in STM_MESSAGE_TYPES:
            LOG.warning(
                "Unexpected Pi-direction message type=%s seq=%d",
                msg_type.name,
                frame.sequence,
            )
            return

        if msg_type in (MessageType.STM_MSG_ACK, MessageType.STM_MSG_NACK):
            self._handle_ack(frame, msg_type)
            return

        if not self._payload_is_valid(msg_type, frame.payload):
            LOG.warning(
                "Invalid payload length=%d for %s seq=%d",
                len(frame.payload),
                msg_type.name,
                frame.sequence,
            )
            return

        duplicate = False
        if frame.flags & FLAG_ACK_REQUIRED:
            duplicate = self.duplicate_window.is_duplicate(
                frame.msg_type, frame.sequence
            )
            if duplicate:
                self._send_pi_ack(frame)
                self.duplicate_count += 1
                LOG.info(
                    "Duplicate %s seq=%d ACKed without reprocessing",
                    msg_type.name,
                    frame.sequence,
                )
                return

        if msg_type == MessageType.STM_MSG_HEARTBEAT:
            uptime_ms = struct.unpack_from("<I", frame.payload)[0]
            LOG.debug("STM32 heartbeat uptime=%d ms", uptime_ms)
        elif msg_type == MessageType.STM_MSG_STATUS:
            state, rear_confirmed, arm_remaining = struct.unpack_from(
                "<BBH", frame.payload
            )
            LOG.info(
                "STM32 status state=%s rearConfirmed=%d armRemaining=%d s",
                SYSTEM_STATE_NAMES.get(state, "UNKNOWN(%d)" % state),
                rear_confirmed,
                arm_remaining,
            )
        elif msg_type == MessageType.STM_MSG_EVENT:
            event_code, state = struct.unpack_from("<BB", frame.payload)
            LOG.info(
                "STM32 event code=%d state=%s data=%s",
                event_code,
                SYSTEM_STATE_NAMES.get(state, "UNKNOWN(%d)" % state),
                frame.payload[2:].hex(),
            )
        elif msg_type == MessageType.STM_MSG_TELEMETRY_CHUNK:
            telemetry = self.telemetry_assembler.feed(frame.payload)
            if telemetry is not None:
                if self.telemetry is not None:
                    self.telemetry.accept_stm32(telemetry)
                else:
                    LOG.info("Complete STM32 telemetry received")
        elif msg_type == MessageType.STM_MSG_ALERT_REQUEST:
            alert_kind, state, value = struct.unpack_from(
                "<BBH", frame.payload
            )
            if self.telemetry is not None:
                self.telemetry.accept_alert(alert_kind, state, value)
            else:
                LOG.warning(
                    "STM32 alert kind=%d state=%d value=%d",
                    alert_kind, state, value,
                )
        elif msg_type == MessageType.STM_MSG_SCAN_START:
            session_id, duration_seconds = struct.unpack(
                "<IH", frame.payload
            )
            if (
                self.scan_controller is None
                or not self.scan_controller.start(
                    session_id, duration_seconds
                )
            ):
                LOG.error(
                    "SCAN_START not applied session=%d duration=%d",
                    session_id, duration_seconds,
                )
                return
        elif msg_type == MessageType.STM_MSG_SCAN_STOP:
            session_id, reason = struct.unpack("<IB", frame.payload)
            if (
                self.scan_controller is None
                or not self.scan_controller.stop(session_id, reason)
            ):
                LOG.error(
                    "SCAN_STOP not applied session=%d reason=%d",
                    session_id, reason,
                )
                return
            if reason == SCAN_STOP_REASON_RESOLVED:
                # Driver acknowledgement is the only automatic way to clear
                # the durable OCCUPIED latch.  Keep the new CLEAR state in the
                # retry/resync set until STM32 acknowledges it.
                self.queue_camera_clear()

        # Commit-before-ACK: telemetry/alerts are acknowledged only after the
        # complete local processing above has succeeded.
        if frame.flags & FLAG_ACK_REQUIRED:
            self.duplicate_window.remember(frame.msg_type, frame.sequence)
            self._send_pi_ack(frame)

    @staticmethod
    def _payload_is_valid(msg_type: MessageType, payload: bytes) -> bool:
        if msg_type == MessageType.STM_MSG_HEARTBEAT:
            return len(payload) == 4
        if msg_type == MessageType.STM_MSG_STATUS:
            return len(payload) >= 4
        if msg_type == MessageType.STM_MSG_EVENT:
            return len(payload) >= 2
        if msg_type == MessageType.STM_MSG_TELEMETRY_CHUNK:
            return 8 < len(payload) <= MAX_PAYLOAD
        if msg_type == MessageType.STM_MSG_ALERT_REQUEST:
            return len(payload) == 4
        if msg_type == MessageType.STM_MSG_SCAN_START:
            return len(payload) == 6 and struct.unpack_from(
                "<H", payload, 4
            )[0] > 0
        if msg_type == MessageType.STM_MSG_SCAN_STOP:
            return (
                len(payload) == 5
                and payload[4] in SCAN_STOP_REASON_NAMES
            )
        return True

    def _send_pi_ack(self, received: Frame) -> None:
        ack = encode_frame(
            MessageType.PI_MSG_ACK,
            0,
            self._allocate_sequence(),
            struct.pack("<BH", received.msg_type, received.sequence),
        )
        self._write_frame(ack)

    def _handle_ack(self, frame: Frame, msg_type: MessageType) -> None:
        if len(frame.payload) != 3:
            LOG.warning(
                "ACK/NACK frame seq=%d has invalid payload length=%d",
                frame.sequence,
                len(frame.payload),
            )
            return
        original_type, original_sequence = struct.unpack("<BH", frame.payload)
        key = (original_sequence, original_type)
        pending = self.pending.pop(key, None)
        if pending is None:
            LOG.debug(
                "Stale ACK/NACK frameSeq=%d ackedType=0x%02X ackedSeq=%d",
                frame.sequence,
                original_type,
                original_sequence,
            )
            return

        if msg_type == MessageType.STM_MSG_ACK:
            LOG.debug(
                "ACK type=0x%02X seq=%d", original_type, original_sequence
            )
        else:
            LOG.error(
                "NACK type=0x%02X seq=%d",
                original_type,
                original_sequence,
            )

    def _update_peer_online(self, now: float) -> None:
        if self.last_rx_time is None:
            self.stm_online = False
        elif now - self.last_rx_time > PEER_TIMEOUT_S:
            if self.stm_online:
                LOG.warning("No STM32 packet for %.1f seconds", PEER_TIMEOUT_S)
            self.stm_online = False


class ControlServer(threading.Thread):
    """Local datagram control endpoint; it never touches pyserial."""

    def __init__(self, path: str, link: TripGuardLink,
                 stop_event: threading.Event) -> None:
        super().__init__(name="tripguard-control", daemon=True)
        self.path = path
        self.link = link
        self.stop_event = stop_event
        self.failure: Optional[BaseException] = None

    def run(self) -> None:
        server: Optional[socket.socket] = None
        try:
            parent = os.path.dirname(self.path)
            if parent:
                os.makedirs(parent, mode=0o770, exist_ok=True)
            self._remove_stale_socket()

            server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            server.bind(self.path)
            os.chmod(self.path, 0o660)
            server.settimeout(0.5)
            LOG.info("Control socket ready: %s", self.path)

            while not self.stop_event.is_set():
                try:
                    data, client = server.recvfrom(256)
                except socket.timeout:
                    continue
                except OSError:
                    if self.stop_event.is_set():
                        break
                    raise

                response = self._execute(data)
                if client:
                    try:
                        server.sendto(response.encode("ascii"), client)
                    except OSError as exc:
                        LOG.warning("Cannot reply to control client: %s", exc)
        except BaseException as exc:
            self.failure = exc
            LOG.error("Control socket failed: %s", exc)
            self.stop_event.set()
        finally:
            if server is not None:
                server.close()
            self._remove_stale_socket(ignore_regular_file=True)

    def _remove_stale_socket(self, ignore_regular_file: bool = False) -> None:
        try:
            file_stat = os.lstat(self.path)
        except FileNotFoundError:
            return
        if not stat.S_ISSOCK(file_stat.st_mode):
            if ignore_regular_file:
                return
            raise RuntimeError(
                "refusing to replace non-socket path: %s" % self.path
            )
        os.unlink(self.path)

    def _execute(self, raw: bytes) -> str:
        try:
            text = raw.decode("ascii", errors="strict").strip()
            parts = shlex.split(text)
        except (UnicodeDecodeError, ValueError):
            return "ERR invalid command encoding"
        if not parts:
            return "ERR empty command"

        command = parts[0].lower()
        try:
            if command == "ready" and len(parts) == 1:
                queued = self.link.set_camera_ready()
            elif command == "person" and len(parts) in (2, 3):
                confidence = int(parts[1], 10)
                queued = self.link.queue_camera_person(confidence)
                if queued and len(parts) == 3:
                    if self.link.telemetry is None:
                        return "ERR telemetry store unavailable"
                    evidence_rows = self.link.telemetry.accept_camera_evidence(
                        parts[2], confidence
                    )
                    return "OK queued evidence rows=%d" % evidence_rows
            elif command == "clear" and len(parts) == 1:
                queued = self.link.queue_camera_clear()
            elif command == "status" and len(parts) == 1:
                queued = self.link.queue_status_request()
            elif command == "scan-complete" and len(parts) == 3:
                session_id = int(parts[1], 10)
                result_name = parts[2].lower()
                if result_name not in SCAN_RESULT_CODES:
                    return "ERR invalid scan result"
                queued = self.link.queue_scan_complete(
                    session_id, SCAN_RESULT_CODES[result_name]
                )
            else:
                return (
                    "ERR command must be ready, person N [JPEG], clear, "
                    "status, or scan-complete SESSION RESULT"
                )
        except (OSError, ValueError) as exc:
            return "ERR %s" % exc

        return "OK queued" if queued else "ERR TX queue full"


def send_control_command(socket_path: str, command: str,
                         timeout_s: float = 2.0) -> str:
    """Submit one command to the running daemon and return its response."""

    client_path = os.path.join(
        tempfile.gettempdir(),
        "tripguard-uart-%d-%s.sock" % (os.getpid(), uuid.uuid4().hex[:8]),
    )
    client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        client.bind(client_path)
        os.chmod(client_path, 0o600)
        client.settimeout(timeout_s)
        client.sendto(command.encode("ascii"), socket_path)
        response, _ = client.recvfrom(256)
        return response.decode("ascii", errors="replace")
    finally:
        client.close()
        try:
            file_stat = os.lstat(client_path)
            if stat.S_ISSOCK(file_stat.st_mode):
                os.unlink(client_path)
        except FileNotFoundError:
            pass


def run_daemon(args: argparse.Namespace) -> int:
    stop_event = threading.Event()
    store = TelemetryStore(args.queue_db)
    telemetry = GatewayTelemetry(store)
    scan_controller = ScanController(telemetry=telemetry)
    publisher = ThingsBoardPublisher(
        store, args.tb_host, args.tb_token, args.tb_interface, stop_event
    )
    link = TripGuardLink(
        args.port, args.baud, stop_event, telemetry, scan_controller
    )
    control = ControlServer(args.socket, link, stop_event)

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    control.start()
    publisher.start()
    # READY means the Pi gateway is able to receive a SCAN_START command.
    # It must not depend on the camera process, which is intentionally stopped
    # between end-trip sessions.
    link.set_camera_ready()
    saved_person, saved_confidence = telemetry.get_camera_state()
    if saved_person:
        link.queue_camera_person(saved_confidence)
    else:
        link.queue_camera_clear()
    try:
        link.run()
    except RuntimeError as exc:
        LOG.error("%s", exc)
        stop_event.set()
        return 1
    finally:
        stop_event.set()
        control.join(timeout=2.0)
        publisher.join(timeout=2.0)
        store.close()

    if control.failure is not None:
        return 1
    return 0


def run_self_test() -> int:
    assert crc16_ccitt_false(b"123456789") == 0x29B1

    first = encode_frame(
        MessageType.PI_MSG_CAMERA_PERSON,
        FLAG_ACK_REQUIRED,
        0xFFFF,
        struct.pack("<H", 873),
    )
    second = encode_frame(
        MessageType.PI_MSG_CAMERA_CLEAR,
        FLAG_ACK_REQUIRED,
        0,
        b"",
    )

    parser = FrameParser()
    assert parser.feed(b"noise\x54") == []
    assert parser.feed(first[1:5]) == []
    parsed = parser.feed(first[5:] + second)
    assert len(parsed) == 2
    assert parsed[0].sequence == 0xFFFF
    assert parsed[0].payload == struct.pack("<H", 873)
    assert parsed[1].sequence == 0

    damaged = bytearray(first)
    damaged[-1] ^= 0x80
    recovered = parser.feed(bytes(damaged) + second)
    assert len(recovered) == 1
    assert recovered[0].msg_type == MessageType.PI_MSG_CAMERA_CLEAR
    assert parser.crc_error_count == 1

    invalid_length = MAGIC + struct.pack(
        "<BBBHH", PROTOCOL_VERSION, MessageType.PI_MSG_READY, 0, 9, 129
    )
    recovered = parser.feed(invalid_length + second)
    assert len(recovered) == 1
    assert recovered[0].sequence == 0
    assert parser.length_error_count == 1

    ack_payload = struct.pack(
        "<BH", MessageType.PI_MSG_CAMERA_PERSON, 0xFFFF
    )
    ack_frame = encode_frame(MessageType.STM_MSG_ACK, 0, 77, ack_payload)
    parsed_ack = FrameParser().feed(ack_frame)[0]
    assert parsed_ack.sequence == 77
    assert struct.unpack("<BH", parsed_ack.payload) == (
        MessageType.PI_MSG_CAMERA_PERSON,
        0xFFFF,
    )

    maximum = bytes(range(128))
    maximum_frame = encode_frame(
        MessageType.STM_MSG_EVENT, 0, 123, maximum
    )
    assert len(maximum_frame) == MAX_FRAME_SIZE
    assert FrameParser().feed(maximum_frame)[0].payload == maximum

    scan_start = encode_frame(
        MessageType.STM_MSG_SCAN_START,
        FLAG_ACK_REQUIRED,
        124,
        struct.pack("<IH", 42, 900),
    )
    parsed_scan_start = FrameParser().feed(scan_start)[0]
    assert struct.unpack("<IH", parsed_scan_start.payload) == (42, 900)

    scan_complete = encode_frame(
        MessageType.PI_MSG_SCAN_COMPLETE,
        FLAG_ACK_REQUIRED,
        125,
        struct.pack("<IB", 42, SCAN_RESULT_CODES["occupied"]),
    )
    parsed_scan_complete = FrameParser().feed(scan_complete)[0]
    assert struct.unpack("<IB", parsed_scan_complete.payload) == (42, 1)

    duplicates = DuplicateWindow(capacity=2)
    assert not duplicates.is_duplicate_and_remember(0x83, 0xFFFF)
    assert duplicates.is_duplicate_and_remember(0x83, 0xFFFF)
    assert not duplicates.is_duplicate_and_remember(0x83, 0)

    telemetry_json = json.dumps(
        {"telemetry_seq": 7, "systemState": "STANDBY"},
        separators=(",", ":"),
    ).encode("utf-8")
    assembler = TelemetryAssembler()
    split = len(telemetry_json) // 2
    chunk_1 = struct.pack("<IHH", 9, len(telemetry_json), 0) + \
        telemetry_json[:split]
    chunk_2 = struct.pack("<IHH", 9, len(telemetry_json), split) + \
        telemetry_json[split:]
    assert assembler.feed(chunk_1) is None
    assembled = assembler.feed(chunk_2)
    assert assembled is not None
    assert assembled["telemetry_seq"] == 7

    with tempfile.TemporaryDirectory() as temp_dir:
        store = TelemetryStore(os.path.join(temp_dir, "queue.db"), max_rows=2)
        assert store.enqueue({"n": 1}) == 1
        assert store.enqueue({"n": 2}) == 2
        assert store.enqueue({"n": 3}) == 2
        first_row = store.peek()
        assert first_row is not None
        assert json.loads(first_row[1])["n"] == 2
        store.remove(first_row[0])
        assert store.count() == 1
        store.close()

        legacy_path = os.path.join(temp_dir, "legacy.db")
        legacy_db = sqlite3.connect(legacy_path)
        with legacy_db:
            legacy_db.execute(
                "CREATE TABLE telemetry ("
                "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                "created_ms INTEGER NOT NULL, payload TEXT NOT NULL)"
            )
            legacy_db.execute(
                "INSERT INTO telemetry(created_ms,payload) VALUES(?,?)",
                (1, '{"legacy":true}'),
            )
        legacy_db.close()

        store = TelemetryStore(legacy_path, max_rows=4)
        image_path = os.path.join(temp_dir, "evidence.jpg")
        with open(image_path, "wb") as image_file:
            image_file.write(b"\xff\xd8test-evidence\xff\xd9")
        camera_state_path = os.path.join(temp_dir, "camera-state.json")
        telemetry = GatewayTelemetry(store, camera_state_path)
        telemetry.set_camera(True, 927)
        reloaded_telemetry = GatewayTelemetry(store, camera_state_path)
        assert reloaded_telemetry.get_camera_state() == (True, 927)
        scan = ScanController(
            environment_path=os.path.join(temp_dir, "camera.env"),
            state_path=os.path.join(temp_dir, "scan-state.json"),
            telemetry=reloaded_telemetry,
        )
        # OCCUPIED must reject another scan before any systemctl call occurs.
        assert scan.start(42, 900) is False
        assert telemetry.accept_camera_evidence(image_path, 927) == 2
        evidence_row = store.peek()
        assert evidence_row is not None
        evidence_payload = json.loads(evidence_row[1])
        assert evidence_payload["values"]["eventType"] == "PERSON_DETECTED"
        assert evidence_payload["values"]["confidence"] == 0.927
        assert evidence_payload["values"]["evidenceImage"].startswith(
            "data:image/jpeg;base64,"
        )
        store.close()

    print("TripGuard UART self-test: PASS")
    return 0


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="TripGuard Raspberry Pi UART bridge"
    )
    parser.add_argument(
        "--log-level",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        default="INFO",
    )
    subparsers = parser.add_subparsers(dest="action", required=True)

    daemon_parser = subparsers.add_parser("daemon", help="run UART daemon")
    daemon_parser.add_argument("--port", default=DEFAULT_PORT)
    daemon_parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    daemon_parser.add_argument("--socket", default=DEFAULT_CONTROL_SOCKET)
    daemon_parser.add_argument("--queue-db", default=DEFAULT_QUEUE_DB)
    daemon_parser.add_argument("--tb-host", default=DEFAULT_TB_HOST)
    daemon_parser.add_argument("--tb-token", default=DEFAULT_TB_TOKEN)
    daemon_parser.add_argument(
        "--tb-interface", default=DEFAULT_TB_INTERFACE,
        help="force ThingsBoard host routes through this interface",
    )

    for action in ("ready", "clear", "status"):
        action_parser = subparsers.add_parser(action)
        action_parser.add_argument("--socket", default=DEFAULT_CONTROL_SOCKET)

    person_parser = subparsers.add_parser("person")
    person_parser.add_argument(
        "--confidence",
        type=float,
        default=1.0,
        help="confidence from 0.0 to 1.0 (default: 1.0)",
    )
    person_parser.add_argument(
        "--image",
        default="",
        help="optional JPEG evidence path (maximum 80 KiB)",
    )
    person_parser.add_argument("--socket", default=DEFAULT_CONTROL_SOCKET)

    complete_parser = subparsers.add_parser("scan-complete")
    complete_parser.add_argument(
        "--session-id",
        type=int,
        default=int(os.environ.get("TRIPGUARD_SCAN_SESSION_ID", "0")),
    )
    complete_parser.add_argument(
        "--result", choices=tuple(SCAN_RESULT_CODES), required=True
    )
    complete_parser.add_argument("--socket", default=DEFAULT_CONTROL_SOCKET)

    subparsers.add_parser("self-test", help="test framing/parser without UART")
    return parser


def main() -> int:
    parser = build_argument_parser()
    args = parser.parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    if args.action == "daemon":
        return run_daemon(args)
    if args.action == "self-test":
        return run_self_test()

    if args.action == "person":
        if not 0.0 <= args.confidence <= 1.0:
            parser.error("--confidence must be from 0.0 to 1.0")
        confidence = int(round(args.confidence * 1000.0))
        command = "person %d" % confidence
        if args.image:
            command += " %s" % shlex.quote(os.path.abspath(args.image))
    elif args.action == "scan-complete":
        if not 0 <= args.session_id <= 0xFFFFFFFF:
            parser.error("--session-id must be uint32")
        command = "scan-complete %d %s" % (
            args.session_id, args.result
        )
    else:
        command = args.action

    try:
        response = send_control_command(args.socket, command)
    except (OSError, socket.timeout) as exc:
        LOG.error("Cannot contact daemon at %s: %s", args.socket, exc)
        return 1
    print(response)
    return 0 if response.startswith("OK") else 1


if __name__ == "__main__":
    raise SystemExit(main())
