from __future__ import annotations

import math
import tomllib
import uuid
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

PROTOCOL_PREFIX = "UAS1"
MAX_STATUSTEXT_CHARS = 50
CHUNKABLE_KINDS = {"E", "V", "L"}

KIND_COMMAND = "C"
KIND_ACK = "A"
KIND_ERROR = "E"
KIND_EVENT = "V"
KIND_LOG = "L"
KIND_HEARTBEAT = "H"

SEVERITY_INFO = 6
SEVERITY_WARNING = 4
SEVERITY_ERROR = 3

REPO_ROOT = Path(__file__).resolve().parents[1]
PARAMETER_FILE = REPO_ROOT / "parameter_files" / "mavlink_command_system.toml"
RECEIVER_LOG_DIR = REPO_ROOT / "logs" / "mavlink_command_receiver"


@dataclass(slots=True)
class ScriptSpec:
    script_path: str
    category: str
    friendly_name: str
    restart_policy: str
    log_rate_limit_hz: float


@dataclass(slots=True)
class SystemConfig:
    pixhawk_connection: str
    gcs_sender_connection: str
    gcs_receiver_connection: str
    link_loss_timeout_s: float
    heartbeat_period_s: float
    ack_timeout_s: float
    process_stop_timeout_s: float
    daemon_source_system: int
    daemon_source_component: int
    sender_source_system: int
    sender_source_component: int
    receiver_source_system: int
    receiver_source_component: int
    script_specs: dict[str, ScriptSpec]

    def allowed_scripts(self) -> set[str]:
        return set(self.script_specs)

    def resolve_script(self, script_path: str) -> ScriptSpec:
        normalized = normalize_repo_script_path(script_path)
        if normalized not in self.script_specs:
            raise ValueError(f"Script is not registered in the parameter file: {normalized}")
        return self.script_specs[normalized]


@dataclass(slots=True)
class OperatorCommand:
    action: str
    script_path: str | None
    canonical_text: str


@dataclass(slots=True)
class ProtocolFrame:
    kind: str
    ref: str
    payload: str


def normalize_repo_script_path(raw_path: str) -> str:
    cleaned = raw_path.strip().replace("\\", "/")
    if cleaned.startswith("./"):
        cleaned = cleaned[2:]

    path = PurePosixPath(cleaned)
    if path.is_absolute():
        raise ValueError("Absolute paths are not allowed.")
    if ".." in path.parts:
        raise ValueError("Parent-directory traversal is not allowed.")
    if not path.parts or path.parts[0] != "src":
        raise ValueError("Managed scripts must live under src/.")
    if path.suffix != ".py":
        raise ValueError("Managed scripts must be Python files under src/.")

    return path.as_posix()


def parse_operator_command(text: str) -> OperatorCommand:
    stripped = text.strip()
    if not stripped:
        raise ValueError("Command cannot be empty.")

    collapsed = " ".join(stripped.split())
    if collapsed.upper() == "STOP ALL":
        return OperatorCommand(action="STOP_ALL", script_path=None, canonical_text="STOP ALL")

    parts = collapsed.split(" ", 1)
    if len(parts) != 2:
        raise ValueError("Commands must be START src/<script>.py, STOP src/<script>.py, or STOP ALL.")

    action = parts[0].upper()
    if action not in {"START", "STOP"}:
        raise ValueError("Only START, STOP, and STOP ALL are supported.")

    script_path = normalize_repo_script_path(parts[1])
    return OperatorCommand(
        action=action,
        script_path=script_path,
        canonical_text=f"{action} {script_path}",
    )


def load_system_config(parameter_file: Path = PARAMETER_FILE) -> SystemConfig:
    with parameter_file.open("rb") as handle:
        raw = tomllib.load(handle)

    maneuver_scripts = [normalize_repo_script_path(item) for item in raw.get("maneuver_scripts", [])]
    background_scripts = [normalize_repo_script_path(item) for item in raw.get("background_scripts", [])]

    overlap = set(maneuver_scripts) & set(background_scripts)
    if overlap:
        raise ValueError(f"Scripts cannot be both maneuver and background: {sorted(overlap)}")

    metadata_table = raw.get("script_metadata", {})
    script_specs: dict[str, ScriptSpec] = {}

    for script_path in maneuver_scripts:
        metadata = metadata_table.get(script_path, {})
        script_specs[script_path] = ScriptSpec(
            script_path=script_path,
            category="maneuver",
            friendly_name=metadata.get("friendly_name", PurePosixPath(script_path).stem),
            restart_policy=metadata.get("restart_policy", "never"),
            log_rate_limit_hz=float(metadata.get("log_rate_limit_hz", 2.0)),
        )

    for script_path in background_scripts:
        metadata = metadata_table.get(script_path, {})
        script_specs[script_path] = ScriptSpec(
            script_path=script_path,
            category="background",
            friendly_name=metadata.get("friendly_name", PurePosixPath(script_path).stem),
            restart_policy=metadata.get("restart_policy", "never"),
            log_rate_limit_hz=float(metadata.get("log_rate_limit_hz", 1.0)),
        )

    if not script_specs:
        raise ValueError("At least one managed script must be registered.")

    return SystemConfig(
        pixhawk_connection=raw["pixhawk_connection"],
        gcs_sender_connection=raw["gcs_sender_connection"],
        gcs_receiver_connection=raw["gcs_receiver_connection"],
        link_loss_timeout_s=float(raw["link_loss_timeout_s"]),
        heartbeat_period_s=float(raw["heartbeat_period_s"]),
        ack_timeout_s=float(raw["ack_timeout_s"]),
        process_stop_timeout_s=float(raw["process_stop_timeout_s"]),
        daemon_source_system=int(raw["daemon_source_system"]),
        daemon_source_component=int(raw["daemon_source_component"]),
        sender_source_system=int(raw["sender_source_system"]),
        sender_source_component=int(raw["sender_source_component"]),
        receiver_source_system=int(raw["receiver_source_system"]),
        receiver_source_component=int(raw["receiver_source_component"]),
        script_specs=script_specs,
    )


def encode_protocol_frames(kind: str, ref: str, payload: str, allow_chunking: bool = False) -> list[str]:
    if "|" in ref:
        raise ValueError("Frame references cannot contain '|'.")

    frame_prefix = f"{PROTOCOL_PREFIX}|{kind}|{ref}|"
    if len(frame_prefix) + len(payload) <= MAX_STATUSTEXT_CHARS:
        return [frame_prefix + payload]

    if not allow_chunking or kind not in CHUNKABLE_KINDS:
        raise ValueError("Frame exceeds MAVLink STATUSTEXT size limit.")

    marker_width = len("[01/99] ")
    payload_capacity = MAX_STATUSTEXT_CHARS - len(frame_prefix) - marker_width
    if payload_capacity <= 0:
        raise ValueError("Frame header leaves no room for chunked payload data.")

    chunk_count = math.ceil(len(payload) / payload_capacity)
    if chunk_count > 99:
        raise ValueError("Payload is too large to chunk safely.")

    frames: list[str] = []
    for index in range(chunk_count):
        start = index * payload_capacity
        end = start + payload_capacity
        marker = f"[{index + 1:02d}/{chunk_count:02d}] "
        frames.append(frame_prefix + marker + payload[start:end])
    return frames


def parse_protocol_frame(text: str) -> ProtocolFrame | None:
    cleaned = text.replace("\x00", "").strip()
    parts = cleaned.split("|", 3)
    if len(parts) != 4:
        return None
    if parts[0] != PROTOCOL_PREFIX:
        return None
    return ProtocolFrame(kind=parts[1], ref=parts[2], payload=parts[3])


def protocol_severity(kind: str) -> int:
    if kind == KIND_ERROR:
        return SEVERITY_ERROR
    if kind == KIND_EVENT:
        return SEVERITY_WARNING
    return SEVERITY_INFO


def build_command_payload(session_id: str, command: OperatorCommand) -> str:
    return f"{validate_session_id(session_id)} {command.canonical_text}"


def unpack_command_payload(payload: str) -> tuple[str, OperatorCommand]:
    parts = payload.strip().split(" ", 1)
    if len(parts) != 2:
        raise ValueError("Command payload is missing a session identifier.")
    session_id = validate_session_id(parts[0])
    command = parse_operator_command(parts[1])
    return session_id, command


def validate_session_id(session_id: str) -> str:
    cleaned = session_id.strip().upper()
    if len(cleaned) != 4 or not cleaned.isalnum():
        raise ValueError("Session identifiers must be exactly four alphanumeric characters.")
    return cleaned


def new_session_id() -> str:
    return uuid.uuid4().hex[:4].upper()


def statustext_to_string(message: object) -> str:
    text = getattr(message, "text", "")
    if isinstance(text, bytes):
        return text.decode("utf-8", errors="replace")
    return str(text)
