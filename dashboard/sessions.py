"""Recorded sessions and their export.

Every server run records its raw frames to its own file (see FrameRecorder), so
a session is one of those files. Exporting one bundles its frames with the
events logged over the same time span and a summary, as a zip that's ready to
hand to a script or an AI for analysis afterwards.
"""

from datetime import datetime
from itertools import pairwise
from pathlib import Path
import io
import json
import re
import zipfile


# Session ids are FrameRecorder's timestamp slugs. Checked before touching the
# filesystem, so an id can never reach outside the frames directory.
SESSION_ID = re.compile(r"\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}")

# Longest gap between frames to integrate energy across; anything longer is a
# dropped link rather than a steady load.
MAX_STEP_SECONDS = 0.5

# Plausible cell readings. Anything outside is a placeholder the BMS sends while a
# segment isn't reporting.
CELL_VOLTAGE_RANGE = (2.0, 5.0)
CELL_TEMPERATURE_RANGE = (-40, 120)

README = """\
YFS car telemetry session, recorded by the dashboard from the rear distribution's
radio link.

Files:
- frames.jsonl: one telemetry frame per line, roughly every 100 ms. "time" is the
  laptop's wall clock in ms since the epoch, "uptime" the rear distribution's in ms
  (it restarts from zero when the rear reboots). Flag fields list the flags currently
  set; for "online_flags" and "fuses" that means the healthy ones. The *_seen fields
  list everything the rear saw since its previous frame, catching flags set and
  cleared in between. "segment_error_flags" lists each BMS segment slot's errors (A is
  slot 0); "segment" carries one segment's full readings per frame, in turn.
- events.jsonl: the dashboard's event log over the same span, one change per line,
  with a snapshot and lead-up on TS and RTD deactivations.
- summary.json: headline figures worked out from the frames.

Units: voltages V, currents A, temperatures degC, motor_rpm rpm. positive_current and
negative_current are the BMS's pack current sensors, low-pass filtered with a 0.5 s
time constant. motor_current is the inverter's AC motor current. TS is active while
precharge_state is ACTIVE, and RTD while rtd_prevention_flags is empty.
"""


def list_sessions(frames_directory: Path, current: Path) -> list[dict]:
    """Every recorded session, newest first. Empty runs with no frames are left out."""
    sessions = []
    for path in sorted(frames_directory.glob("*.jsonl"), reverse=True):
        if not SESSION_ID.fullmatch(path.stem):
            continue
        size = path.stat().st_size
        if size == 0 and path != current:
            continue
        started = datetime.strptime(path.stem, "%Y-%m-%d_%H-%M-%S")
        sessions.append(
            {
                "id": path.stem,
                "started": started.isoformat(),
                "bytes": size,
                "current": path == current,
            }
        )
    return sessions


def _read_jsonl(path: Path) -> list[dict]:
    rows = []
    with path.open() as file:
        for line in file:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                # Most likely a line cut short, or still being written.
                continue
    return rows


def _session_events(log_directory: Path, start_ms: int, end_ms: int) -> list[dict]:
    # The event log is archived rather than deleted when cleared, so a session's
    # events can be spread over the live file and any archives.
    events = {}
    for path in log_directory.glob("events*.jsonl"):
        for event in _read_jsonl(path):
            if start_ms <= event.get("time", 0) <= end_ms:
                events[event["id"]] = event
    return sorted(events.values(), key=lambda event: event["id"])


def _extreme(frames: list[dict], field: str, pick, valid=None):
    values = [
        frame[field]
        for frame in frames
        if isinstance(frame.get(field), (int, float))
        and (valid is None or valid[0] <= frame[field] <= valid[1])
    ]
    return pick(values) if values else None


def _lead_up_faults(event: dict) -> list[str]:
    # The snapshot's lead-up, oldest first, with the routine waiting states and
    # repeats left out, which is what explains a deactivation.
    lead_up = event.get("snapshot", {}).get("lead_up", [])
    faults = [
        f"{entry['category']} {entry['code']}"
        for entry in reversed(lead_up)
        if entry.get("severity") == "fault"
    ]
    return list(dict.fromkeys(faults))


def _summarise(session_id: str, frames: list[dict], events: list[dict]) -> dict:
    active_seconds = 0.0
    energy_wh = 0.0
    activations = 0
    reboots = 0
    for before, after in pairwise(frames):
        if after.get("uptime", 0) < before.get("uptime", 0):
            reboots += 1
        if (
            before.get("precharge_state") != "ACTIVE"
            and after.get("precharge_state") == "ACTIVE"
        ):
            activations += 1
        step = (after["time"] - before["time"]) / 1000
        if not 0 < step <= MAX_STEP_SECONDS:
            continue
        if before.get("precharge_state") == "ACTIVE":
            active_seconds += step
        volts, amps = (
            before.get("inverter_input_voltage"),
            before.get("positive_current"),
        )
        if volts is not None and amps is not None:
            energy_wh += volts * amps * step / 3600

    powers = [
        frame["inverter_input_voltage"] * frame["positive_current"] / 1000
        for frame in frames
        if frame.get("inverter_input_voltage") is not None
        and frame.get("positive_current") is not None
    ]

    faults = {}
    for event in events:
        if event.get("severity") == "fault":
            key = f"{event['category']} {event['kind']} {event.get('code', '')}".strip()
            faults[key] = faults.get(key, 0) + 1

    def wall_clock(ms: int) -> str:
        return datetime.fromtimestamp(ms / 1000).isoformat(timespec="seconds")

    return {
        "session": session_id,
        "start": wall_clock(frames[0]["time"]),
        "end": wall_clock(frames[-1]["time"]),
        "duration_s": round((frames[-1]["time"] - frames[0]["time"]) / 1000),
        "frames": len(frames),
        "rear_reboots": reboots,
        "ts_activations": activations,
        "ts_active_s": round(active_seconds),
        "ts_deactivations": [
            {
                "time": wall_clock(event["time"]),
                "faults_before": _lead_up_faults(event),
            }
            for event in events
            if event.get("category") == "ts"
            and event.get("kind") in ("deactivated", "aborted")
        ],
        "battery_energy_wh": round(energy_wh, 1),
        "peak": {
            "motor_rpm": _extreme(frames, "motor_rpm", max),
            "motor_current_a": _extreme(frames, "motor_current", max),
            "battery_current_a": _extreme(frames, "positive_current", max),
            "battery_power_kw": round(max(powers), 2) if powers else None,
            "inverter_temperature_c": _extreme(frames, "inverter_temperature", max),
            "motor_temperature_c": _extreme(frames, "motor_temperature", max),
            "cell_temperature_c": _extreme(
                frames, "max_cell_temperature", max, CELL_TEMPERATURE_RANGE
            ),
        },
        "lowest": {
            "cell_voltage_v": _extreme(
                frames, "min_cell_voltage", min, CELL_VOLTAGE_RANGE
            ),
            # Only while TS is on: it sits near zero the rest of the time.
            "ts_voltage_while_active_v": _extreme(
                [f for f in frames if f.get("precharge_state") == "ACTIVE"],
                "inverter_input_voltage",
                min,
            ),
            "lv_voltage_v": _extreme(frames, "lvs_min_voltage", min),
        },
        "fault_event_counts": dict(sorted(faults.items(), key=lambda item: -item[1])),
    }


def export_session(log_directory: Path, session_id: str) -> bytes | None:
    """The session as a zip of summary, frames and events, or None if there's none."""
    path = log_directory / "frames" / f"{session_id}.jsonl"
    if not SESSION_ID.fullmatch(session_id) or not path.is_file():
        return None

    frames = _read_jsonl(path)
    events = (
        _session_events(log_directory, frames[0]["time"], frames[-1]["time"])
        if frames
        else []
    )
    summary = (
        _summarise(session_id, frames, events)
        if frames
        else {"session": session_id, "frames": 0}
    )

    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("README.txt", README)
        archive.writestr("summary.json", json.dumps(summary, indent=2))
        archive.write(path, "frames.jsonl")
        archive.writestr(
            "events.jsonl", "".join(json.dumps(event) + "\n" for event in events)
        )
    return buffer.getvalue()
