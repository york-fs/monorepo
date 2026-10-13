"""Event log for the dashboard.

Compares consecutive telemetry frames and records everything that changed: flags
raised and cleared, flags which came and went between two frames, and TS/RTD
activations and deactivations with a snapshot of what led up to them. Events and
raw frames are written to disk, so the history survives restarts and is recorded
whether or not a browser is open.
"""

from collections import deque
from datetime import datetime
from pathlib import Path
import json
import time


# How long a telemetry silence must last before it's logged as a gap.
GAP_SECONDS = 3.0

# How far back to look for problems raised in the run-up to a deactivation.
LEAD_UP_SECONDS = 2.0

# Number of events kept in memory and served to the frontend.
EVENT_CAPACITY = 5000

# Flag fields where a set flag is a problem.
PROBLEM_FLAG_FIELDS = {
    "ts_prevention_flags": "ts",
    "rtd_prevention_flags": "rtd",
    "bms_master_error_flags": "bms",
    "precharge_error_flags": "precharge",
}

# Flag fields where a set flag is healthy, so a flag disappearing is the problem.
HEALTHY_FLAG_FIELDS = {
    "online_flags": "online",
    "fuses": "fuse",
}

# Enum fields where NONE is the healthy value.
ENUM_FIELDS = {
    "shutdown_open_cause": "shutdown",
    "inverter_fault": "inverter",
}

# Flags which describe routine progress (waiting on the driver or another state)
# rather than a fault.
ROUTINE_FLAGS = {
    "ts_prevention_flags": {"NOT_REQUESTED"},
    "rtd_prevention_flags": {"NOT_REQUESTED", "BRAKE_NOT_PRESSED", "TS_NOT_ACTIVE"},
    "precharge_error_flags": {"WAITING_DISCHARGE", "WAITING_ACTIVATION", "RATE_LIMIT"},
}

# Fields listing every flag the rear distribution saw since its previous frame,
# which catch flags set and cleared in between. Only sent by newer rear firmware.
SEEN_FLAG_FIELDS = {
    "ts_prevention_flags": "ts_prevention_flags_seen",
    "rtd_prevention_flags": "rtd_prevention_flags_seen",
}

# Code for failed rear GPIO expander reads, logged under the shutdown category since
# the expander samples the rear shutdown inputs.
EXPANDER_READ_CODE = "EXPANDER_READ"

# Event kinds that mark a problem appearing, and so belong in a lead-up.
PROBLEM_KINDS = {"raised", "blip"}

# Fields captured alongside TS/RTD deactivations and connection events.
SNAPSHOT_FIELDS = [
    "shutdown_open_cause",
    "ts_prevention_flags",
    "rtd_prevention_flags",
    "online_flags",
    "fuses",
    "precharge_state",
    "precharge_error_flags",
    "precharge_relay_states",
    "precharge_prchg_voltage",
    "precharge_ts_voltage",
    "bms_master_error_flags",
    "bms_i2c_error_count",
    "inverter_fault",
    "inverter_input_voltage",
    "motor_current",
    "motor_rpm",
    "positive_current",
    "negative_current",
    "min_cell_voltage",
    "max_cell_voltage",
    "max_cell_temperature",
    "lvs_min_voltage",
    "ts_prevention_flags_seen",
    "rtd_prevention_flags_seen",
    "shutdown_open_causes_seen",
    "blown_fuses_seen",
    "expander_error_count",
]

PRECHARGING_STATES = {"PRECHARGE", "PRECHARGE_HOLD"}


def timestamp_slug() -> str:
    return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


class EventLog:
    """Keeps recent events in memory and appends each one to a JSON lines file."""

    directory: Path
    path: Path
    events: deque[dict]
    next_id: int

    def __init__(self, directory: Path):
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / "events.jsonl"
        self.events = deque(maxlen=EVENT_CAPACITY)
        # Seed ids from the clock so they keep increasing across clears and restarts,
        # which lets a browser that stayed open tell new events from ones it has.
        self.next_id = int(time.time() * 1000)
        self._load()

    def _load(self):
        if not self.path.exists():
            return
        with self.path.open() as file:
            for line in file:
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    # Most likely a line cut short by the server being killed.
                    continue
                self.events.append(event)
                self.next_id = max(self.next_id, event.get("id", 0) + 1)

    def append(self, event: dict) -> dict:
        event["id"] = self.next_id
        self.next_id += 1
        self.events.append(event)
        with self.path.open("a") as file:
            file.write(json.dumps(event, separators=(",", ":")) + "\n")
        return event

    def clear(self):
        # Archive rather than delete, in case the history is needed after all.
        if self.path.exists():
            self.path.rename(self.directory / f"events-{timestamp_slug()}.jsonl")
        self.events.clear()


class FrameRecorder:
    """Appends every raw telemetry frame to a JSON lines file per server session."""

    path: Path

    def __init__(self, directory: Path):
        frames_directory = directory / "frames"
        frames_directory.mkdir(parents=True, exist_ok=True)
        self.path = frames_directory / f"{timestamp_slug()}.jsonl"
        self._file = self.path.open("a", buffering=1)

    def record(self, frame: dict, now: float):
        line = json.dumps({"time": int(now * 1000), **frame}, separators=(",", ":"))
        self._file.write(line + "\n")

    def close(self):
        self._file.close()


class EventTracker:
    """Compares consecutive telemetry frames and logs anything that changed."""

    log: EventLog
    previous: dict | None
    previous_time: float | None

    def __init__(self, log: EventLog):
        self.log = log
        self.previous = None
        self.previous_time = None

    def process(self, frame: dict, now: float) -> list[dict]:
        previous, previous_time = self.previous, self.previous_time
        self.previous, self.previous_time = frame, now

        if previous is None:
            snapshot = self._snapshot(frame, now)
            return [
                self._log(frame, now, "system", "connected", "info", snapshot=snapshot)
            ]

        events = []
        if now - previous_time >= GAP_SECONDS:
            gap = f"{now - previous_time:.1f}"
            events.append(self._log(frame, now, "system", "gap", "info", detail=gap))
        rebooted = frame["uptime"] < previous["uptime"]
        if rebooted:
            events.append(self._log(frame, now, "system", "reboot", "info"))

        # Flag changes are logged first so that a deactivation snapshot includes them.
        # Flags seen during a reboot are start-up transients, so blips are skipped.
        events.extend(self._diff_flags(previous, frame, now, blips=not rebooted))
        events.extend(self._diff_activation(previous, frame, now))
        return events

    def _log(
        self, frame: dict, now: float, category: str, kind: str, severity: str, **extra
    ) -> dict:
        event = {
            "time": int(now * 1000),
            "uptime": frame.get("uptime"),
            "category": category,
            "kind": kind,
            "severity": severity,
        }
        event.update({key: value for key, value in extra.items() if value is not None})
        return self.log.append(event)

    def _flag_event(
        self,
        frame: dict,
        now: float,
        field: str,
        category: str,
        code: str,
        kind: str,
        active: bool = False,
    ) -> dict:
        # Routine flags only matter if they interrupt something which was active,
        # such as a brief TS_NOT_ACTIVE which unlatches RTD.
        if code in ROUTINE_FLAGS.get(field, ()) and not active:
            severity = "info"
        else:
            severity = "ok" if kind == "cleared" else "fault"
        return self._log(frame, now, category, kind, severity, code=code)

    def _diff_flags(
        self, previous: dict, frame: dict, now: float, blips: bool
    ) -> list[dict]:
        events = []
        for field, category in PROBLEM_FLAG_FIELDS.items():
            before, after = set(previous[field]), set(frame[field])
            # A blip is a flag which was set and cleared again between two frames.
            seen = set(frame.get(SEEN_FLAG_FIELDS.get(field)) or ()) if blips else set()
            for code in sorted(seen - before - after):
                event = self._flag_event(
                    frame, now, field, category, code, "blip", active=not before
                )
                events.append(event)
            for code in sorted(after - before):
                events.append(
                    self._flag_event(frame, now, field, category, code, "raised")
                )
            for code in sorted(before - after):
                events.append(
                    self._flag_event(frame, now, field, category, code, "cleared")
                )

        for field, category in HEALTHY_FLAG_FIELDS.items():
            before, after = set(previous[field]), set(frame[field])
            for code in sorted(before - after):
                events.append(
                    self._flag_event(frame, now, field, category, code, "raised")
                )
            for code in sorted(after - before):
                events.append(
                    self._flag_event(frame, now, field, category, code, "cleared")
                )

        if blips:
            # Fuses which read as blown between two frames where they read fine.
            blown_seen = set(frame.get("blown_fuses_seen") or ())
            dipped = blown_seen & set(previous["fuses"]) & set(frame["fuses"])
            for code in sorted(dipped):
                events.append(
                    self._flag_event(frame, now, "fuses", "fuse", code, "blip")
                )

        for field, category in ENUM_FIELDS.items():
            before, after = previous[field], frame[field]
            if blips and field == "shutdown_open_cause":
                seen = set(frame.get("shutdown_open_causes_seen") or ())
                for code in sorted(seen - {before, after}):
                    events.append(
                        self._flag_event(frame, now, field, category, code, "blip")
                    )
            if before == after:
                continue
            if before != "NONE":
                events.append(
                    self._flag_event(frame, now, field, category, before, "cleared")
                )
            if after != "NONE":
                events.append(
                    self._flag_event(frame, now, field, category, after, "raised")
                )

        # The count resets when the rear reboots, so only an increase is a failure.
        expander_before = previous.get("expander_error_count")
        expander_after = frame.get("expander_error_count")
        if expander_before is not None and expander_after is not None:
            if expander_after > expander_before:
                failures = str(expander_after - expander_before)
                events.append(
                    self._log(
                        frame,
                        now,
                        "shutdown",
                        "blip",
                        "fault",
                        code=EXPANDER_READ_CODE,
                        detail=failures,
                    )
                )

        before, after = previous["precharge_state"], frame["precharge_state"]
        if before != after:
            events.append(
                self._log(
                    frame,
                    now,
                    "precharge",
                    "state",
                    "info",
                    code=after,
                    previous=before,
                )
            )
        return events

    def _diff_activation(self, previous: dict, frame: dict, now: float) -> list[dict]:
        events = []

        # The TS is on while the precharge is in its active state.
        before, after = previous["precharge_state"], frame["precharge_state"]
        if before != "ACTIVE" and after == "ACTIVE":
            events.append(self._log(frame, now, "ts", "activated", "ok"))
        elif before == "ACTIVE" and after != "ACTIVE":
            snapshot = self._snapshot(frame, now)
            events.append(
                self._log(frame, now, "ts", "deactivated", "fault", snapshot=snapshot)
            )
        elif before in PRECHARGING_STATES and after == "PRECHECK":
            snapshot = self._snapshot(frame, now)
            events.append(
                self._log(frame, now, "ts", "aborted", "fault", snapshot=snapshot)
            )

        # RTD is active while nothing is preventing it.
        rtd_before = not previous["rtd_prevention_flags"]
        rtd_after = not frame["rtd_prevention_flags"]
        if not rtd_before and rtd_after:
            events.append(self._log(frame, now, "rtd", "activated", "ok"))
        elif rtd_before and not rtd_after:
            snapshot = self._snapshot(frame, now)
            events.append(
                self._log(frame, now, "rtd", "deactivated", "fault", snapshot=snapshot)
            )
        return events

    def _snapshot(self, frame: dict, now: float) -> dict:
        # Problems raised shortly beforehand often explain a deactivation better than
        # the flags still set afterwards, e.g. a brief BMS fault which makes the front
        # distribution drop its TS request.
        cutoff = (now - LEAD_UP_SECONDS) * 1000
        lead_up = []
        for event in reversed(self.log.events):
            if event["time"] < cutoff:
                break
            if event["kind"] in PROBLEM_KINDS:
                lead_up.append(
                    {
                        "category": event["category"],
                        "code": event["code"],
                        "severity": event["severity"],
                        "before_ms": int(now * 1000) - event["time"],
                    }
                )
        return {
            "frame": {field: frame.get(field) for field in SNAPSHOT_FIELDS},
            "lead_up": lead_up,
        }
