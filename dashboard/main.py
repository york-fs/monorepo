from collections.abc import AsyncIterable
from contextlib import asynccontextmanager
from enum import Enum, Flag
from events import EventLog, EventTracker, FrameRecorder
from fastapi import FastAPI, HTTPException, Request, Depends
from fastapi.responses import FileResponse, Response
from fastapi.sse import EventSourceResponse, ServerSentEvent
from pathlib import Path
from sessions import export_session, list_sessions
import asyncio
import attrs
import crcmod
import enum
import os
import serial_asyncio
import struct
import time


ERPM_FACTOR = 10
# BMS segment slots, and cells and thermistors per segment, as the rear forwards them.
SEGMENT_COUNT = 10
SEGMENT_CELL_COUNT = 16
SEGMENT_TEMPERATURE_COUNT = 24
# The inverter's max motor current, which the pedal's demand is relative to. Matches
# the frontend's MOTOR_CURRENT_MAX.
MAX_MOTOR_CURRENT = 360


class OnlineFlags(Flag):
    FRONT_ONLINE = enum.auto()
    BMS_ONLINE = enum.auto()
    PRECHARGE_ONLINE = enum.auto()
    INVERTER_ONLINE = enum.auto()


class FuseFlags(Flag):
    BMS = enum.auto()
    IMD = enum.auto()
    TSAC_FANS = enum.auto()
    PRECHARGE = enum.auto()
    COOLANT_PUMP = enum.auto()
    BRAKE_LIGHT = enum.auto()
    TSAL_LED = enum.auto()
    INVERTER = enum.auto()
    SHUTDOWN_LATCH = enum.auto()
    ENERGY_METER = enum.auto()
    RTD_HORN = enum.auto()
    APPS_1 = enum.auto()
    APPS_2 = enum.auto()
    FRONT = enum.auto()
    DWIN = enum.auto()
    AUX_1 = enum.auto()
    AUX_2 = enum.auto()


class ShutdownOpenCause(Enum):
    NONE = 0
    REAR_INPUT = enum.auto()
    FRONT_ESTOP = enum.auto()
    BRAKE_OVER_TRAVEL = enum.auto()
    INERTIA_SWITCH = enum.auto()
    FRONT_AUXILIARY = enum.auto()
    FRONT_OUTPUT = enum.auto()
    BMS_LATCH = enum.auto()
    IMD_LATCH = enum.auto()
    INVERTER_INTERLOCK = enum.auto()
    SHUTDOWN_LATCH_FAILURE = enum.auto()
    LEFT_ESTOP = enum.auto()
    RIGHT_ESTOP = enum.auto()
    HVD_INTERLOCK = enum.auto()
    REAR_AUXILIARY = enum.auto()
    TSMS = enum.auto()


class TsPreventionFlags(Flag):
    SHUTDOWN_OPEN = enum.auto()
    BAD_FUSE = enum.auto()
    FRONT_OFFLINE = enum.auto()
    NOT_REQUESTED = enum.auto()
    PRECHARGE_OFFLINE = enum.auto()
    PRECHARGE_STATE = enum.auto()
    INVERTER_OFFLINE = enum.auto()
    INVERTER_FAULT = enum.auto()
    BMS_OFFLINE = enum.auto()
    BMS_FAULT = enum.auto()


class RtdPreventionFlags(Flag):
    TS_NOT_ACTIVE = enum.auto()
    NOT_REQUESTED = enum.auto()
    APPS_NOT_CALIBRATED = enum.auto()
    BRAKE_NOT_PRESSED = enum.auto()


class InverterFaultCode(Enum):
    NONE = 0
    OVERVOLTAGE = enum.auto()
    UNDERVOLTAGE = enum.auto()
    DRIVE = enum.auto()
    OVERCURRENT = enum.auto()
    CONTROLLER_OVERTEMPERATURE = enum.auto()
    MOTOR_OVERTEMPERATURE = enum.auto()
    SENSOR_WIRE_FAULT = enum.auto()
    SENSOR_GENERAL_FAULT = enum.auto()
    CAN_COMMAND_FAULT = enum.auto()
    ANALOG_INPUT_FAULT = enum.auto()


class PrechargeState(Enum):
    LED_CHECK = 0
    PRECHECK = enum.auto()
    STANDBY = enum.auto()
    PRECHARGE = enum.auto()
    PRECHARGE_HOLD = enum.auto()
    ACTIVE = enum.auto()


class PrechargeErrorFlags(Flag):
    DISCHARGE_OPEN = enum.auto()
    PRECHARGE_CLOSED = enum.auto()
    AIR_POS_CLOSED = enum.auto()
    AIR_NEG_CLOSED = enum.auto()
    PRECHARGE_SAMPLE_VOLTAGE = enum.auto()
    WAITING_DISCHARGE = enum.auto()
    WAITING_ACTIVATION = enum.auto()
    SHUTDOWN_OPEN = enum.auto()
    PRECHARGE_OPEN = enum.auto()
    AIR_POS_OPEN = enum.auto()
    AIR_NEG_OPEN = enum.auto()
    DEACTIVATION = enum.auto()
    SLOW_DEVIATION = enum.auto()
    FAST_DEVIATION = enum.auto()
    RATE_LIMIT = enum.auto()


class PrechargeRelayStates(Flag):
    DISCHARGE_CLOSED = enum.auto()
    PRECHARGE_CLOSED = enum.auto()
    AIR_POS_CLOSED = enum.auto()
    AIR_NEG_CLOSED = enum.auto()


class BmsMasterErrorFlags(Flag):
    CAN_OFFLINE = enum.auto()
    NO_CONFIG = enum.auto()
    DEADLINE_OVERRUN = enum.auto()
    BAD_REFERENCE = enum.auto()
    OVERTEMPERATURE = enum.auto()
    BAD_CURRENT_SENSOR = enum.auto()
    OVERCURRENT_THRESHOLD = enum.auto()
    OVERCURRENT_MEASURED = enum.auto()
    SEGMENT_ERROR = enum.auto()
    BAD_SEGMENT_COUNT = enum.auto()
    BAD_EEPROM = enum.auto()


class SegmentErrorFlags(Flag):
    DISCONNECTED = enum.auto()
    BAD_CELL_COUNT = enum.auto()
    BAD_THERMISTOR_COUNT = enum.auto()
    BAD_RAIL_VOLTAGE = enum.auto()
    UNRELIABLE_COMMUNICATION = enum.auto()
    UNRELIABLE_MEASUREMENT = enum.auto()
    UNDERVOLTAGE = enum.auto()
    OVERVOLTAGE = enum.auto()
    UNDERTEMPERATURE = enum.auto()
    OVERTEMPERATURE = enum.auto()


@attrs.define
class TelemetryFrame:
    uptime: int
    missed_tx_count: int

    # Online statuses.
    online_flags: OnlineFlags = attrs.field(converter=OnlineFlags)

    # Distribution.
    fuses: FuseFlags = attrs.field(converter=FuseFlags)
    lvs_min_voltage: float = attrs.field(converter=lambda V: V / 1000)
    lvs_max_voltage: float = attrs.field(converter=lambda V: V / 1000)
    shutdown_open_cause: ShutdownOpenCause = attrs.field(converter=ShutdownOpenCause)
    ts_prevention_flags: TsPreventionFlags = attrs.field(converter=TsPreventionFlags)
    rtd_prevention_flags: RtdPreventionFlags = attrs.field(converter=RtdPreventionFlags)

    # Powertrain.
    inverter_fault: InverterFaultCode = attrs.field(converter=InverterFaultCode)
    inverter_temperature: float = attrs.field(converter=lambda T: T / 10)
    motor_temperature: float = attrs.field(converter=lambda T: T / 10)
    inverter_input_voltage: int
    motor_current: float = attrs.field(converter=lambda I: I / 10)
    motor_rpm: int = attrs.field(converter=lambda R: R // ERPM_FACTOR)

    # APPS.
    desired_throttle: float = attrs.field(converter=lambda P: P / 10)
    pedal_travel: float = attrs.field(converter=lambda P: P / 10)
    desired_motor_current: int = attrs.field(
        init=False,
        default=attrs.Factory(
            lambda self: self.desired_throttle * MAX_MOTOR_CURRENT / 100,
            takes_self=True,
        ),
    )

    # Precharge status.
    precharge_state: PrechargeState = attrs.field(converter=PrechargeState)
    precharge_error_flags: PrechargeErrorFlags = attrs.field(
        converter=PrechargeErrorFlags
    )
    precharge_prchg_voltage: int
    precharge_ts_voltage: int
    precharge_relay_states: PrechargeRelayStates = attrs.field(
        converter=PrechargeRelayStates
    )

    # BMS.
    bms_master_error_flags: BmsMasterErrorFlags = attrs.field(
        converter=BmsMasterErrorFlags
    )
    bms_i2c_error_count: int
    positive_current: float = attrs.field(converter=lambda I: I / 1000)
    negative_current: float = attrs.field(converter=lambda I: I / 1000)
    min_cell_voltage: float = attrs.field(converter=lambda V: V / 10000)
    max_cell_voltage: float = attrs.field(converter=lambda V: V / 10000)
    min_cell_temperature: int
    max_cell_temperature: int

    # Everything the rear distribution saw since its previous frame, so that flags set
    # for less than a radio period still show up.
    ts_prevention_flags_seen: TsPreventionFlags = attrs.field(
        converter=TsPreventionFlags
    )
    rtd_prevention_flags_seen: RtdPreventionFlags = attrs.field(
        converter=RtdPreventionFlags
    )
    shutdown_open_causes_seen: list[str] = attrs.field(
        converter=lambda bits: [
            cause.name
            for cause in ShutdownOpenCause
            if cause is not ShutdownOpenCause.NONE and bits & (1 << cause.value)
        ]
    )
    blown_fuses_seen: FuseFlags = attrs.field(converter=FuseFlags)
    expander_error_count: int

    # Every BMS segment's error flags, and one segment's full readings. The rear sends
    # the connected segments' readings in turn, one per frame.
    segment_error_flags: list[SegmentErrorFlags]
    segment: dict

    crc: int

    def serialize(self, inst, field, value):
        if isinstance(value, Flag):
            return [flag.name for flag in type(value) if flag in value]
        if isinstance(value, Enum):
            return value.name
        return value

    def to_dict(self):
        return attrs.asdict(self, value_serializer=self.serialize)


FRAME_BASE = struct.Struct(">IBBIHHBHBBhhhhiHHBHHHBHIiiHHbbHBHII")
FRAME_SEGMENTS = struct.Struct(
    f">{SEGMENT_COUNT}HBHHH{SEGMENT_CELL_COUNT}H{SEGMENT_TEMPERATURE_COUNT}b"
)
FRAME_CRC = struct.Struct(">I")


def parse_segment(fields: tuple) -> tuple[list[SegmentErrorFlags], dict]:
    flags = [SegmentErrorFlags(value) for value in fields[:SEGMENT_COUNT]]
    segment_id, degraded, balancing, max_correction = fields[
        SEGMENT_COUNT : SEGMENT_COUNT + 4
    ]
    voltages = fields[SEGMENT_COUNT + 4 : SEGMENT_COUNT + 4 + SEGMENT_CELL_COUNT]
    temperatures = fields[SEGMENT_COUNT + 4 + SEGMENT_CELL_COUNT :]
    cells = range(SEGMENT_CELL_COUNT)
    # Readings the segment doesn't have are sent as 0xffff and -128.
    return flags, {
        "id": segment_id,
        "cell_voltages": [None if v == 0xFFFF else v / 10000 for v in voltages],
        "temperatures": [None if t == -128 else t for t in temperatures],
        "degraded_cells": [i for i in cells if degraded >> i & 1],
        "balancing_cells": [i for i in cells if balancing >> i & 1],
        "max_correction": max_correction,
    }


def parse_frame(data: bytes):
    base = FRAME_BASE.unpack_from(data)
    segment_fields = FRAME_SEGMENTS.unpack_from(data, FRAME_BASE.size)
    (crc,) = FRAME_CRC.unpack_from(data, FRAME_BASE.size + FRAME_SEGMENTS.size)
    flags, segment = parse_segment(segment_fields)
    return TelemetryFrame(*base, flags, segment, crc)


def cobs_unstuff(b: bytes) -> bytearray:
    out = bytearray()
    idx = 0
    while idx < len(b) and b[idx] != 0:
        out.append(0x00)
        count = b[idx]
        out.extend(b[idx + 1 : idx + count])
        idx += count
    return out[1:]


stm_crc = crcmod.mkCrcFun(
    0x104C11DB7,
    initCrc=0xFFFFFFFF,
    xorOut=0,
    rev=False,
)


def pad_data(data: bytes) -> bytes:
    if len(data) % 4 == 0:
        return data
    return data[:-4] + bytes(4 - len(data) % 4) + data[-4:]


class SerialManager:
    subscribers: set[asyncio.Queue]
    event_log: EventLog
    event_tracker: EventTracker
    frame_recorder: FrameRecorder

    def __init__(self, log_directory: Path):
        self.subscribers = set()
        self.event_log = EventLog(log_directory)
        self.event_tracker = EventTracker(self.event_log)
        self.frame_recorder = FrameRecorder(log_directory)

    def subscribe(self) -> asyncio.Queue:
        queue = asyncio.Queue()
        self.subscribers.add(queue)
        return queue

    def unsubscribe(self, queue: asyncio.Queue):
        self.subscribers.discard(queue)

    def publish(self, frame: TelemetryFrame):
        now = time.time()
        data = frame.to_dict()
        self.frame_recorder.record(data, now)
        events = self.event_tracker.process(data, now)
        for queue in self.subscribers:
            queue.put_nowait(data)
            for event in events:
                queue.put_nowait(ServerSentEvent(data=event, event="log"))


class SerialReaderProtocol(asyncio.Protocol):
    manager: SerialManager
    buffer: bytearray

    def __init__(self, manager: SerialManager):
        self.manager = manager
        self.buffer = bytearray()

    def connection_made(self, transport):
        self.manager.transport = transport

    def _pop_frame(self):
        # Extract up to the frame delimiter.
        frame, _, rest = self.buffer.partition(b"\x00")
        self.buffer = bytearray(rest)

        unstuffed = cobs_unstuff(frame)
        if stm_crc(pad_data(unstuffed)) == 0:
            self.manager.publish(parse_frame(unstuffed))
        else:
            print("Bad CRC!")

    def data_received(self, data: bytes):
        self.buffer.extend(data)
        while 0x0 in self.buffer:
            self._pop_frame()


@asynccontextmanager
async def lifespan(app: FastAPI):
    manager = SerialManager(Path(os.environ.get("DASHBOARD_LOG_DIR", "logs")))
    app.state.serial_manager = manager

    loop = asyncio.get_running_loop()
    transport, protocol = await serial_asyncio.create_serial_connection(
        loop,
        lambda: SerialReaderProtocol(manager),
        "/dev/ttyUSB0",
        baudrate=115200,
    )
    yield
    transport.close()
    manager.frame_recorder.close()


app = FastAPI(lifespan=lifespan)
app.frontend("/", directory="frontend/dist")


def get_serial_manager(request: Request) -> SerialManager:
    return request.app.state.serial_manager


@app.get("/api/stream", response_class=EventSourceResponse)
async def stream(
    manager: SerialManager = Depends(get_serial_manager),
) -> AsyncIterable[ServerSentEvent]:
    queue = manager.subscribe()
    try:
        while True:
            yield await queue.get()
    finally:
        manager.unsubscribe(queue)


@app.get("/api/events")
async def get_events(
    manager: SerialManager = Depends(get_serial_manager),
) -> list[dict]:
    return list(manager.event_log.events)


@app.post("/api/events/clear")
async def clear_events(manager: SerialManager = Depends(get_serial_manager)):
    manager.event_log.clear()


@app.get("/api/events/download")
async def download_events(manager: SerialManager = Depends(get_serial_manager)):
    if not manager.event_log.path.exists():
        raise HTTPException(status_code=404, detail="No events logged yet")
    return FileResponse(manager.event_log.path, filename="events.jsonl")


@app.get("/api/sessions")
async def get_sessions(
    manager: SerialManager = Depends(get_serial_manager),
) -> list[dict]:
    recorder = manager.frame_recorder
    return list_sessions(recorder.path.parent, recorder.path)


# A plain function, so FastAPI runs the file reading and zipping in its threadpool
# rather than blocking the telemetry stream.
@app.get("/api/sessions/{session_id}/export")
def export_session_zip(
    session_id: str, manager: SerialManager = Depends(get_serial_manager)
):
    archive = export_session(manager.event_log.directory, session_id)
    if archive is None:
        raise HTTPException(status_code=404, detail="No such session")
    filename = f"yfs-session-{session_id}.zip"
    return Response(
        archive,
        media_type="application/zip",
        headers={"Content-Disposition": f'attachment; filename="{filename}"'},
    )
