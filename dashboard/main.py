from collections.abc import AsyncIterable
from contextlib import asynccontextmanager
from enum import Enum, Flag
from events import EventLog, EventTracker, FrameRecorder
from fastapi import FastAPI, HTTPException, Request, Depends
from fastapi.responses import FileResponse
from fastapi.sse import EventSourceResponse, ServerSentEvent
from pathlib import Path
import asyncio
import attrs
import crcmod
import enum
import os
import serial_asyncio
import struct
import time


ERPM_FACTOR = 10
MAX_MOTOR_CURRENT = 200


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

    crc: int

    # Everything the rear distribution saw since its previous frame, so that flags set
    # for less than a radio period still show up. None for older rear firmware.
    ts_prevention_flags_seen: TsPreventionFlags | None = attrs.field(
        default=None, converter=attrs.converters.optional(TsPreventionFlags)
    )
    rtd_prevention_flags_seen: RtdPreventionFlags | None = attrs.field(
        default=None, converter=attrs.converters.optional(RtdPreventionFlags)
    )
    shutdown_open_causes_seen: list[str] | None = attrs.field(
        default=None,
        converter=attrs.converters.optional(
            lambda bits: [
                cause.name
                for cause in ShutdownOpenCause
                if cause is not ShutdownOpenCause.NONE and bits & (1 << cause.value)
            ]
        ),
    )
    blown_fuses_seen: FuseFlags | None = attrs.field(
        default=None, converter=attrs.converters.optional(FuseFlags)
    )
    expander_error_count: int | None = None

    def serialize(self, inst, field, value):
        if isinstance(value, Flag):
            return [flag.name for flag in type(value) if flag in value]
        if isinstance(value, Enum):
            return value.name
        return value

    def to_dict(self):
        return attrs.asdict(self, value_serializer=self.serialize)


# Frame fields before the CRC. Newer rear firmware appends the flags it saw since its
# previous frame, also before the CRC.
FRAME_FORMAT = ">IBBIHHBHBBhhhhiHHBHHHBHIiiHHbb"
FRAME_FORMAT_SEEN = "HBHII"


def parse_frame(data: bytes):
    # Accept frames with and without the seen fields, so the dashboard keeps working
    # with a rear distribution that hasn't been reflashed.
    if len(data) == struct.calcsize(FRAME_FORMAT + FRAME_FORMAT_SEEN + "I"):
        *fields, crc = struct.unpack(FRAME_FORMAT + FRAME_FORMAT_SEEN + "I", data)
        split = len(fields) - len(FRAME_FORMAT_SEEN)
        return TelemetryFrame(*fields[:split], crc, *fields[split:])
    return TelemetryFrame(*struct.unpack(FRAME_FORMAT + "I", data))


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
