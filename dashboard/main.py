from collections.abc import AsyncIterable
from contextlib import asynccontextmanager
from enum import Enum, Flag
from fastapi import FastAPI, Request, Depends
from fastapi.sse import EventSourceResponse, ServerSentEvent
import asyncio
import attrs
import crcmod
import enum
import serial_asyncio
import struct


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


class RtdPreventionFlags(Flag):
    TS_NOT_ACTIVE = enum.auto()
    NOT_REQUESTED = enum.auto()
    BRAKE_NOT_PRESSED = enum.auto()


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
    PRECHECK_VOLTAGE = enum.auto()
    WAITING_DISCHARGE = enum.auto()
    WAITING_ACTIVATION = enum.auto()
    SHUTDOWN_OPEN = enum.auto()
    PRECHARGE_OPEN = enum.auto()
    AIR_POS_OPEN = enum.auto()
    AIR_NEG_OPEN = enum.auto()
    DEACTIVATION = enum.auto()
    DEVIATION = enum.auto()


class PrechargeRelayStates(Flag):
    DISCHARGE_CLOSED = enum.auto()
    PRECHARGE_CLOSED = enum.auto()
    AIR_POS_CLOSED = enum.auto()
    AIR_NEG_CLOSED = enum.auto()


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

    crc: int

    def serialize(self, inst, field, value):
        if isinstance(value, Flag):
            return [flag.name for flag in type(value) if flag in value]
        if isinstance(value, Enum):
            return value.name
        return value

    def to_dict(self):
        return attrs.asdict(self, value_serializer=self.serialize)


def parse_frame(data: bytes):
    return TelemetryFrame(*struct.unpack(">IBBIHHBBBBHHHBI", data))


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

    def __init__(self):
        self.subscribers = set()

    def subscribe(self) -> asyncio.Queue:
        queue = asyncio.Queue()
        self.subscribers.add(queue)
        return queue

    def unsubscribe(self, queue: asyncio.Queue):
        self.subscribers.discard(queue)

    def publish(self, frame: TelemetryFrame):
        for queue in self.subscribers:
            queue.put_nowait(frame.to_dict())


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
    manager = SerialManager()
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
