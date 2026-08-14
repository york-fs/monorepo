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


class PrechargeState(Enum):
    LED_CHECK = 0
    PRECHECK = 1
    STANDBY = 2
    PRECHARGE = 3
    PRECHARGE_HOLD = 4
    ACTIVE = 5


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
    precharge_online: bool = attrs.field(converter=bool)

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
    return TelemetryFrame(*struct.unpack(">IBBBHHHBI", data))


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
