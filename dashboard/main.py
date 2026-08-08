from collections.abc import AsyncIterable
from contextlib import asynccontextmanager
from fastapi import FastAPI, Request, Depends
from fastapi.sse import EventSourceResponse, ServerSentEvent
import asyncio
import attrs
import crcmod
import serial_asyncio
import struct

@attrs.define
class TelemetryFrame:
    uptime: int
    missed_tx_count: int
    minimum_voltage: int = attrs.field(converter=lambda x: x / 1000)
    crc: int


def parse_frame(data: bytes):
    return TelemetryFrame(*struct.unpack('>HBHI', data))


def cobs_unstuff(b: bytes) -> bytearray:
    out = bytearray()
    idx = 0
    while idx < len(b) and b[idx] != 0:
        out.append(0x00)
        count = b[idx]
        out.extend(b[idx + 1: idx + count])
        idx += count
    return out[1:]


stm_crc = crcmod.mkCrcFun(
    0x104c11db7,
    initCrc=0xffffffff,
    xorOut=0,
    rev=False,
)


def pad_data(data: bytes) -> bytes:
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
            queue.put_nowait(attrs.asdict(frame))


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
        frame, _, rest = self.buffer.partition(b'\x00')
        self.buffer = bytearray(rest)
        
        unstuffed = cobs_unstuff(frame)
        if stm_crc(pad_data(unstuffed)) == 0:
            self.manager.publish(parse_frame(unstuffed))
        else:
            print('Bad CRC!')
        
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
        '/dev/ttyUSB0',
        baudrate=115200,
    )
    yield
    transport.close()


app = FastAPI(lifespan=lifespan)


def get_serial_manager(request: Request) -> SerialManager:
    return request.app.state.serial_manager


@app.get('/api/stream', response_class=EventSourceResponse)
async def stream(manager: SerialManager = Depends(get_serial_manager)) -> AsyncIterable[ServerSentEvent]:
    queue = manager.subscribe()
    try:
        while True:
            try:
                frame = await asyncio.wait_for(queue.get(), timeout=30)
                yield ServerSentEvent(data=frame)
            except asyncio.TimeoutError:
                yield ServerSentEvent(comment="keep-alive")
    finally:
        manager.unsubscribe(queue)
