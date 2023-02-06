from amaranth import *
from amaranth_stdio.serial import *
from amaranth.build import *

from blackice_mx import *

roomba_pmod= [
    Resource("roomba", 0,
            Subsignal("dd",      Pins("4", dir="o", conn=("pmod",5)), Attrs(IO_STANDARD="SB_LVCMOS")))
]

class RoombaTest(Elaboratable):
    def elaborate(self, platform):

        uart    = platform.request("uart")
        #leds    = Cat([platform.request("led", i) for i in range(4)])
        clk_freq = int(platform.default_clk_frequency)
        baud = 115200
        divisor = int(clk_freq // baud)
        roomba = platform.request("roomba")
        btn = platform.request("button", 0)

        speed = 200

        start = 128
        control = 130
        move = 137

        m = Module()

        # Create the uart
        m.submodules.serial = serial = AsyncSerial(divisor=divisor, pins=uart)

        m.d.comb += [
            # Always allow reads
            serial.rx.ack.eq(1),
            # Show any errors on leds: red for parity, green for overflow, blue for frame
            #leds.eq(Cat(serial.rx.err.frame, serial.rx.err.overflow, 0b0, serial.rx.err.parity))
        ]

        cnt = Signal(28, reset=0)

        with m.FSM():
            with m.State("BUTTON"):
                with m.If(btn):
                    m.next = "BEGIN"
            with m.State("BEGIN"):
                m.d.sync += [
                    serial.tx.ack.eq(0),
                    roomba.dd.eq(0),
                    cnt.eq(cnt + 1)
                ]
                with m.If(cnt == (clk_freq // 10)):
                    m.d.sync += [
                        roomba.dd.eq(1),
                        cnt.eq(0)
                    ]
                    m.next = "WAKE"
            with m.State("WAKE"):
                m.d.sync += cnt.eq(cnt + 1)
                with m.If(cnt == (clk_freq * 2)):
                    m.d.sync += [
                        cnt.eq(0),
                        # Send start command
                        serial.tx.data.eq(start),
                        serial.tx.ack.eq(1)
                    ]
                    m.next = "START"
            with m.State("START"):
                m.d.sync += serial.tx.ack.eq(0)
                with m.If(serial.tx.rdy & ~serial.tx.ack):
                    m.d.sync += [
                        # Send control command
                        serial.tx.data.eq(control),
                        serial.tx.ack.eq(1)
                    ]
                    m.next = "CONTROL"
            with m.State("CONTROL"):
                m.d.sync += serial.tx.ack.eq(0)
                with m.If(serial.tx.rdy & ~serial.tx.ack):
                    m.d.sync += [
                        # Send move command
                        serial.tx.data.eq(move),
                        serial.tx.ack.eq(1)
                    ]
                    m.next = "FORWARD1"
            with m.State("FORWARD1"):
                m.d.sync += serial.tx.ack.eq(0)
                with m.If(serial.tx.rdy & ~serial.tx.ack):
                    m.d.sync += [
                        # Send msb of speed
                        serial.tx.data.eq(0),
                        serial.tx.ack.eq(1)
                    ]
                    m.next = "FORWARD2"
            with m.State("FORWARD2"):
                m.d.sync += serial.tx.ack.eq(0)
                with m.If(serial.tx.rdy & ~serial.tx.ack):
                    m.d.sync += [
                        # Send lsb of speed
                        serial.tx.data.eq(speed),
                        serial.tx.ack.eq(1)
                    ]
                    m.next = "FORWARD3"
            with m.State("FORWARD3"):
                m.d.sync += serial.tx.ack.eq(0)
                with m.If(serial.tx.rdy & ~serial.tx.ack):
                    m.d.sync += [
                        # Send msb of arc
                        serial.tx.data.eq(0),
                        serial.tx.ack.eq(1)
                    ]
                    m.next = "FORWARD4"
            with m.State("FORWARD4"):
                m.d.sync += serial.tx.ack.eq(0)
                with m.If(serial.tx.rdy & ~serial.tx.ack):
                    m.d.sync += [
                        # Send lsb of arc
                        serial.tx.data.eq(0),
                        serial.tx.ack.eq(1)
                    ]
                    m.next = "FORWARD"
            with m.State("FORWARD"):
                m.d.sync += [
                    serial.tx.ack.eq(0),
                    cnt.eq(cnt + 1)
                ]
                with m.If(cnt == (clk_freq * 5)):
                    m.d.sync += [
                        cnt.eq(0),
                        serial.tx.data.eq(move),
                        serial.tx.ack.eq(1)
                    ]
                    m.next = "STOP1"
            with m.State("STOP1"):
                m.d.sync += serial.tx.ack.eq(0)
                with m.If(serial.tx.rdy & ~serial.tx.ack):
                    m.d.sync += [
                        # Send msb of speed
                        serial.tx.data.eq(0),
                        serial.tx.ack.eq(1)
                    ]
                    m.next = "STOP2"
            with m.State("STOP2"):
                m.d.sync += serial.tx.ack.eq(0)
                with m.If(serial.tx.rdy & ~serial.tx.ack):
                    m.d.sync += [
                        # Send lsb of speed
                        serial.tx.data.eq(0),
                        serial.tx.ack.eq(1)
                    ]
                    m.next = "STOP3"
            with m.State("STOP3"):
                m.d.sync += serial.tx.ack.eq(0)
                with m.If(serial.tx.rdy & ~serial.tx.ack):
                    m.d.sync += [
                        # Send msb of arc
                        serial.tx.data.eq(0x80),
                        serial.tx.ack.eq(1)
                    ]
                    m.next = "STOP4"
            with m.State("STOP4"):
                m.d.sync += serial.tx.ack.eq(0)
                with m.If(serial.tx.rdy & ~serial.tx.ack):
                    m.d.sync += [
                        # Send lsb of arc
                        serial.tx.data.eq(0),
                        serial.tx.ack.eq(1)
                    ]
                    m.next = "STOP"
            with m.State("STOP"):
                m.d.sync += [
                    serial.tx.ack.eq(0)
                ]
                m.next = "BUTTON"

        return m

if __name__ == "__main__":
    platform = BlackIceMXPlatform()
    platform.add_resources(roomba_pmod)
    platform.build(RoombaTest(), do_program=True)

