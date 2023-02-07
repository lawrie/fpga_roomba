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
        leds    = Cat([platform.request("led", i) for i in range(2,4)])
        clk_freq = int(platform.default_clk_frequency)
        baud = 115200
        divisor = int(clk_freq // baud)
        roomba = platform.request("roomba")
        btn = platform.request("button", 0)

        speed = 200
        start = 128
        control = 130
        move = 137
        turn_time = 2.5
        forward_time = 5
        wake_time = 2
        wait_time = 1

        m = Module()

        # Create the uart
        m.submodules.serial = serial = AsyncSerial(divisor=divisor, pins=uart)

        m.d.comb += [
            # Always allow reads
            serial.rx.ack.eq(1),
        ]

        cnt = Signal(28, reset=0)
        cmd = Signal(40)
        l = Signal(3)
        sending = Signal(reset=0)

        with m.FSM():
            with m.State("BUTTON"):
                m.d.sync += [
                    leds[0].eq(0),
                    leds[1].eq(0)
                ]
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
                with m.If(cnt == (clk_freq * wake_time)):
                    m.d.sync += [
                        cnt.eq(0),
                        # Send start command
                        cmd[:8].eq(start),
                        l.eq(0),
                        sending.eq(1)
                    ]
                    m.next = "START"
            with m.State("START"):
                with m.If(~sending):
                    m.d.sync += [
                        # Send control command
                        cmd[:8].eq(control),
                        l.eq(0),
                        sending.eq(1)
                    ]
                    m.next = "CONTROL"
            with m.State("CONTROL"):
                m.d.sync += cnt.eq(cnt + 1)
                with m.If(cnt == (clk_freq * wait_time)):
                    m.d.sync += [
                        cnt.eq(0),
                        # Send move command
                        cmd[:8].eq(move),
                        cmd[8:16].eq(0),
                        cmd[16:24].eq(speed),
                        cmd[24:32].eq(0x80),
                        cmd[32:].eq(0),
                        l.eq(4),
                        sending.eq(1)
                    ]
                    m.next = "FORWARD"
            with m.State("FORWARD"):
                m.d.sync += [
                    cnt.eq(cnt + 1),
                    leds[0].eq(1)
                ]
                # Wait 5 seconds
                with m.If(cnt == (clk_freq * forward_time)):
                    m.d.sync += [
                        # Send spinleft
                        cnt.eq(0),
                        cmd[:8].eq(move),
                        cmd[8:16].eq(0),
                        cmd[16:24].eq(speed),
                        cmd[24:32].eq(0),
                        cmd[32:].eq(1),
                        l.eq(4),
                        sending.eq(1)
                    ]
                    m.next = "SPINLEFT"
            with m.State("SPINLEFT"):
                m.d.sync += cnt.eq(cnt + 1)
                # Wait 5 seconds
                with m.If(cnt == int(clk_freq * turn_time)):
                    m.d.sync += [
                        # Send forward
                        cnt.eq(0),
                        cmd[:8].eq(move),
                        cmd[8:16].eq(0),
                        cmd[16:24].eq(speed),
                        cmd[24:32].eq(0x80),
                        cmd[32:].eq(1),
                        l.eq(4),
                        sending.eq(1)
                    ]
                    m.next = "FORWARD2"
            with m.State("FORWARD2"):
                m.d.sync += [
                    cnt.eq(cnt + 1),
                    leds[1].eq(1)
                ]
                # Wait 5 seconds
                with m.If(cnt == (clk_freq * 5)):
                    m.d.sync += [
                        # Send spinleft
                        cnt.eq(0),
                        cmd[:8].eq(move),
                        cmd[8:16].eq(0),
                        cmd[16:24].eq(speed),
                        cmd[24:32].eq(0),
                        cmd[32:].eq(1),
                        l.eq(4),
                        sending.eq(1)
                    ]
                    m.next = "SPINLEFT2"
            with m.State("SPINLEFT2"):
                m.d.sync += cnt.eq(cnt + 1)
                # Wait 5 seconds
                with m.If(cnt == int(clk_freq * turn_time)):
                    m.d.sync += [
                        # Send stop
                        cnt.eq(0),
                        cmd[:8].eq(move),
                        cmd[8:16].eq(0),
                        cmd[16:24].eq(0),
                        cmd[24:32].eq(0x80),
                        cmd[32:].eq(1),
                        l.eq(4),
                        sending.eq(1)
                    ]
                    m.next = "STOP"
            with m.State("STOP"):
                with m.If(~sending):
                    m.next = "BUTTON"

        with m.FSM():
            with m.State("IDLE"):
                with m.If(sending):
                    m.d.sync += [
                        # Send command byte
                        l.eq(0),
                        serial.tx.data.eq(cmd[:8]),
                        serial.tx.ack.eq(1)
                    ]
                    # Move command takes 4 bytes of parameters
                    with m.If(cmd[:8] == 137):
                        m.d.sync += l.eq(4)
                    m.next = "SEND"
            with m.State("SEND"):
                m.d.sync += serial.tx.ack.eq(0)
                with m.If(serial.tx.rdy & ~serial.tx.ack):
                    with m.If(l == 0):
                        m.d.sync += sending.eq(0)
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += [
                            # Send parameter byte
                            l.eq(l - 1),
                            cmd.eq(cmd[8:]),
                            serial.tx.data.eq(cmd[8:16]),
                            serial.tx.ack.eq(1)
                        ]

        return m

if __name__ == "__main__":
    platform = BlackIceMXPlatform()
    platform.add_resources(roomba_pmod)
    platform.build(RoombaTest(), do_program=True)

