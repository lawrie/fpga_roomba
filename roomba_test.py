from amaranth import *
from amaranth_stdio.serial import *
from amaranth.build import *

from blackice_mx import *

from debouncer import Debouncer

roomba_pmod= [
    Resource("roomba", 0,
            Subsignal("dd",      Pins("4", dir="o", conn=("pmod",5)), Attrs(IO_STANDARD="SB_LVCMOS")))
]

class RoombaTest(Elaboratable):
    def elaborate(self, platform):

        # Pins
        uart    = platform.request("uart")
        leds    = Cat([platform.request("led", i) for i in range(2,4)])
        roomba = platform.request("roomba")
        btn1 = platform.request("button", 0)
        btn2 = platform.request("button", 1)

        # Uart parameters
        clk_freq = int(platform.default_clk_frequency)
        baud = 115200
        divisor = int(clk_freq // baud)

        # Roomba commands
        start = 128
        set_baud = 129
        control = 130
        safe = 131
        full = 132
        sleep = 133
        spot = 134
        clean = 135
        do_max = 136
        drive = 137
        motors = 138
        set_leds = 139
        song = 140
        play = 141
        read_sensors = 142
        dock = 143

        # Parameters and times
        speed = 200
        turn_time = 2.3
        forward_time = 5
        wake_time = 2
        wait_time = 1

        m = Module()

        # Create the uart
        m.submodules.serial = serial = AsyncSerial(divisor=divisor, pins=uart)

        # Always allow reads
        m.d.comb += serial.rx.ack.eq(1)

        # Signals
        cnt = Signal(28, reset=0)
        cmd = Signal(40)
        l = Signal(3)
        sending = Signal(reset=0)
        sensor = Signal(40)
        i = Signal(4)

        # Set leds to to bump sensors
        #m.d.comb += [
        #    leds[0].eq(sensor[0]),
        #    leds[1].eq(sensor[1])
        #]

        # Create debouncers for the buttons
        m.submodules.deb1 = deb1 = Debouncer()
        m.submodules.deb2 = deb2 = Debouncer()

        m.d.comb += [
            deb1.btn.eq(btn1),
            deb2.btn.eq(btn2)
        ]

        # Control state machine
        with m.FSM():
            with m.State("BUTTON"):
                #m.d.sync += [
                #    leds[0].eq(0),
                #    leds[1].eq(0)
                #]
                with m.If(deb1.btn_up):
                    m.next = "BEGIN"
                with m.If(deb2.btn_up):
                    m.next = "SENSORS"
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
                        sending.eq(1)
                    ]
                    m.next = "START"
            with m.State("START"):
                with m.If(~sending):
                    m.d.sync += [
                        # Send control command
                        cmd[:8].eq(control),
                        sending.eq(1)
                    ]
                    m.next = "CONTROL"
            with m.State("CONTROL"):
                with m.If(~sending):
                    m.d.sync += [
                        # Send full command
                        cmd[:8].eq(full),
                        sending.eq(1)
                    ]
                    m.next = "FULL"
            with m.State("FULL"):
                m.d.sync += cnt.eq(cnt + 1)
                with m.If(cnt == (clk_freq * wait_time)):
                    m.d.sync += [
                        cnt.eq(0),
                        # Send drive command
                        cmd[:8].eq(drive),
                        cmd[8:16].eq(0),
                        cmd[16:24].eq(speed),
                        cmd[24:32].eq(0x80),
                        cmd[32:].eq(0),
                        sending.eq(1)
                    ]
                    m.next = "FORWARD"
            with m.State("FORWARD"):
                m.d.sync += [
                    cnt.eq(cnt + 1),
                    #leds[0].eq(1)
                ]
                # Wait 5 seconds
                with m.If(cnt == (clk_freq * forward_time)):
                    m.d.sync += [
                        # Send spinleft
                        cnt.eq(0),
                        cmd[:8].eq(drive),
                        cmd[8:16].eq(0),
                        cmd[16:24].eq(speed),
                        cmd[24:32].eq(0),
                        cmd[32:].eq(1),
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
                        cmd[:8].eq(drive),
                        cmd[8:16].eq(0),
                        cmd[16:24].eq(speed),
                        cmd[24:32].eq(0x80),
                        cmd[32:].eq(1),
                        sending.eq(1)
                    ]
                    m.next = "FORWARD2"
            with m.State("FORWARD2"):
                m.d.sync += [
                    cnt.eq(cnt + 1),
                    #leds[1].eq(1)
                ]
                # Wait 5 seconds
                with m.If(cnt == (clk_freq * 5)):
                    m.d.sync += [
                        # Send spinleft
                        cnt.eq(0),
                        cmd[:8].eq(drive),
                        cmd[8:16].eq(0),
                        cmd[16:24].eq(speed),
                        cmd[24:32].eq(0),
                        cmd[32:].eq(1),
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
                        cmd[:8].eq(drive),
                        cmd[8:16].eq(0),
                        cmd[16:24].eq(0),
                        cmd[24:32].eq(0x80),
                        cmd[32:].eq(1),
                        sending.eq(1)
                    ]
                    m.next = "STOP"
            with m.State("STOP"):
                with m.If(~sending):
                    m.next = "BUTTON"
            with m.State("DOCK"):
                m.d.sync += [
                    # Send dock
                    cmd[:8].eq(dock),
                    sending.eq(1)
                ]
                m.next = "BUTTON"
            with m.State("SENSORS"):
                m.d.sync += [
                    cmd[:8].eq(read_sensors),
                    cmd[8:16].eq(1), # read 10 bytes
                    sending.eq(1),
                    leds[0].eq(1)
                ]
                m.next = "SENSOR_WAIT"   
            with m.State("SENSOR_WAIT"):
                m.d.sync += cnt.eq(cnt + 1)
                with m.If(cnt == int(clk_freq * 0.5)):
                    m.d.sync += cnt.eq(0)
                    m.next = "BUTTON"

        # Send command state machine
        with m.FSM():
            with m.State("IDLE"):
                with m.If(sending):
                    m.d.sync += [
                        # Send command byte
                        l.eq(0),
                        serial.tx.data.eq(cmd[:8]),
                        serial.tx.ack.eq(1)
                    ]
                    # Drive command takes 4 bytes of parameters
                    with m.If(cmd[:8] == drive):
                        m.d.sync += l.eq(4)
                    with m.Elif((cmd[:8] == read_sensors) | (cmd[:8] == play) |
                                (cmd[:8] == set_baud) | (cmd[:8] == motors)):
                        m.d.sync += l.eq(1)
                    with m.Elif(cmd[:8] == set_leds):
                        m.d.sync += l.eq(3)
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

        # Read sensor data
        with m.If(serial.rx.rdy):
            m.d.sync += leds[1].eq(1)
            m.d.sync += sensor.word_select(i, 8).eq(serial.rx.data)
            with m.If(i == 9):
                m.d.sync += i.eq(0)
            with m.Else():
                m.d.sync += i.eq(i + 1)

        return m

if __name__ == "__main__":
    platform = BlackIceMXPlatform()
    platform.add_resources(roomba_pmod)
    platform.build(RoombaTest(), do_program=True)

