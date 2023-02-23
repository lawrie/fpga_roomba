from amaranth import *
from amaranth_stdio.serial import *
from amaranth.build import *

from amaranth.lib.cdc import FFSynchronizer

from blackice_mx import *

from debouncer import Debouncer

import math

# Connect the Roomba Device Detect pin
roomba_pmod= [
    Resource("roomba", 0,
            Subsignal("dd",      Pins("10", dir="o", conn=("pmod",5)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("rx",      Pins("9", dir="i", conn=("pmod",5)), Attrs(IO_STANDARD="SB_LVCMOS")))
]

# iCEBreaker Pmod used for extra buttons and Leds
breaker_pmod= [
    Resource("breaker", 0,
            Subsignal("led1",      Pins("7", dir="o", conn=("pmod",0)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("led2",      Pins("1", dir="o", conn=("pmod",0)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("led3",      Pins("2", dir="o", conn=("pmod",0)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("led4",      Pins("8", dir="o", conn=("pmod",0)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("led5",      Pins("3", dir="o", conn=("pmod",0)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("btn1",      Pins("9", dir="i", conn=("pmod",0)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("btn2",      Pins("4", dir="i", conn=("pmod",0)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("btn3",      Pins("10", dir="i", conn=("pmod",0)), Attrs(IO_STANDARD="SB_LVCMOS")))
]

# Digilent 8LED Pmod
pmod_led8_1 = [
    Resource("led8_1", 0,
        Subsignal("leds", Pins("1 2 3 4 7 8 9 10", dir="o", conn=("pmod",2))),
        Attrs(IO_STANDARD="SB_LVCMOS"))
]

pmod_led8_2 = [
    Resource("led8_2", 0,
        Subsignal("leds", Pins("1 2 3 4 7 8 9 10", dir="o", conn=("pmod",3))),
        Attrs(IO_STANDARD="SB_LVCMOS"))
]


# Pmod for HM-10 BLE device
pmod_bt = [
    Resource("bt", 0,
        Subsignal("rx", Pins("8", dir="i", conn=("pmod",1))),
        Attrs(IO_STANDARD="SB_LVCMOS"))
]

angles = [math.radians(0.4 + (x * 0.8)) for x in range(450)]
sin = [int(math.sin(x) * 128) for x in angles]
cos = [int(math.cos(x) * 128) for x in angles]

class RoombaTest(Elaboratable):
    """ Drives a Roomba vacuum cleaner robot via its serial interface """
    def elaborate(self, platform):

        # Pins
        uart    = platform.request("uart")
        leds    = Cat([platform.request("led", i) for i in range(2,4)])
        roomba = platform.request("roomba")
        breaker = platform.request("breaker")
        btn1 = platform.request("button", 0)
        btn2 = platform.request("button", 1)
        btn3 = breaker.btn1
        btn4 = breaker.btn2
        btn5 = breaker.btn3
        led8_1 = platform.request("led8_1")
        led8_2 = platform.request("led8_2")
        led16 = Cat(led8_1, led8_2)
        hm10 = platform.request("bt") # HM-10 Bluetooth device

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
        init_speed = 200
        init_turn_time = int((0.7 + (320 / init_speed)) * 1000)
        init_forward_time = int((1000 / init_speed) * 1000)
        print("Turn time: ", init_turn_time)
        print("Forward time: ", init_forward_time)
        wake_time = 2
        wait_time = 1
        dd_time = 0.1

        m = Module()

        # Create the uart
        m.submodules.serial = serial = AsyncSerial(divisor=divisor, pins=uart)

        # Always allow reads
        m.d.comb += serial.rx.ack.eq(1)

        # Create the bluetooth uart
        bt_divisor = int(clk_freq // 9600)
        m.submodules.bt = bt = AsyncSerial(divisor=bt_divisor)

        # Always allow reads
        m.d.comb += bt.rx.ack.eq(1)

        # Connect the RX pin
        m.submodules += FFSynchronizer(hm10.rx, bt.rx.i, reset=1)

        # Create uart for LDRobot LD19
        ld19_divisor = int(clk_freq // 230400)
        m.submodules.ld19 = ld19 = AsyncSerial(divisor=ld19_divisor)
        
        # Always allow reads
        m.d.comb += ld19.rx.ack.eq(1)

        # Connect the RX pin
        m.submodules += FFSynchronizer(roomba.rx, ld19.rx.i, reset=1)

        # Signals
        dd = Signal(reset=1)
        cnt = Signal(28, reset=0)
        cmd = Array(Signal(8) for _ in range(35))
        l = Signal(6)
        j = Signal.like(l)
        k = Signal.like(l)
        num_notes = Signal(5)
        sending = Signal(reset=0)
        sensor = Signal(80)
        i = Signal(4)
        speed = Signal(16, reset=init_speed)
        turn_time = Signal(16, reset=init_turn_time)
        forward_time = Signal(16, reset=init_forward_time)
        millis = Signal(16)
        ii = Signal(6)
        lidar = Signal(8 * 47)
        start_angle = Signal(16, reset=0xffff)
        distance = Signal(16)
        last_byte = Signal(8)
        index = Signal(6)
        points = Array(Signal(24) for _ in range(450))
        intensity = Signal(8)
        x = Signal(8)
        y = Signal(8)

        # Set device detect pin
        m.d.comb += roomba.dd.eq(dd)

        # Song definition
        song_bytes = [67, 16, 67, 16, 67, 16, 64, 64]

        # Set leds to to bump sensors
        #m.d.comb += [
        #    breaker.led4.eq(sensor[0]),
        #    breaker.led5.eq(sensor[1])
        #]

        # Create debouncers for the buttons
        m.submodules.deb1 = deb1 = Debouncer()
        m.submodules.deb2 = deb2 = Debouncer()
        m.submodules.deb3 = deb3 = Debouncer()
        m.submodules.deb4 = deb4 = Debouncer()
        m.submodules.deb5 = deb5 = Debouncer()

        # Connect buttons to the debouncers
        m.d.comb += [
            deb1.btn.eq(btn1),
            deb2.btn.eq(btn2),
            deb3.btn.eq(btn3),
            deb4.btn.eq(btn4),
            deb5.btn.eq(btn5)
        ]

        # Create a ROM to do some commands
        rom = [drive, 0, 200, 0x80, 0, 
               0, 0, 20,
               drive, 0, 200, 0, 1, 
               0, 0, 9, 
               drive, 0, 200, 0x80, 0,
               0, 0, 20,
               drive, 0, 200, 0, 1,
               0, 0, 9,
               drive, 0, 0, 0x80, 0
              ]

        mem = Memory(width=8, depth=len(rom), init=rom)
        m.submodules.r = r = mem.read_port()

        # Address needs to go one higher than r.addr
        addr = Signal(range(len(rom) + 1))
        m.d.comb += r.addr.eq(addr)

        # Create a table of sines
        sin_mem = Memory(width=8, depth=450, init=sin)
        m.submodules.sin_r = sin_r = sin_mem.read_port()

        # Create a table of cosines
        cos_mem = Memory(width=8, depth=450, init=cos)
        m.submodules.cos_r = cos_r = cos_mem.read_port()

        # Memory for distance measurements
        dist_mem = Memory(width=16, depth=450)
        m.submodules.dist_r = dist_r = dist_mem.read_port()
        m.submodules.dist_w = dist_w = dist_mem.write_port()

        # Memory for intensity measurements
        int_mem = Memory(width=8, depth=450)
        m.submodules.int_r = int_r = int_mem.read_port()
        m.submodules.int_w = int_w = int_mem.write_port()

        # Functions to send commands
        def send(c):
            m.d.sync += [
                cnt.eq(0),
                cmd[0].eq(c),
                sending.eq(1)
            ]

        def do_drive(sp, rad):
            send(drive)
            m.d.sync += [
                cmd[1].eq(sp[8:]),
                cmd[2].eq(sp[:8]),
                cmd[3].eq(rad[8:]),
                cmd[4].eq(rad[:8])
            ]

        def forward():
            do_drive(speed, C(0x8000, 16))

        def backward():
            do_drive(-speed, C(0x8000, 16))

        def stop():
            do_drive(C(0,16), C(0x8000, 16))

        def spin_left():
            do_drive(speed, C(0x0001, 16))

        def spin_right():
            do_drive(speed, C(0xFFFF, 16))

        # Set signal l to the number of data bytes for current command
        def set_l(c):
            with m.If(c == drive):
                m.d.sync += l.eq(4)
            with m.Elif((c == read_sensors) | (c == play) |
                        (c == set_baud) | (c == motors)):
                m.d.sync += l.eq(1)
            with m.Elif(c == set_leds):
                m.d.sync += l.eq(3)
            with m.Elif(c == song):
                m.d.sync += l.eq((num_notes << 1) + 2)
            with m.Elif(c[7] == 0):
                m.d.sync += l.eq(2)
            with m.Else():
                m.d.sync += l.eq(0)

        def play_song(n):
            send(play)
            m.d.sync += cmd[1].eq(n)

        # Get inyensity for current x and y co-ordinates
        with m.If((points[0][:8] == x) & (points[0][8:16] == y)):
            m.d.sync += intensity.eq(points[0][16:])

        for p in range(1, 450):
            with m.If((points[p][:8] == x) & (points[p][8:16] == y)):
                m.d.sync += intensity.eq(points[p][16:])

        with m.Else():
            intensity.eq(0)

        # Check disatance to obstacle
        with m.If(ld19.rx.rdy & (ii == 46)): # Read checksum
            with m.If(lidar[32:48] < 0x0080): # First frame
                # Set frame index to zero
                m.d.sync += index.eq(0)
                m.d.sync += led16.eq(lidar[48:64])
                m.d.sync += distance.eq(lidar[48:64])
                # Stop if obstacle closer than about 25cm
                with m.If(lidar[48:64] < 0x100):
                    stop()
            with m.Else():
                m.d.sync += index.eq(index + 1)

        # Control state machine
        with m.FSM():
            with m.State("BEGIN"):
                m.d.sync += [
                    # Set device detect low to wake-up Roomba
                    serial.tx.ack.eq(0),
                    dd.eq(0),
                    cnt.eq(cnt + 1)
                ]
                with m.If(cnt == int(clk_freq * dd_time)):
                    # Set device detect high
                    m.d.sync += [
                        dd.eq(1),
                        cnt.eq(0)
                    ]
                    m.next = "WAKE"
            with m.State("WAKE"):
                m.d.sync += cnt.eq(cnt + 1)
                with m.If(cnt == int(clk_freq * wake_time)):
                    # Send start command
                    send(start)
                    m.next = "START"
            with m.State("START"):
                with m.If(~sending):
                    # Send control command
                    send(control)
                    m.next = "FULL"
            with m.State("CONTROL"):
                with m.If(~sending):
                    # Send full command
                    send(full)
                    m.next = "FULL"
            with m.State("FULL"):
                m.d.sync += cnt.eq(cnt + 1)
                with m.If(cnt == int(clk_freq * wait_time)):
                    m.next = "SONG"
            with m.State("SONG"):
                send(song)
                m.d.sync += [
                    num_notes.eq(len(song_bytes) // 2),
                    cmd[1].eq(0), # Song 0
                    cmd[2].eq(len(song_bytes) // 2)
                ]
                for n in range(len(song_bytes)):
                  m.d.sync += cmd[n+3].eq(song_bytes[n])
                m.next = "PLAY"
            with m.State("BUTTON"):
                with m.If(deb1.btn_up):
                    m.next = "MOVE"
                with m.If(deb2.btn_up):
                    m.next = "SLEEP"
                with m.If(deb3.btn_up):
                    m.next = "SENSORS"
                with m.If(deb4.btn_up):
                    m.next = "SONG"
                with m.If(deb5.btn_up):
                    m.next = "ROM"
            with m.State("MOVE"):
                # Send drive command
                forward()
                m.next = "FORWARD"
            with m.State("FORWARD"):
                m.d.sync += cnt.eq(cnt + 1)
                # Wait 5 seconds
                with m.If(cnt == int((clk_freq // 1000)) * forward_time):
                    # Send spinleft
                    spin_left()
                    m.next = "SPINLEFT"
            with m.State("SPINLEFT"):
                m.d.sync += cnt.eq(cnt + 1)
                # Wait 5 seconds
                with m.If(cnt == int((clk_freq // 1000)) * turn_time):
                    # Send forward
                    forward()
                    m.next = "FORWARD2"
            with m.State("FORWARD2"):
                m.d.sync += cnt.eq(cnt + 1)
                # Wait 5 seconds
                with m.If(cnt == int(clk_freq // 1000) * forward_time):
                    # Send spinleft
                    spin_left()
                    m.next = "SPINLEFT2"
            with m.State("SPINLEFT2"):
                m.d.sync += cnt.eq(cnt + 1)
                # Wait 5 seconds
                with m.If(cnt == int(clk_freq // 1000) * turn_time):
                    # Send stop
                    stop()
                    m.next = "STOP"
            with m.State("STOP"):
                with m.If(~sending):
                    m.next = "BUTTON"
            with m.State("DOCK"):
                # Send dock
                send(dock)
                m.next = "BUTTON"
            with m.State("SENSORS"):
                send(read_sensors)
                m.d.sync += [
                    cmd[1].eq(1), # read 10 bytes
                    leds[0].eq(1)
                ]
                m.next = "WAIT"   
            with m.State("WAIT"):
                m.d.sync += cnt.eq(cnt + 1)
                with m.If(cnt == int(clk_freq * 0.5)):
                    m.d.sync += cnt.eq(0)
                    m.next = "BUTTON"
            with m.State("SLEEP"):
                send(sleep)
                m.next = "WAIT_UART"
            with m.State("PLAY"):
                with m.If(~sending):
                    play_song(0)
                    m.next = "WAIT_UART"
            with m.State("WAIT_UART"):
                with m.If(~sending):
                    m.next = "BUTTON"
            with m.State("LEDS"):
                send(set_leds)
                m.d.sync += [
                    breaker.led5.eq(1),
                    cmd[1].eq(1), # ledbits, dirt detected
                    cmd[2].eq(0), # power color, green
                    cmd[3].eq(128) # power intensity
                ]
                m.next = "WAIT_UART"
            with m.State("ROM"):
                with m.If(addr < len(rom)):
                    m.d.sync += [
                        cmd[0].eq(r.data),
                        k.eq(0),
                        addr.eq(addr + 1)
                    ]
                    set_l(r.data)
                    m.next = "PARAM0"
                with m.Else():
                    m.d.sync += addr.eq(0)
                    m.next = "BUTTON"
            with m.State("PARAM0"):
                m.next = "PARAM"
            with m.State("PARAM"):
                with m.If(k == l):
                    with m.If(cmd[0][7]):
                        m.d.sync += sending.eq(1)
                        m.next = "EXEC"
                    with m.Else():
                        m.d.sync += [
                            millis.eq(Cat(cmd[1], cmd[2])),
                            cnt.eq(0)
                        ]
                        m.next = "COUNTDOWN"
                with m.Else():
                    m.d.sync += [
                        k.eq(k + 1),
                        cmd[k+1].eq(r.data),
                        addr.eq(addr + 1)
                    ]
                    m.next = "PARAM0"
            with m.State("EXEC"):
                with m.If(~sending):
                    m.next = "ROM"
            with m.State("COUNTDOWN"):
                m.d.sync += cnt.eq(cnt + 1)
                with m.If(cnt == int(clk_freq // 1000)):
                    m.d.sync += [
                        millis.eq(millis - 1),
                        cnt.eq(0)
                    ]
                    with m.If(millis == 1):
                        m.next = "ROM"

        # Receive Bluetooth commands
        with m.FSM():
            with m.State("IDLE"):
                with m.If(bt.rx.rdy):
                    with m.Switch(bt.rx.data):
                        with m.Case(ord('f')):
                            forward()
                            m.next = "WAIT"
                        with m.Case(ord('b')):
                            backward()
                            m.next = "WAIT"
                        with m.Case(ord('r')):
                            spin_right()
                            m.next = "WAIT"
                        with m.Case(ord('l')):
                            spin_left()
                            m.next = "WAIT"
                        with m.Case(ord('s')):
                            stop()
                            m.next = "WAIT"
                        with m.Case(ord('p')):
                            play_song(0)
                            m.next = "WAIT"
            with m.State("WAIT"):
                with m.If(~sending):
                    m.next = "IDLE"

        # Send command state machine
        with m.FSM():
            with m.State("IDLE"):
                with m.If(sending):
                    m.d.sync += [
                        # Send command byte
                        j.eq(0),
                        serial.tx.data.eq(cmd[0]),
                        serial.tx.ack.eq(1)
                    ]
                    # Set l to number of parameter bytes for the command
                    set_l(cmd[0])
                    m.next = "SEND"
            with m.State("SEND"):
                m.d.sync += serial.tx.ack.eq(0)
                with m.If(serial.tx.rdy & ~serial.tx.ack):
                    with m.If(j == l):
                        m.d.sync += sending.eq(0)
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += [
                            # Send parameter byte
                            j.eq(j + 1),
                            serial.tx.data.eq(cmd[j + 1]),
                            serial.tx.ack.eq(1)
                        ]

        # Read sensor data - 10 byte packets
        with m.If(serial.rx.rdy):
            m.d.sync += leds[1].eq(1)
            m.d.sync += sensor.word_select(i, 8).eq(serial.rx.data)
            with m.If(i == 9):
                m.d.sync += i.eq(0)
            with m.Else():
                m.d.sync += i.eq(i + 1)

        # Read lidar data
        with m.If(ld19.rx.rdy):
            m.d.sync += last_byte.eq(ld19.rx.data)
            m.d.sync += lidar.word_select(ii, 8).eq(ld19.rx.data)
            with m.If(ii == 46):
                m.d.sync += ii.eq(0)
            with m.Elif((ld19.rx.data == 0x2c) & (last_byte == 0x54)):
                m.d.sync += ii.eq(2)
                m.d.sync += lidar[:8].eq(0x54)
                m.d.sync += breaker.led1.eq(~breaker.led1)
            with m.Else():
                m.d.sync += ii.eq(ii + 1)

        return m

if __name__ == "__main__":
    platform = BlackIceMXPlatform()
    platform.add_resources(roomba_pmod)
    platform.add_resources(breaker_pmod)
    platform.add_resources(pmod_led8_1)
    platform.add_resources(pmod_led8_2)
    platform.add_resources(pmod_bt)
    platform.build(RoombaTest(), do_program=True)
