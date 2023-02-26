from amaranth import *
from amaranth_stdio.serial import *
from amaranth.build import *

from amaranth.lib.cdc import FFSynchronizer

from blackice_mx import *

from debouncer import Debouncer
from st7789 import ST7789

import math

# Connect the Roomba Device Detect pin, and the Lidar TX pin to unused Pmod slot
roomba_pmod= [
    Resource("roomba", 0,
            Subsignal("dd",      Pins("10", dir="o", conn=("pmod",4)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("rx",      Pins("9", dir="i", conn=("pmod",4)), Attrs(IO_STANDARD="SB_LVCMOS")))
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

# Digilent 8LED Pmods x 2
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

# Pmod for ST7789 LCD
oled_pmod = [
    Resource("oled", 0,
            Subsignal("oled_clk", Pins("7", dir="o", conn=("pmod",5)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("oled_mosi", Pins("8", dir="o", conn=("pmod",5)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("oled_resn", Pins("3", dir="o", conn=("pmod",5)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("oled_dc", Pins("1", dir="o", conn=("pmod",5)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("oled_csn", Pins("2", dir="o", conn=("pmod",5)), Attrs(IO_STANDARD="SB_LVCMOS")))
]

# Generate sine and cosine tables for angles frrom 0 to 360 degrees in 0.8 degree steps
angles = [math.radians(x * 0.8) for x in range(450)]
sin = [int(math.sin(x) * 128) + 128 for x in angles]
cos = [int(math.cos(x) * 128) + 128 for x in angles]

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

        # LCD/OLED Pmod
        oled  = platform.request("oled")
        oled_clk  = oled.oled_clk
        oled_mosi = oled.oled_mosi
        oled_dc   = oled.oled_dc
        oled_resn = oled.oled_resn
        oled_csn  = oled.oled_csn

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
        wake_time = 2
        wait_time = 1
        dd_time = 0.1

        # Start of elaboratin
        m = Module()

        # Create ST7789 LCD module and connect pins
        st7789 = ST7789(reset_delay=100000,reset_period=100000)
        m.submodules.st7789 = st7789

        m.d.comb += [
            oled_clk .eq(st7789.spi_clk),
            oled_mosi.eq(st7789.spi_mosi),
            oled_dc  .eq(st7789.spi_dc),
            oled_resn.eq(st7789.spi_resn),
            oled_csn .eq(st7789.spi_csn)
        ]

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
        dd = Signal(reset=1) # Device detect
        cnt = Signal(28, reset=0) # Time counter
        cmd = Array(Signal(8) for _ in range(35)) # Current roomba command
        l = Signal(6) # Command length
        num_notes = Signal(5) # Number of notes in song
        sending = Signal(reset=0) # Set when sending bytes to Roomba
        sensor = Signal(80) # Sensor data
        speed = Signal(16, reset=init_speed) # Forward and turn speed
        turn_time = Signal(16, reset=init_turn_time) # Roomba speed in mm/s
        forward_time = Signal(16, reset=init_forward_time) # Millisecond for forward
        millis = Signal(16) # Millisecond counter
        lidar = Signal(8 * 47) # Lidar data
        distance = Signal(16) # Current distance from frame
        intensity = Signal(8) # Current intensity from frame
        last_byte = Signal(8) # Last lidar byte read
        ci = Signal.like(l) # Command index
        ri = Signal.like(l) # Rom command index
        si = Signal(4) # Index for sensor data
        li = Signal(6) # Lidar index of byte within frame
        fi = Signal(6) # Frame index of data frames per scan
        ai = Signal(9) # Angle index, 0 to 449
        pi = Signal(2) # Point index for byte within point data
        pp = Signal(4) # Pointer to current point in frame
        sc = Signal(3) # Scan counter
        x = Signal(signed(24)) # x-cordinate in signed 16.8 format
        y = Signal(signed(24)) # y-coordinate in signed 16.8 format
        cos_s = Signal(signed(8)) # Signed current cosine in 0.8 format
        sin_s = Signal(signed(8)) # Signed current sine in 0.8 format
        lx = Signal(8) # LCD screen x-coordinate
        ly = Signal(8) # LCD screen y-coordinate
        angle = Signal(16) # Current angle in 100s of a degree
        start_angle = Signal(16) # Start of frame angle in 1/100s of a degree
        min_angle = Signal(16, reset=0xFFFF) # Minimum start of frame angle in 1/100s
        max_angle = Signal(16, reset=0x0000) # Maximum start of frame angle in 1/100s

        # Connect Roomba device detect pin
        m.d.comb += roomba.dd.eq(dd)

        # Song definition
        song_bytes = [67, 16, 67, 16, 67, 16, 64, 64]

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
        rom = [drive, 0, 200, 0x80, 0,  # Drive forward 
               0, 0, 20,                # Wait 20 x 256 (5k) miilseconds
               drive, 0, 200, 0, 1,     # Spin left
               0, 0, 9,                 # Wait 9 x 256 (about 2300) milliseconds
               drive, 0, 200, 0x80, 0,  # Drive forward
               0, 0, 20,                # Wait about 5 seconds
               drive, 0, 200, 0, 1,     # Spin left
               0, 0, 9,                 # Wait 2.3 seconds
               drive, 0, 0, 0x80, 0     # Stop (drive at zero speed)
              ]

        # Memory for ROM
        mem = Memory(width=8, depth=len(rom), init=rom)
        m.submodules.r = r = mem.read_port()

        # Address needs to go one higher than r.addr
        addr = Signal(range(len(rom) + 1))
        m.d.comb += r.addr.eq(addr)

        # Create a table of sines
        sin_mem = Memory(width=8, depth=450, init=sin)
        m.submodules.sin_r = sin_r = sin_mem.read_port()

        # Set sin_s to the signed value
        m.d.comb += sin_r.addr.eq(ai)
        m.d.comb += sin_s.eq(sin_r.data - 128)

        # Create a table of cosines
        cos_mem = Memory(width=8, depth=450, init=cos)
        m.submodules.cos_r = cos_r = cos_mem.read_port()

        # Set cos_s to the signed value
        m.d.comb += cos_r.addr.eq(ai)
        m.d.comb += cos_s.eq(cos_r.data - 128)

        # Memory for LCD buffer
        lcd_mem = Memory(width=1, depth=240 * 240)
        m.submodules.lcd_r = lcd_r = lcd_mem.read_port()
        m.submodules.lcd_w = lcd_w = lcd_mem.write_port()

        m.d.comb += [
            # Set addresses to write to and read from the LCD memory
            lcd_w.addr.eq((ly * 240) + lx),
            lcd_r.addr.eq((st7789.x * 240) + st7789.y),
            # Draw robot in centre in blue, and draw map in red
            st7789.color.eq(Mux(((st7789.y == 127) | (st7789.y == 128)) & 
                                ((st7789.x == 127) | (st7789.x == 128)), 
                                  0x001F, Mux(lcd_r.data, 0xF800, 0x0000)))
        ]

        # Functions to send commands to Roomba
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

        # Play a previously defined song on Roomba
        def play_song(n):
            send(play)
            m.d.sync += cmd[1].eq(n)

        # Check distance to obstacle, and stop if too close
        with m.If(ld19.rx.rdy & (li == 46)): # Read checksum
            # Stop if obstacle closer than about 25cm
            with m.If((ai == 0) & (distance < 0x100)):
                stop()

        # Show forward distance measurement on leds
        with m.If((ai == 0) & (li == 46)):
            #m.d.sync += led16.eq(distance)
            m.d.sync += led16.eq(intensity)

        # Roomba control state machine
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
                # Define a song
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
                # Wait for a button to be pressed
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
            # The following sequence up to "STOP" is a fixed test set of movements and could be removed
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
                # Wait for cnt to cycles
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
                # Wait for the uart write to complete
                with m.If(~sending):
                    m.next = "BUTTON"
            with m.State("LEDS"):
                # Set the Roomba Leds
                send(set_leds)
                m.d.sync += [
                    breaker.led5.eq(1),
                    cmd[1].eq(1), # ledbits, dirt detected
                    cmd[2].eq(0), # power color, green
                    cmd[3].eq(128) # power intensity
                ]
                m.next = "WAIT_UART"
            with m.State("ROM"):
                # Execute the ROM
                with m.If(addr < len(rom)):
                    m.d.sync += [
                        cmd[0].eq(r.data),
                        ri.eq(0),
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
                with m.If(ri == l):
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
                        ri.eq(ri + 1),
                        cmd[ri+1].eq(r.data),
                        addr.eq(addr + 1)
                    ]
                    m.next = "PARAM0"
            with m.State("EXEC"):
                with m.If(~sending):
                    m.next = "ROM"
            with m.State("COUNTDOWN"):
                # Wait for 'millis' milliseconds
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

        # Send Roomba command state machine
        with m.FSM():
            with m.State("IDLE"):
                with m.If(sending):
                    m.d.sync += [
                        # Send command byte
                        ci.eq(0),
                        serial.tx.data.eq(cmd[0]),
                        serial.tx.ack.eq(1)
                    ]
                    # Set l to number of parameter bytes for the command
                    set_l(cmd[0])
                    m.next = "SEND"
            with m.State("SEND"):
                m.d.sync += serial.tx.ack.eq(0)
                with m.If(serial.tx.rdy & ~serial.tx.ack):
                    with m.If(ci == l):
                        m.d.sync += sending.eq(0)
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += [
                            # Send parameter byte
                            ci.eq(ci + 1),
                            serial.tx.data.eq(cmd[ci + 1]),
                            serial.tx.ack.eq(1)
                        ]

        # Read Roomba sensor data - 10 byte packets
        with m.If(serial.rx.rdy):
            m.d.sync += leds[1].eq(1)
            m.d.sync += sensor.word_select(si, 8).eq(serial.rx.data)
            with m.If(si == 9):
                m.d.sync += si.eq(0)
            with m.Else():
                m.d.sync += si.eq(si + 1)

        # Clear screen periodically. Period set by width of 'sc'
        m.d.sync += [
            lcd_w.en.eq((sc == 0)),
            lcd_w.data.eq(0)
        ]

        # Either clear screen or use lx and ly to write to LCD memory
        with m.If(sc == 0):
            m.d.sync += lx.eq(lx + 1)
            with m.If(lx == 239):
                m.d.sync += lx.eq(0)
                with m.If(ly == 239):
                    m.d.sync += ly.eq(0)
                with m.Else():
                    m.d.sync += ly.eq(ly + 1)
        with m.Else():
            # Set the correct orientation
            m.d.sync += [
                lx.eq(120 - x[12:20]),
                ly.eq(120 - y[12:20])
            ]

        # Read lidar data
        #
        # Lidar data consists of frames of 47 bytes, containing:
        #
        # byte type   index data         description
        #    0   u8     0:8 header       0x54
        #    1   u8    8:16 ver_len      0x2c
        #  2-3  u16   16:32 speed        millimeters per second
        #  4-5  u16   32:48 start_angle  hundredth of degrees
        # 6-41                           12 points
        #       u16         distance     millimeters
        #        u8         intensity    0 - 255
        #   42  u16 336:352 end_angle    hundredth of degress
        #   44  u16 352:368 timestamp
        #   46   u8 368:376 checksum
        with m.If(ld19.rx.rdy):
            # Save last byte
            m.d.sync += last_byte.eq(ld19.rx.data)
            # Put all the data in long lidar signal
            m.d.sync += lidar.word_select(li, 8).eq(ld19.rx.data)

            # Increment lidar index and sync to header
            # The method of syncing to start of frame is flawed as 0x542c could appear in the data.
            # Should really check for the gaps between bytes received to determine start of frame
            with m.If(li == 46): # End of frame
                m.d.sync += li.eq(0)
            with m.Elif((ld19.rx.data == 0x2c) & (last_byte == 0x54)):
                # Header is 0x54 followed by 0x2c version
                m.d.sync += [
                    li.eq(2),
                    lidar[:8].eq(0x54)
                ]
            with m.Else():
                m.d.sync += li.eq(li + 1)

            # Set index to points that start at byte 6. 
            # There are 12 points consisting or a 16-bit distance and 8-bit intensity
            # The data following the points is the 16-bit end angle, a 16-bit temestamp and an 8-bit crc
            # Those are currently ignored
            with m.If(li == 5):
                m.d.sync += [
                    pi.eq(0),
                    pp.eq(0)
                ]
            with m.Elif((li >= 6) & (li < 42)):
                # Check for frames with start angle closer to zero
                with m.If(li == 6):
                    # Bytes 4 and 6 are the start angle for the frame
                    # Bytes 2 and 3 are the speed, which is currently ignored
                    m.d.sync += [
                        angle.eq(lidar[32:48]),
                        start_angle.eq(lidar[32:48])
                    ]
                    with m.If(lidar[32:48] < min_angle):
                        m.d.sync += [
                            min_angle.eq(lidar[32:48]),
                            # Reset the angle index to 0
                            ai.eq(0),
                            sc.eq(sc + 1)
                        ]
                with m.If(pi == 2): # End of point data
                    m.d.sync += [
                        # Move on to next point
                        pi.eq(0),
                        pp.eq(pp + 1),
                        # Set intensity and write to LCD buffer
                        intensity.eq(ld19.rx.data),
                        lcd_w.data.eq((ld19.rx.data > 0) & 
                                      ((y[20:] == 0) | (y[20:] == 0xF)) & 
                                      ((x[20:] == 0) | (x[20:] == 0xF)) &
                                      (sc > 0) & (ai < 450)),
                        lcd_w.en.eq(1)
                    ]

                    #  Check for angle going over 360 degrees or index reaching 450
                    #  Angle icrements by 0.8 degrees for each point
                    with m.If((pp < 11) & ((angle + 80) >= 36000)):
                        m.d.sync += [
                            ai.eq(0),
                            sc.eq(sc + 1),
                            angle.eq(angle + 80 - 36000)
                        ]
                    with m.Else():
                        m.d.sync += [
                            ai.eq(ai + 1),
                            angle.eq(angle + 80)
                        ]
                        with m.If(ai == 449):
                            m.d.sync += [
                                ai.eq(0),
                                sc.eq(sc + 1)
                           ]
                with m.Else():
                    m.d.sync += pi.eq(pi + 1)
                    # Set distance and calculate x and y co-ordinates using sin and cosine tables
                    with m.If(pi == 1):
                        m.d.sync += [
                            distance.eq(Cat(last_byte, ld19.rx.data)),
                            x.eq(Cat(last_byte, ld19.rx.data) * sin_s),
                            y.eq(Cat(last_byte, ld19.rx.data) * cos_s)
                        ]

        return m

# Generate and upload bitstream
if __name__ == "__main__":
    platform = BlackIceMXPlatform()
    platform.add_resources(roomba_pmod)
    platform.add_resources(breaker_pmod)
    platform.add_resources(pmod_led8_1)
    platform.add_resources(pmod_led8_2)
    platform.add_resources(pmod_bt)
    platform.add_resources(oled_pmod)
    platform.build(RoombaTest(), do_program=True)

