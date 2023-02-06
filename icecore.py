import os
import subprocess

from amaranth.build import *
from amaranth.vendor.lattice_ice40 import *
from amaranth_boards.resources import *


__all__ = ["IceCorePlatform"]


# IceCore : https://github.com/folknology/IceCore
class IceCorePlatform(LatticeICE40Platform):
    device      = "iCE40HX4K"
    package     = "TQ144"
    default_clk = "clk25"
    resources   = [
        Resource("clk25", 0, Pins("60", dir="i"),
            Clock(25e6), Attrs(IO_STANDARD="SB_LVCMOS")
        ),

        *LEDResources(pins="49 52 55 56", invert=True, attrs=Attrs(IO_STANDARD="SB_LVCMOS", PULLUP=1)),
        # Color aliases, pullups helps when signals used beyond driving leds to get good logic high
        Resource("led_b", 0, Pins("49", dir="o"), Attrs(IO_STANDARD="SB_LVCMOS", PULLUP=1)),
        Resource("led_g", 0, Pins("52", dir="o"), Attrs(IO_STANDARD="SB_LVCMOS", PULLUP=1)),
        Resource("led_y", 0, Pins("55", dir="o"), Attrs(IO_STANDARD="SB_LVCMOS", PULLUP=1)),
        Resource("led_r", 0, Pins("56", dir="o"), Attrs(IO_STANDARD="SB_LVCMOS", PULLUP=1)),

        Resource("sck", 0, Pins("70", dir="i"), Attrs(IO_STANDARD="SB_LVCMOS")),
        Resource("mosi", 0, Pins("67", dir="o"), Attrs(IO_STANDARD="SB_LVCMOS")),
        Resource("cs", 0, Pins("71", dir="i"), Attrs(IO_STANDARD="SB_LVCMOS")),
        Resource("miso", 0, Pins("68", dir="i"), Attrs(IO_STANDARD="SB_LVCMOS")),

        Resource("dsck", 0, Pins("76", dir="i"), Attrs(IO_STANDARD="SB_LVCMOS")),
        Resource("dd0", 0, Pins("73", dir="i"), Attrs(IO_STANDARD="SB_LVCMOS")),
        Resource("dcs", 0, Pins("75", dir="i"), Attrs(IO_STANDARD="SB_LVCMOS")),
        Resource("dd1", 0, Pins("74", dir="i"), Attrs(IO_STANDARD="SB_LVCMOS")),

        # Buttons overlap with Blue & Green leds in addition to PMOD 10
        *ButtonResources(pins="49 52", invert=True, attrs=Attrs(IO_STANDARD="SB_LVCMOS", PULLUP=1)),

        UARTResource(0,
            rx="61", tx="62",
            attrs=Attrs(IO_STANDARD="SB_LVCMOS", PULLUP=1)
        ),

        *SPIFlashResources(0, cs_n="71", clk="70", copi="67", cipo="68", wp_n="64", hold_n="63",
           attrs=Attrs(IO_STANDARD="SB_LVCMOS")
        ),

        SDRAMResource(0, clk="129", cke="128", cs_n="113", we_n="107",  ras_n="112", cas_n="110",
            dqm="93 94", ba="114",
            a="117 119 121 124 130 125 122 120 118 116 115",
            dq="78 79 80 81 82 83 84 85 87 88 90 91 95 96 97 98",
            attrs=Attrs(IO_STANDARD="SB_LVCMOS")
        ),

        # Also available as PMODs 9/10 when SDCard not used 
        *SDCardResources(0, clk="104", cmd="102", dat0="105", dat1="106", dat2="99", dat3="101",
            attrs=Attrs(IO_STANDARD="SB_LVCMOS")
        ),

    ]
    connectors = []

    def toolchain_program(self, products, name, **kwargs):
        device = os.environ.get("DEVICE", "/dev/ttyACM0")
        print("Programming ", device)
        with products.extract("{}.bin".format(name)) as bitstream_filename:
            subprocess.check_call(["cp", bitstream_filename, device])


if __name__ == "__main__":
    from .test.blinky import *
    IceCorePlatform().build(Blinky(), do_program=True)

