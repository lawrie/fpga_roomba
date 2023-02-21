from amaranth.build import Connector
from icecore import IceCorePlatform

__all__ = ["BlackIceMXPlatform"]

# PMOD pinout primnitives for building modular connectors
PMOD1 = " 38  37  32  31 - -"
PMOD2 = " 34  33  29  28 - -"
PMOD3 = " 18 17  12  11 - -"
PMOD4 = " 16 15  10  9 - -"
PMOD5 = " 136  137  134  135 - -"
PMOD6 = " 141  142  138  139 - -"
PMOD7 = " 1  2  7  8  - -"
PMOD8 = " 143  144  3  4  - -"
PMOD9 = " 106  105  101  99 - -"
PMOD10 = " 104  102  49  52 - -"
PMOD11 = " 19  20  23  24 - -"
PMOD12 = " 21  22  25  26 - -"
MXPINS = " - - -"


# BlackIce Mx : https://github.com/folknology/BlackIceMx
# Includes/Extends IceCore : https://github.com/folknology/IceCore
class BlackIceMXPlatform(IceCorePlatform):
    # BlackIce Mx Connectors can be mapped as Dual Pmods
    # or MixMods (quad Pmods with MiXed signals in between)
    # https://raw.githubusercontent.com/folknology/BlackIceMx/master/blackicemx.svg.png
    connectors  = [
        # Double Pmod pin mapping
        Connector("pmod", 0, PMOD2 + PMOD1),  # PMOD1/2
        Connector("pmod", 1, PMOD4 + PMOD3),  # PMOD3/4
        Connector("pmod", 2, PMOD6 + PMOD5),  # PMOD5/6
        Connector("pmod", 3, PMOD8 + PMOD7),  # PMOD7/8
        Connector("pmod", 4, PMOD10 + PMOD9),  # PMOD9/10
        Connector("pmod", 5, PMOD12 + PMOD11),  # PMOD11/12
        # MixMod pin mapping
        Connector("mixmod", 0, PMOD1 + MXPINS + PMOD3 + PMOD2 + MXPINS + PMOD4),  # MX1
        Connector("mixmod", 1, PMOD5 + MXPINS + PMOD7 + PMOD6 + MXPINS + PMOD8),  # MX2
        Connector("mixmod", 2, PMOD9 + MXPINS + PMOD11 + PMOD10 + MXPINS + PMOD12)  # MX3
    ]


if __name__ == "__main__":
    from .test.blinky import *
    BlackIceMXPlatform().build(Blinky(), do_program=True)

