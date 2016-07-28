from itertools import product

from migen import *
from migen.genlib.fsm import FSM, NextState
from misoc.interconnect import wishbone, csr


class SPIClockGen(Module):
    def __init__(self, width):
        self.load = Signal(width)
        self.bias = Signal()  # bias this clock phase to longer times
        self.edge = Signal()
        self.clk = Signal(reset=1)

        cnt = Signal.like(self.load)
        bias = Signal()
        zero = Signal()
        self.comb += [
            zero.eq(cnt == 0),
            self.edge.eq(zero & ~bias),
        ]
        self.sync += [
            If(zero,
                bias.eq(0),
            ).Else(
                cnt.eq(cnt - 1),
            ),
            If(self.edge,
                cnt.eq(self.load[1:]),
                bias.eq(self.load[0] & (self.clk ^ self.bias)),
                self.clk.eq(~self.clk),
            )
        ]


class SPIRegister(Module):
    def __init__(self, width):
        self.data = Signal(width)
        self.o = Signal()
        self.i = Signal()
        self.lsb = Signal()
        self.shift = Signal()
        self.sample = Signal()

        self.comb += [
            self.o.eq(Mux(self.lsb, self.data[0], self.data[-1])),
        ]
        self.sync += [
            If(self.shift,
                If(self.lsb,
                    self.data[:-1].eq(self.data[1:]),
                ).Else(
                    self.data[1:].eq(self.data[:-1]),
                )
            ),
            If(self.sample,
                If(self.lsb,
                    self.data[-1].eq(self.i),
                ).Else(
                    self.data[0].eq(self.i),
                )
            )
        ]


class SPIBitCounter(Module):
    def __init__(self, width):
        self.n_read = Signal(width)
        self.n_write = Signal(width)
        self.read = Signal()
        self.write = Signal()
        self.done = Signal()

        self.comb += [
            self.write.eq(self.n_write != 0),
            self.read.eq(self.n_read != 0),
            self.done.eq(~(self.write | self.read)),
        ]
        self.sync += [
            If(self.write,
                self.n_write.eq(self.n_write - 1),
            ).Elif(self.read,
                self.n_read.eq(self.n_read - 1),
            )
        ]


class SPIMachine(Module):
    def __init__(self, data_width, clock_width, bits_width):
        ce = CEInserter()
        self.submodules.cg = ce(SPIClockGen(clock_width))
        self.submodules.reg = ce(SPIRegister(data_width))
        self.submodules.bits = ce(SPIBitCounter(bits_width))
        self.div_write = Signal.like(self.cg.load)
        self.div_read = Signal.like(self.cg.load)
        self.clk_phase = Signal()
        self.start = Signal()
        self.cs = Signal()
        self.oe = Signal()
        self.done = Signal()

        # # #

        fsm = CEInserter()(FSM("IDLE"))
        self.submodules += fsm

        fsm.act("IDLE",
            If(self.start,
                If(self.clk_phase,
                    NextState("WAIT"),
                ).Else(
                    NextState("SETUP"),
                )
            )
        )
        fsm.act("SETUP",
            self.reg.sample.eq(1),
            NextState("HOLD"),
        )
        fsm.act("HOLD",
            If(self.bits.done & ~self.start,
                If(self.clk_phase,
                    NextState("IDLE"),
                ).Else(
                    NextState("WAIT"),
                )
            ).Else(
                self.reg.shift.eq(~self.start),
                NextState("SETUP"),
            )
        )
        fsm.act("WAIT",
            If(self.bits.done,
                NextState("IDLE"),
            ).Else(
                NextState("SETUP"),
            )
        )

        write0 = Signal()
        self.sync += [
            If(self.cg.edge & self.reg.shift,
                write0.eq(self.bits.write),
            )
        ]
        self.comb += [
            self.cg.ce.eq(self.start | self.cs | ~self.cg.edge),
            If(self.bits.write | ~self.bits.read,
                self.cg.load.eq(self.div_write),
            ).Else(
                self.cg.load.eq(self.div_read),
            ),
            self.cg.bias.eq(self.clk_phase),
            fsm.ce.eq(self.cg.edge),
            self.cs.eq(~fsm.ongoing("IDLE")),
            self.reg.ce.eq(self.cg.edge),
            self.bits.ce.eq(self.cg.edge & self.reg.sample),
            self.done.eq(self.cg.edge & self.bits.done & fsm.ongoing("HOLD")),
            self.oe.eq(write0 | self.bits.write),
        ]


class SPIMaster(Module):
    """SPI Master.

    Notes:
        * M = 32 is the data width (width of the data register,
          maximum write bits, maximum read bits)
        * Every transfer consists of a write_length 0-M bit write followed
          by a read_length 0-M bit read.
        * cs_n is asserted at the beginning and deasserted at the end of the
          transfer if there is no other transfer pending.
        * cs_n handling is agnostic to whether it is one-hot or decoded
          somewhere downstream. If it is decoded, "cs_n all deasserted"
          should be handled accordingly (no slave selected).
          If it is one-hot, asserting multiple slaves should only be attempted
          if miso is either not connected between slaves, or open collector,
          or correctly multiplexed externally.
        * If config.cs_polarity == 0 (cs active low, the default),
          "cs_n all deasserted" means "all cs_n bits high".
        * cs is not mandatory in pads. Framing and chip selection can also
          be handled independently through other means.
        * If there is a miso wire in pads, the input and output can be done
          with two signals (a.k.a. 4-wire SPI), else mosi must be used for
          both output and input (a.k.a. 3-wire SPI) and config.half_duplex
          must to be set when reading data is desired.
        * For 4-wire SPI only the sum of read_length and write_length matters.
          The behavior is the same no matter how the total transfer length is
          divided between the two. For 3-wire SPI, the direction of mosi/miso
          is switched from output to input after write_len cycles, at the
          "shift_out" clk edge corresponding to bit write_length + 1 of the
          transfer.
        * The first bit output on mosi is always the MSB/LSB (depending on
          config.lsb_first) of the data register, independent of
          xfer.write_len. The last bit input from miso always ends up in
          the LSB/MSB (respectively) of the data register, independent of
          read_len.
        * Data output on mosi in 4-wire SPI during the read cycles is what
          is found in the data register at the time.
          Data in the data register outside the least/most (depending
          on config.lsb_first) significant read_length bits is what is
          seen on miso during the write cycles.
        * The SPI data register is double-buffered: Once a transfer has
          started, new write data can be written, queuing a new transfer.
          Transfers submitted this way are chained and executed without
          deasserting cs. Once a transfer completes, the previous transfer's
          read data is available in the data register.
        * Writes to the config register take effect immediately. Writes to xfer
          and data are synchronized to the start of a transfer.
        * A wishbone data register write is ack-ed when the transfer has
          been written to the intermediate buffer. It will be started when
          there are no other transactions being executed, either starting
          a new SPI transfer of chained to an in-flight transfer.
          Writes take two cycles unless the write is to the data register
          and another chained transfer is pending and the transfer being
          executed is not complete. Reads always finish in two cycles.

    Transaction Sequence:
        * If desired, write the config register to set up the core.
        * If desired, write the xfer register to change lengths and cs_n.
        * Write the data register (also for zero-length writes),
          writing triggers the transfer and when the transfer is accepted to
          the inermediate buffer, the write is ack-ed.
        * If desired, read the data register corresponding to the last
          completed transfer.
        * If desired, change xfer register for the next transfer.
        * If desired, write data queuing the next (possibly chained) transfer.

    Register address and bit map:

    config (address 2):
        1 offline: all pins high-z (reset=1)
        1 active: cs/transfer active (read-only)
        1 pending: transfer pending in intermediate buffer (read-only)
        1 cs_polarity: active level of chip select (reset=0)
        1 clk_polarity: idle level of clk (reset=0)
        1 clk_phase: first edge after cs assertion to sample data on (reset=0)
            (clk_polarity, clk_phase) == (CPOL, CPHA) in Freescale language.
            (0, 0): idle low, output on falling, input on rising
            (0, 1): idle low, output on rising, input on falling
            (1, 0): idle high, output on rising, input on falling
            (1, 1): idle high, output on falling, input on rising
            There is never a clk edge during a cs edge.
        1 lsb_first: LSB is the first bit on the wire (reset=0)
        1 half_duplex: 3-wire SPI, in/out on mosi (reset=0)
        8 undefined
        8 div_write: counter load value to divide this module's clock
            to generate the SPI write clk (reset=0)
            f_clk/f_spi_write == div_write + 2
        8 div_read: ditto for the read clock

    xfer (address 1):
        16 cs: active high bit mask of chip selects to assert (reset=0)
        6 write_len: 0-M bits (reset=0)
        2 undefined
        6 read_len: 0-M bits (reset=0)
        2 undefined

    data (address 0):
        M write/read data (reset=0)
    """
    def __init__(self, pads, data_width=32, clock_width=8, bits_width=6):
        self.wbus = wbus = wishbone.Interface(data_width=32)

        ###

        # Wishbone
        config = Record([
            ("offline", 1),
            ("active", 1),
            ("pending", 1),
            ("cs_polarity", 1),
            ("clk_polarity", 1),
            ("clk_phase", 1),
            ("lsb_first", 1),
            ("half_duplex", 1),
            ("padding", 8),
            ("div_write", 8),
            ("div_read", 8),
        ])
        config.offline.reset = 1

        xfer = Record([
            ("cs", 16),
            ("write_length", 6),
            ("padding0", 2),
            ("read_length", 6),
            ("padding1", 2),
        ])

        self.submodules.spi = spi = SPIMachine(
            data_width=len(wbus.dat_w),
            clock_width=len(config.div_read),
            bits_width=len(xfer.read_length))

        pending = Signal()
        cs = Signal.like(xfer.cs)
        data_read = Signal.like(spi.reg.data)
        data_write = Signal.like(spi.reg.data)

        self.comb += [
            wbus.dat_r.eq(
                Array([data_read, xfer.raw_bits(), config.raw_bits()
                       ])[wbus.adr]),
            spi.start.eq(pending & (~spi.cs | spi.done)),
            spi.clk_phase.eq(config.clk_phase),
            spi.reg.lsb.eq(config.lsb_first),
            spi.div_write.eq(config.div_write),
            spi.div_read.eq(config.div_read),
        ]
        self.sync += [
            If(spi.done,
                data_read.eq(spi.reg.data),
            ),
            If(spi.start,
                cs.eq(xfer.cs),
                spi.bits.n_write.eq(xfer.write_length),
                spi.bits.n_read.eq(xfer.read_length),
                spi.reg.data.eq(data_write),
                pending.eq(0),
            ),
            # wb.ack a transaction if any of the following:
            # a) reading,
            # b) writing to non-data register
            # c) writing to data register and no pending transfer
            # d) writing to data register and pending and swapping buffers
            wbus.ack.eq(wbus.cyc & wbus.stb &
                       (~wbus.we | (wbus.adr != 0) | ~pending | spi.done)),
            If(wbus.ack,
                wbus.ack.eq(0),
                If(wbus.we,
                    Array([data_write, xfer.raw_bits(), config.raw_bits()
                          ])[wbus.adr].eq(wbus.dat_w),
                    If(wbus.adr == 0,  # data register
                        pending.eq(1),
                    ),
                ),
            ),
            config.active.eq(spi.cs),
            config.pending.eq(pending),
        ]

        # I/O
        if hasattr(pads, "cs_n"):
            cs_n_t = TSTriple(len(pads.cs_n))
            self.specials += cs_n_t.get_tristate(pads.cs_n)
            self.comb += [
                cs_n_t.oe.eq(~config.offline),
                cs_n_t.o.eq((cs & Replicate(spi.cs, len(cs))) ^
                            Replicate(~config.cs_polarity, len(cs))),
            ]

        clk_t = TSTriple()
        self.specials += clk_t.get_tristate(pads.clk)
        self.comb += [
            clk_t.oe.eq(~config.offline),
            clk_t.o.eq((spi.cg.clk & spi.cs) ^ config.clk_polarity),
        ]

        mosi_t = TSTriple()
        self.specials += mosi_t.get_tristate(pads.mosi)
        self.comb += [
            mosi_t.oe.eq(~config.offline & spi.cs &
                         (spi.oe | ~config.half_duplex)),
            mosi_t.o.eq(spi.reg.o),
            spi.reg.i.eq(Mux(config.half_duplex, mosi_t.i,
                             getattr(pads, "miso", mosi_t.i))),
        ]
