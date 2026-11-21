# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 LitePCIe contributors

from migen import *
from migen.genlib.misc import WaitTimer

from litex.gen import LiteXModule
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import CSRStorage, CSRStatus
from litex.soc.interconnect.csr_eventmanager import EventManager, EventSourceLevel

from litepcie.core import LitePCIeEndpoint
from litepcie.frontend.wishbone import LitePCIeWishboneMaster, map_wishbone_dat

# LitePCIe EPC Outbound ----------------------------------------------------------------------------

class LitePCIeEPCOutbound(LiteXModule):
    """Wishbone window on the memory of the host.

    The window is made of ``slots`` slots of ``slot_size`` bytes. Each slot has a 64-bit bus
    address of the host, aligned on 4 KiB, with bit 0 set when the slot is mapped. An access to a
    slot is issued as a Memory Read or Write Request of one Dword at the address of the slot
    plus the offset of the access in the slot; the byte selects of a write are its Byte Enables.
    An access to a slot that is not mapped, or one the host does not complete, reads all ones.
    """
    def __init__(self, endpoint, slots=32, slot_size=0x10000, qword_aligned=False):
        assert slot_size >= 0x1000
        self.bus      = wishbone.Interface(data_width=32, address_width=32, addressing="word")
        self.slot     = Signal(max=max(slots, 2))
        self.base     = Signal(64)
        self.base_we  = Signal()
        self.size     = slots*slot_size

        # # #

        offset_bits = log2_int(slot_size) - 2 # In Dwords.
        index_bits  = log2_int(slots)

        # Slots.
        table   = Memory(64, slots)
        wr_port = table.get_port(write_capable=True)
        rd_port = table.get_port(async_read=True)
        self.specials += table, wr_port, rd_port
        self.comb += [
            wr_port.adr.eq(self.slot),
            wr_port.dat_w.eq(self.base),
            wr_port.we.eq(self.base_we),
            rd_port.adr.eq(self.bus.adr[offset_bits:offset_bits + index_bits]),
        ]

        # Registered before the Request: keeps the translation out of the path to the packetizer.
        address = Signal(64)
        mapped  = Signal()

        # Timeout.
        self.timeout = timeout = WaitTimer(2**16)

        # Get Master port from Crossbar.
        port = endpoint.crossbar.get_master_port()
        self.comb += [
            port.source.channel.eq(port.channel),
            port.source.first.eq(1),
            port.source.last.eq(1),
            port.source.adr.eq(address),
            port.source.req_id.eq(endpoint.phy.id),
            port.source.tag.eq(0),
            port.source.len.eq(1),
            port.source.dat.eq(self.bus.dat_w),
            port.source.first_be.eq(Mux(self.bus.we, self.bus.sel, 0xf)),
        ]

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(self.bus.stb & self.bus.cyc,
                NextValue(address, Cat(Constant(0, 12), rd_port.dat_r[12:]) +
                    Cat(Constant(0, 2), self.bus.adr[:offset_bits])),
                NextValue(mapped, rd_port.dat_r[0] & (self.bus.sel != 0)),
                NextState("ISSUE")
            )
        )
        fsm.act("ISSUE",
            If(~mapped,
                self.bus.dat_r.eq(0xffffffff),
                self.bus.ack.eq(1),
                NextState("IDLE")
            ).Elif(self.bus.we,
                NextState("ISSUE-WRITE")
            ).Else(
                NextState("ISSUE-READ")
            )
        )
        fsm.act("ISSUE-WRITE",
            timeout.wait.eq(1),
            port.source.valid.eq(1),
            port.source.we.eq(1),
            If(port.source.ready | timeout.done,
                self.bus.ack.eq(1),
                NextState("IDLE")
            )
        )
        fsm.act("ISSUE-READ",
            timeout.wait.eq(1),
            port.source.valid.eq(1),
            port.source.we.eq(0),
            If(port.source.ready | timeout.done,
                NextState("RECEIVE-READ-COMPLETION")
            )
        )
        read_error = Signal()
        self.comb += read_error.eq(timeout.done | (port.sink.valid & port.sink.first & port.sink.err))
        fsm.act("RECEIVE-READ-COMPLETION",
            timeout.wait.eq(1),
            port.sink.ready.eq(1),
            If((port.sink.valid & port.sink.first) | timeout.done,
                If(read_error,
                    self.bus.dat_r.eq(0xffffffff),
                ).Else(
                    map_wishbone_dat(
                        address       = port.sink.adr,
                        data          = port.sink.dat,
                        wishbone_dat  = self.bus.dat_r,
                        qword_aligned = qword_aligned,
                    ),
                ),
                self.bus.ack.eq(1),
                NextState("IDLE")
            )
        )

# LitePCIe EPC -------------------------------------------------------------------------------------

class LitePCIeEPC(LiteXModule):
    """Firmware-controlled endpoint with one fixed BAR0.

    ABI v2, one 32-bit word per register:

    ======  ================  ==================================================================
    Offset  Register          Description
    ======  ================  ==================================================================
    0x00    control           Bit 0: link enable. Bit 1: BAR0 enable. Bit 2: BAR0 serviced by
                              the firmware instead of the ``bus`` master.
    0x04    bar0_base         CPU address BAR0 is mapped at on ``bus``, taken while BAR0 is
                              disabled.
    0x08    bar0_size         Size of BAR0.
    0x0c    link_status       Bit 0: link up.
    0x10    version           2.
    0x14    access_info       Host access to service. Bit 31: pending. Bit 30: write.
                              Bits 23-8: identifier. Bits 3-0: byte selects of the Dword.
    0x18    access_offset     Offset of the Dword in BAR0.
    0x1c    access_data       Dword of a write.
    0x20    access_response   Dword returned to a read.
    0x24    access_done       Write the identifier in bits 15-0 to complete the access; bit 31
                              fails it. A write with another identifier is ignored.
    0x28    outbound_slot     Slot of the outbound window written by outbound_base_hi.
    0x2c    outbound_base_lo  Bits 31-12 of the host bus address of the slot. Bit 0: mapped.
    0x30    outbound_base_hi  Bits 63-32 of the address. Its write takes both words.
    0x34    outbound_info     Bits 15-0: number of slots. Bits 23-16: log2 of the slot size.
    0x38    msi               Write the number of the MSI to send.
    0x3c    msi_status        Bit 0: the MSI is not sent yet.
    0x40    ev_status         Bit 0: a host access is pending.
    0x44    ev_pending        Same, as an interrupt source.
    0x48    ev_enable         Bit 0: enables the interrupt.
    ======  ================  ==================================================================

    Host requests to a disabled BAR0 are terminated: writes are dropped and reads return all ones.
    A failed access is terminated the same way. The configuration space is not programmable.
    """
    def __init__(self, phy, outbound_slots=32, outbound_slot_size=0x10000):
        self.control          = CSRStorage(3)
        self.bar0_base        = CSRStorage(32)
        self.bar0_size        = CSRStatus(32, reset=phy.bar0_size)
        self.link_status      = CSRStatus(1)
        self.version          = CSRStatus(32, reset=2)
        self.access_info      = CSRStatus(32)
        self.access_offset    = CSRStatus(32)
        self.access_data      = CSRStatus(32)
        self.access_response  = CSRStorage(32)
        self.access_done      = CSRStorage(32)
        self.outbound_slot    = CSRStorage(16)
        self.outbound_base_lo = CSRStorage(32)
        self.outbound_base_hi = CSRStorage(32)
        self.outbound_info    = CSRStatus(32,
            reset=outbound_slots | (log2_int(outbound_slot_size) << 16))
        self.msi              = CSRStorage(8)
        self.msi_status       = CSRStatus(1)
        self.ev               = EventManager()
        self.ev.access        = EventSourceLevel()
        self.ev.finalize()

        # # #

        link_enable = self.control.storage[0]
        bar_enable  = self.control.storage[1]
        bar_trap    = self.control.storage[2]

        bar0_base = Signal(32)
        self.sync += If(~bar_enable, bar0_base.eq(self.bar0_base.storage))

        # Endpoint / Inbound -----------------------------------------------------------------------
        self.endpoint = ResetInserter()(LitePCIeEndpoint(phy,
            endianness    = phy.endianness,
            address_width = 64))
        self.mmap = ResetInserter()(LitePCIeWishboneMaster(self.endpoint,
            base_address  = bar0_base,
            qword_aligned = phy.qword_aligned))
        self.bus = wishbone.Interface()
        mmap_bus = self.mmap.bus

        # Firmware-serviced accesses: one Dword of a host request at a time.
        access_id   = Signal(16)
        access_ack  = Signal()
        access_hide = Signal() # The identifier changes in the cycle after the completion.
        pending     = Signal()
        self.comb += [
            pending.eq(mmap_bus.cyc & mmap_bus.stb & bar_enable & bar_trap & ~access_hide),
            access_ack.eq(pending & self.access_done.re &
                (self.access_done.storage[:16] == access_id)),
            self.access_info.status.eq(Cat(mmap_bus.sel, Constant(0, 4), access_id,
                Constant(0, 6), mmap_bus.we, pending)),
            self.access_offset.status.eq(
                Cat(Constant(0, 2), mmap_bus.adr - bar0_base[2:]) & (phy.bar0_size - 1)),
            self.access_data.status.eq(mmap_bus.dat_w),
            self.ev.access.trigger.eq(pending),
        ]
        self.sync += [
            access_hide.eq(access_ack),
            If(access_ack, access_id.eq(access_id + 1)),
        ]

        self.comb += [
            self.endpoint.reset.eq(~link_enable),
            self.mmap.reset.eq(~link_enable),
            phy.reset.eq(~link_enable),
            self.link_status.status.eq(phy._link_status.fields.status),
            If(~bar_enable,
                # Terminate the request instead of stalling the host.
                mmap_bus.ack.eq(mmap_bus.cyc & mmap_bus.stb),
                mmap_bus.dat_r.eq(0xffffffff),
            ).Elif(bar_trap,
                mmap_bus.ack.eq(access_ack),
                mmap_bus.dat_r.eq(Mux(self.access_done.storage[31], 0xffffffff,
                    self.access_response.storage)),
            ).Else(
                mmap_bus.connect(self.bus),
            ),
        ]

        # Outbound ---------------------------------------------------------------------------------
        self.outbound = ResetInserter()(LitePCIeEPCOutbound(self.endpoint,
            slots         = outbound_slots,
            slot_size     = outbound_slot_size,
            qword_aligned = phy.qword_aligned))
        self.outbound_bus  = self.outbound.bus
        self.outbound_size = self.outbound.size
        self.comb += [
            self.outbound.reset.eq(~link_enable),
            self.outbound.slot.eq(self.outbound_slot.storage),
            self.outbound.base.eq(Cat(self.outbound_base_lo.storage, self.outbound_base_hi.storage)),
            self.outbound.base_we.eq(self.outbound_base_hi.re),
        ]

        # MSI --------------------------------------------------------------------------------------
        msi = getattr(phy, "msi", None)
        if msi is not None:
            msi_pending = Signal()
            self.sync += [
                If(self.msi.re,
                    msi_pending.eq(1)
                ).Elif(msi.ready | ~link_enable,
                    msi_pending.eq(0)
                )
            ]
            self.comb += [
                msi.valid.eq(msi_pending),
                msi.dat.eq(self.msi.storage),
                self.msi_status.status.eq(msi_pending),
            ]
