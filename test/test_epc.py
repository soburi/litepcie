# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 LitePCIe contributors

from types import SimpleNamespace

import pytest

from migen import Signal
from litex.soc.interconnect import stream
from litepcie.common import msi_layout
from litex.gen import LiteXModule
from litex.gen.sim import run_simulation
from litepcie.frontend.epc import LitePCIeEPC
from test.model.host import Host


@pytest.mark.parametrize("data_width", [64, 256])
def test_epc_relocation_and_reset(data_width):
    class DUT(LiteXModule):
        def __init__(self):
            self.host = Host(data_width, 0x100, 0x400)
            phy = self.host.phy
            phy.endianness = "big"
            phy.qword_aligned = False
            phy.bar0_size = 0x100000
            phy.reset = Signal()
            phy._link_status = SimpleNamespace(fields=SimpleNamespace(status=Signal()))
            self.epc = LitePCIeEPC(phy)
            self.comb += self.epc.bus.ack.eq(self.epc.bus.cyc & self.epc.bus.stb)

    dut = DUT()
    writes = []

    def monitor():
        for _ in range(150):
            if (yield dut.epc.bus.cyc) and (yield dut.epc.bus.stb):
                writes.append((yield dut.epc.bus.adr))
            yield

    def stimulus():
        yield
        assert (yield dut.host.phy.reset) == 1
        assert (yield dut.epc.bus.cyc) == 0
        # The base is taken while the BAR is disabled.
        yield dut.epc.bar0_base.storage.eq(0x10000000)
        yield
        yield
        yield dut.epc.control.storage.eq(3)
        yield
        yield
        assert (yield dut.host.phy.reset) == 0
        yield from dut.host.chipset.wr32(4, [0x12345678])
        for _ in range(30):
            yield
        assert writes == [0x04000004]
        yield dut.epc.control.storage.eq(0)
        yield
        yield
        assert (yield dut.host.phy.reset) == 1
        assert (yield dut.epc.bus.cyc) == 0

    run_simulation(dut, [stimulus(), monitor(),
        dut.host.chipset.phy.phy_sink.generator(),
        dut.host.chipset.phy.phy_source.generator()])


def test_epc_bar_gating():
    class DUT(LiteXModule):
        def __init__(self):
            self.host = Host(64, 0x100, 0x400)
            phy = self.host.phy
            phy.endianness = "big"
            phy.qword_aligned = False
            phy.bar0_size = 0x100000
            phy.reset = Signal()
            phy._link_status = SimpleNamespace(fields=SimpleNamespace(status=Signal()))
            self.epc = LitePCIeEPC(phy)
            self.comb += [
                self.epc.bus.ack.eq(self.epc.bus.cyc & self.epc.bus.stb),
                self.epc.bus.dat_r.eq(0x11223344),
            ]

    dut = DUT()
    accesses = []

    def monitor():
        for _ in range(400):
            if (yield dut.epc.bus.cyc) and (yield dut.epc.bus.stb):
                accesses.append(((yield dut.epc.bus.adr), (yield dut.epc.bus.we)))
            yield

    def stimulus():
        yield
        yield dut.epc.bar0_base.storage.eq(0x10000000)
        yield
        yield
        # Link up, BAR disabled: requests complete without reaching the bus.
        yield dut.epc.control.storage.eq(1)
        yield
        yield
        yield from dut.host.chipset.wr32(4, [0x12345678])
        yield from dut.host.chipset.rd32(8)
        assert dut.host.chipset.rd_data == [0xffffffff]
        for _ in range(10):
            yield
        assert accesses == []
        # The base written while the BAR is enabled is not taken.
        yield dut.epc.control.storage.eq(3)
        yield
        yield dut.epc.bar0_base.storage.eq(0x20000000)
        yield
        yield
        yield from dut.host.chipset.rd32(8)
        # The model PHY is big endian.
        assert dut.host.chipset.rd_data == [0x44332211]
        for _ in range(10):
            yield
        assert set(accesses) == {(0x04000008, 0)}

    run_simulation(dut, [stimulus(), monitor(),
        dut.host.chipset.phy.phy_sink.generator(),
        dut.host.chipset.phy.phy_source.generator()])


def swap32(value):
    return int.from_bytes(value.to_bytes(4, "little"), "big")


class EPCDUT(LiteXModule):
    def __init__(self, data_width=64):
        self.host = Host(data_width, 0x100, 0x400)
        phy = self.host.phy
        phy.endianness = "big"
        phy.qword_aligned = False
        phy.bar0_size = 0x100000
        phy.reset = Signal()
        phy.msi = stream.Endpoint(msi_layout())
        phy._link_status = SimpleNamespace(fields=SimpleNamespace(status=Signal()))
        self.epc = LitePCIeEPC(phy, outbound_slots=4, outbound_slot_size=0x1000)

    def generators(self, *generators):
        return [*generators,
            self.host.generator(),
            self.host.chipset.generator(),
            self.host.chipset.phy.phy_sink.generator(),
            self.host.chipset.phy.phy_source.generator()]


def csr_write(csr, value):
    yield csr.storage.eq(value)
    yield csr.re.eq(1)
    yield
    yield csr.re.eq(0)
    yield


def test_epc_bar_trap():
    dut = EPCDUT()
    served = []

    def firmware():
        # Serve two accesses: fail the first, answer the second.
        for response, error in ((0, 1), (0xa1b2c3d4, 0)):
            while not ((yield dut.epc.access_info.status) >> 31):
                yield
            info = (yield dut.epc.access_info.status)
            served.append((info >> 30 & 1, info & 0xf, (yield dut.epc.access_offset.status),
                (yield dut.epc.access_data.status), (yield dut.epc.ev.access.trigger)))
            # A completion with a stale identifier is ignored.
            yield from csr_write(dut.epc.access_done, ((info >> 8) + 1) & 0xffff)
            assert (yield dut.epc.access_info.status) == info
            yield dut.epc.access_response.storage.eq(response)
            yield from csr_write(dut.epc.access_done, (info >> 8 & 0xffff) | (error << 31))

    def stimulus():
        yield
        # The offset does not depend on the BAR base.
        yield dut.epc.bar0_base.storage.eq(0x10000000)
        yield
        yield
        yield dut.epc.control.storage.eq(7)
        yield
        yield
        yield from dut.host.chipset.wr32(5, [0x12345678], first_be=0x4)
        yield from dut.host.chipset.rd32(9)
        assert dut.host.chipset.rd_data == [swap32(0xa1b2c3d4)]
        for _ in range(10):
            yield
        assert (yield dut.epc.bus.cyc) == 0
        assert (yield dut.epc.access_info.status) >> 31 == 0

    run_simulation(dut, dut.generators(stimulus(), firmware()))
    assert served == [(1, 0x4, 0x14, swap32(0x12345678), 1), (0, 0xf, 0x24, served[1][3], 1)]


@pytest.mark.parametrize("data_width,base", [(64, 0x00010000), (64, 0x2_00010000), (256, 0x00010000)])
def test_epc_outbound(data_width, base):
    dut = EPCDUT(data_width)
    bus = dut.epc.outbound_bus
    reads = []

    def stimulus():
        # The host model decodes the lower 32 bits of the address of a 64-bit request.
        dut.host.malloc(base & 0xffffffff, 0x2000)
        dut.host.chipset.enable()
        yield dut.epc.control.storage.eq(1)
        yield
        # Slot 2 is not mapped: nothing reaches the host.
        yield from bus.write(0x2000 >> 2, 0xdeadbeef)
        reads.append((yield from bus.read(0x2000 >> 2)))
        # Map slot 2 on the second page of the host memory.
        yield dut.epc.outbound_slot.storage.eq(2)
        yield dut.epc.outbound_base_lo.storage.eq(((base + 0x1000) & 0xfffff000) | 1)
        yield from csr_write(dut.epc.outbound_base_hi, base >> 32)
        yield from bus.write((0x2000 + 0x10) >> 2, 0x11223344)
        yield from bus.write((0x2000 + 0x14) >> 2, 0x55667788)
        # Only the selected byte of a Dword is written.
        yield from bus.write((0x2000 + 0x14) >> 2, 0xffaaffff, sel=0x4)
        reads.append((yield from bus.read((0x2000 + 0x10) >> 2)))
        reads.append((yield from bus.read((0x2000 + 0x14) >> 2)))
        # Unmap.
        yield dut.epc.outbound_base_lo.storage.eq(0)
        yield from csr_write(dut.epc.outbound_base_hi, 0)
        reads.append((yield from bus.read((0x2000 + 0x10) >> 2)))

    run_simulation(dut, dut.generators(stimulus()))
    assert reads == [0xffffffff, 0x11223344, 0x55aa7788, 0xffffffff]
    low = base & 0xffffffff
    assert dut.host.read_mem(low + 0x1010, 8) == [swap32(0x11223344), swap32(0x55aa7788)]
    assert dut.host.read_mem(low, 4) == [0]


def test_epc_msi():
    dut = EPCDUT()
    sent = []

    def host():
        msi = dut.host.phy.msi
        for _ in range(40):
            if (yield msi.valid):
                sent.append((yield msi.dat))
                yield msi.ready.eq(1)
                yield
                yield msi.ready.eq(0)
            yield

    def stimulus():
        yield dut.epc.control.storage.eq(1)
        yield
        yield from csr_write(dut.epc.msi, 3)
        assert (yield dut.epc.msi_status.status) == 1
        for _ in range(8):
            yield
        assert (yield dut.epc.msi_status.status) == 0

    run_simulation(dut, [stimulus(), host()])
    assert sent == [3]
