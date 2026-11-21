# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 LitePCIe contributors

import random

import pytest
from migen import ClockDomain, Memory
from migen.fhdl.simplify import MemoryToArray
from migen.sim import passive, run_simulation

from litex.soc.interconnect.stream import ClockDomainCrossing
from litepcie.phy.common import PHYTXDatapath, PHYRXDatapath
from litepcie.phy.gw5apciephy import GW5AFIFOStorage


@pytest.mark.parametrize("reverse_clocks", [False, True])
@pytest.mark.parametrize("storage", ["register", "pipeline"])
@pytest.mark.parametrize("depth", [4, 8, 16])
@pytest.mark.parametrize("width", [64, 256])
@pytest.mark.parametrize("direction", ["tx", "rx"])
def test_cdc_preserves_packets_under_backpressure(width, direction, depth, storage, reverse_clocks):
    if direction == "tx":
        path = PHYTXDatapath(width, width, "sys", cdc_depth=depth,
                             with_timing_buffers=(storage == "pipeline"))
        write_domain, read_domain = "sys", "pcie"
    else:
        path = PHYRXDatapath(width, width, "sys", cdc_depth=depth, with_timing_buffers=True)
        write_domain, read_domain = "pcie", "sys"
    path.clock_domains.cd_sys = ClockDomain("sys")
    path.clock_domains.cd_pcie = ClockDomain("pcie")
    cdc = next(module for _, module in path._submodules if isinstance(module, ClockDomainCrossing))
    fragment = (MemoryToArray() if storage == "register" else GW5AFIFOStorage())(path).get_fragment()
    assert not any(isinstance(special, Memory) for special in fragment.specials)
    expected = [((i + 1) * int("12345678" * (width // 32), 16) & ((1 << width) - 1),
                 15 if i % 7 == 6 else (1 << (width // 8)) - 1,
                 int(i % 7 == 0), int(i % 7 == 6)) for i in range(91)]
    received = []
    stalled_input = []

    def source():
        rng = random.Random(18)
        for data, be, first, last in expected:
            yield path.sink.valid.eq(0)
            for _ in range(rng.randrange(2)):
                yield
            yield path.sink.dat.eq(data)
            yield path.sink.be.eq(be)
            yield path.sink.first.eq(first)
            yield path.sink.last.eq(last)
            yield path.sink.valid.eq(1)
            yield
            while not (yield path.sink.ready):
                stalled_input.append(True)
                yield
        yield path.sink.valid.eq(0)
        for _ in range(2000):
            if len(received) == len(expected):
                return
            yield
        raise AssertionError("CDC did not drain")

    @passive
    def monitor():
        rng = random.Random(29)
        previous = None
        cycle = 0
        while True:
            valid, ready = (yield path.source.valid), (yield path.source.ready)
            word = ((yield path.source.dat), (yield path.source.be),
                    (yield path.source.first), (yield path.source.last))
            if previous is not None:
                assert valid
                assert word == previous
            if valid and ready:
                received.append(word)
            previous = word if valid and not ready else None
            yield path.source.ready.eq(cycle >= 300 and rng.randrange(3) != 0)
            cycle += 1
            yield

    @passive
    def watchdog():
        for _ in range(10000):
            yield
        raise AssertionError("CDC simulation timed out")

    periods = {"sys": (10, 2), "pcie": (34, 3)} if reverse_clocks else {"sys": (20, 2), "pcie": (10, 3)}
    run_simulation(fragment,
        {write_domain: [source(), watchdog()], read_domain: monitor()},
        clocks={**periods, f"from{cdc.duid}": periods[write_domain],
                f"to{cdc.duid}": periods[read_domain]})
    assert received == expected
    assert stalled_input, "The test must fill the CDC FIFO"
