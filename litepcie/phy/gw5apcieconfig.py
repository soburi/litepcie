#
# This file is part of LitePCIe.
#
# Copyright (c) 2026 LitePCIe contributors
# SPDX-License-Identifier: BSD-2-Clause

"""Generate GW5AST-138 Gen3 x4 configuration using installed Gowin EDA assets.

The Gowin module generator only emits the wrapper. Place and route also needs
set_csr with the SerDes initialization sequence and PCIe configuration writes.
"""

import argparse
import configparser
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tomllib


def generate(gowin_home, ipc_path, output):
    ide = Path(gowin_home).resolve() / "IDE"
    ipc = configparser.ConfigParser(interpolation=None)
    ipc.read(ipc_path)
    section = next(s for s in ipc.sections() if s.startswith("PCI%20Express%20Controller"))
    pcie = ipc[section]
    if (not ipc["General"]["target_device"].startswith("gw5ast138b-") or
        pcie["Lane_Width"] != "X4" or pcie["Maximum_Link_Speed_8"] != "true" or
        pcie["Reference_Clock_Source"] != "Q0 REFCLK0" or
        pcie["Reference_Clock_Frequency"] != "100MHz"):
        raise ValueError("Only GW5AST-138B, Gen3 x4, Q0 REFCLK0 at 100 MHz is supported.")

    # PCIe exclusively owns Q0; the protocol template configures its lanes.
    # Generate the shared routing/reset prefix with all custom PHYs disabled.
    template = ide / "data/examples/jesd204b_demo/src/serdes/serdes.toml"
    cfg = tomllib.loads(template.read_text())
    lines = ['device = "GW5AST-138"', '[regulator]', 'regulator_enable = false']
    for quad in ("q0", "q1"):
        values = {k: v for k, v in cfg["q0"].items() if not isinstance(v, dict)}
        values.update(enable=False, por_toggle_by_fabric=False, pd_toggle_by_fabric=False,
            cmu0_reset_by_fabric=False, cmu1_reset_by_fabric=False, rx_eq_bias=7)
        lines += [f"[{quad}]"] + [f"{k} = {json.dumps(v)}" for k, v in values.items()]
    for quad in ("q0", "q1"):
        for lane in range(4):
            values = copy.deepcopy(cfg["q0"]["ln0"])
            values.update(enable=False, cpll_reset_by_fabric=False, locked_from_fabric=False)
            for key in ("chbond_mst_sel", "ctc_mst_sel", "tx_if_cfg_mst_sel"):
                values[key] = f"{quad}.ln{lane}"
            lines += [f"[{quad}.ln{lane}]"] + [f"{k} = {json.dumps(v)}" for k, v in values.items()]

    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    toml_path = output.with_suffix(".toml")
    toml_path.write_text("\n".join(lines) + "\n")
    converter = ide / "bin/serdes_toml_to_csr.dist/serdes_toml_to_csr_138k.bin"
    try:
        subprocess.run([str(converter), str(toml_path), "-o", str(output)], check=True)
    except subprocess.CalledProcessError as e:
        # Seen with Gowin EDA 1.9.12.04, which needs settings its own template does not have.
        raise OSError(f"{converter} failed on its own template; generate the configuration "
            "with a Gowin EDA version that works (1.9.12.03 does) and pass it to the PHY.") from e
    # These are the configuration addresses emitted by Gowin's PCIe generator.
    # Read all values from the same IPC as the synthesized controller wrapper.
    reference = ide / "ipcore/SERDES_IP/IPlib/PCIE/csrDoc/pcie_gen3_x4.csr"
    writes = [
        (0xb000c8, int(pcie["Device_ID"] + pcie["Vendor_ID"], 16)),
        (0xb000cc, int(pcie["Base_Class_Value"] + pcie["Sub_Class_Value"] + "00" +
            pcie["Revision_ID"], 16)),
    ]
    for bar in range(6):
        value = int(pcie[f"value{bar}"], 16)
        if pcie.getboolean(f"Bar{bar}_Enable"):
            value |= 4 if pcie.getboolean(f"Bit64_{bar}") else 0
            value |= 8 if pcie.getboolean(f"Prefetchable{bar}") else 0
        writes.append((0xb00048 + 4*bar, value))
    payload = int(pcie["Max_Payload_Size"].removesuffix("bytes"))
    if payload not in (128, 256, 512, 1024, 2048, 4096):
        raise ValueError("Invalid PCIe payload size.")
    writes.append((0xb001d8, payload.bit_length() - 8))
    # The controller defaults (K_PCICONF0 and K_SRIOV0 of GTR12_UPAR in hardcore.xml, packed
    # from 0xb00038) expose MSI-X and SR-IOV VF BARs the generator never configures.
    if not pcie.getboolean("Enable_MSI_Capability"):
        raise ValueError("Only the MSI configuration is supported.")
    writes.append((0xb000d8, 0x00000051)) # Default 0x80000051: bit 31 is MSI-X.
    for address in range(0xb0015c, 0xb00178, 4): # VF BARs, 16 VFs of 1 MiB and 64 KiB.
        writes.append((address, 0x00000000))
    # The template leaves the reference clock of the quad unselected. These are the values
    # Gowin's generator emits for Q0 REFCLK0 at 100 MHz; without them the PLL of the quad
    # has no reference and the LTSSM stays in Detect.Quiet.
    template_csr = reference.read_text().rstrip() + "\n"
    for address, unselected, selected in (
        (0xc00008, 0x00000000, 0x00000003),
        (0x808104, 0x00022203, 0x00022233),
        (0x808000, 0x00062203, 0x00062233),
    ):
        old = f"upar_write_driver(0x{address:06x},0x{unselected:08x})\n"
        new = f"upar_write_driver(0x{address:06x},0x{selected:08x})\n"
        if template_csr.lower().count(old) != 1:
            raise ValueError(f"Unexpected reference clock setting at 0x{address:06x} in {reference}.")
        index = template_csr.lower().index(old)
        template_csr = template_csr[:index] + new + template_csr[index + len(old):]
    # Gowin's generator also drops the last write of the template, which leaves 0xb00030 at
    # 0x003f3fff; with 0x003f3f0f the controller passes no TLP to the fabric.
    last = "upar_write_driver(0xb00030,0x003f3f0f)\n"
    if not template_csr.lower().endswith(last):
        raise ValueError(f"Unexpected last write in {reference}.")
    template_csr = template_csr[:-len(last)]
    with output.open("a") as f:
        f.write(template_csr)
        for address, value in writes:
            f.write(f"upar_write_driver(0x{address:06x},0x{value:08X})\n")
    output.with_suffix(".json").write_text(json.dumps({
        "ipc": str(Path(ipc_path).resolve()),
        "gowin_home": str(ide.parent),
        "inputs_sha256": {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (Path(ipc_path), template, reference, converter)},
        "output_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
    }, indent=2) + "\n")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gowin-home", default=os.environ.get("GOWIN_HOME"), required=False)
    parser.add_argument("--ipc", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.gowin_home is None:
        parser.error("Specify --gowin-home or GOWIN_HOME.")
    print(generate(args.gowin_home, args.ipc, args.output))


if __name__ == "__main__":
    main()
