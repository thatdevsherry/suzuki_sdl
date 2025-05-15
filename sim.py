"""
Suzuki Serial Data Line (SDL) simulator.
"""

from random import choice, randint
from time import sleep
from typing import Annotated

import typer
from rich import print as rich_print
from serial import Serial

from main import ObdAddress, ObdParameter


def generate_checksum(input_bytes: bytes) -> bytes:
    """
    Create 8-bit checksum (2's compliment).
    """
    return bytes([-sum(input_bytes) & 0xFF])


def generate_payload_response(
    payload: bytes, fixed: dict[ObdParameter, int] | None
) -> bytes:
    """
    .
    """
    response: list[int] = []
    for addr in payload:
        match addr:
            case _ if addr == ObdAddress.FAULT_CODES_1:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.FAULT_CODES_1.name])
                    if fixed
                    else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.FAULT_CODES_2:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.FAULT_CODES_2.name])
                    if fixed
                    else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.FAULT_CODES_3:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.FAULT_CODES_3.name])
                    if fixed
                    else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.FAULT_CODES_4:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.FAULT_CODES_4.name])
                    if fixed
                    else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.RPM_HIGH:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.RPM_HIGH.name]) if fixed else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.RPM_LOW:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.RPM_LOW.name]) if fixed else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.TARGET_IDLE:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.TARGET_IDLE.name])
                    if fixed
                    else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.VSS:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.VSS.name]) if fixed else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.ECT:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.ECT.name]) if fixed else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.IAT:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.IAT.name]) if fixed else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.TPS_ANGLE:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.TPS_ANGLE.name])
                    if fixed
                    else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.TPS_VOLTAGE:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.TPS_VOLTAGE.name])
                    if fixed
                    else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.INJ_PULSE_WIDTH_HIGH:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.INJ_PULSE_WIDTH_HIGH.name])
                    if fixed
                    else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.INJ_PULSE_WIDTH_LOW:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.INJ_PULSE_WIDTH_LOW.name])
                    if fixed
                    else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.IGNITION_ADVANCE:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.IGNITION_ADVANCE.name])
                    if fixed
                    else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.MAP_SENSOR:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.MAP_SENSOR.name])
                    if fixed
                    else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.BAROMETRIC_PRESSURE:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.BAROMETRIC_PRESSURE.name])
                    if fixed
                    else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.ISC:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.ISC.name]) if fixed else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.BATTERY_VOLTAGE:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.BATTERY_VOLTAGE.name])
                    if fixed
                    else None
                )
                response.append(
                    fixed_value if fixed_value is not None else randint(0, 255)
                )
            case _ if addr == ObdAddress.RADIATOR_FAN:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.RADIATOR_FAN.name])
                    if fixed
                    else None
                )
                response.append(
                    fixed_value if fixed_value is not None else choice([0, 128])
                )
            case _ if addr == ObdAddress.STATUS_FLAGS_1:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.STATUS_FLAGS_1.name])
                    if fixed
                    else None
                )
                base = 0b00000000
                bits_to_flip = [1, 2, 4, 6]
                for i in bits_to_flip:
                    if choice([True, False]):
                        base ^= 1 << i
                response.append(fixed_value if fixed_value is not None else base)
            case _ if addr == ObdAddress.FAULT_CODES_5:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.FAULT_CODES_5.name])
                    if fixed
                    else None
                )
                base = 0b00000000
                bits_to_flip = [0, 1, 2, 3, 4, 5, 6, 7]
                for i in bits_to_flip:
                    if choice([True, False]):
                        base ^= 1 << i
                response.append(fixed_value if fixed_value is not None else base)
            case _ if addr == ObdAddress.FAULT_CODES_6:
                fixed_value = (
                    fixed.get(ObdParameter[ObdAddress.FAULT_CODES_6.name])
                    if fixed
                    else None
                )
                base = 0b00000000
                bits_to_flip = [0, 1, 2, 3, 4, 5, 6, 7]
                for i in bits_to_flip:
                    if choice([True, False]):
                        base ^= 1 << i
                response.append(fixed_value if fixed_value is not None else base)
            case v:
                rich_print(f"WARNING: {hex(v)} param not handled, sending 0")
                response.append(0)
    return bytes(response)


def main(
    scanner_comms_serial_port: Annotated[str | None, typer.Argument()],
    fixed_param: Annotated[list[ObdParameter] | None, typer.Option()] = None,
    fixed_param_value: Annotated[list[int] | None, typer.Option()] = None,
    echo: Annotated[bool, typer.Option()] = False,
):
    """
    yo.
    """
    fixed = None
    if fixed_param is not None and fixed_param_value is not None:
        fixed = dict(zip(fixed_param, fixed_param_value))

    port = Serial(scanner_comms_serial_port, 7812, timeout=30)

    while True:
        sleep(0.05)
        rich_print("waiting for data\n")
        header = port.read(1)
        if not header:
            continue

        header = header[0]
        length = port.read(1)

        match header:
            case header if header == 0x10:
                data = port.read(length[0] - 3)
                checksum = port.read(1)
                rich_print("Received")
                rich_print((bytes([header]) + length + data + checksum).hex(" "))
                if sum(bytes([header]) + length + data + checksum) & 0xFF != 0:
                    raise RuntimeError("Invalid checksum")
                if echo:
                    _ = port.write(bytes([header]) + length + data + checksum)  # echo

                data_without_checksum = bytes([0x10, 0x05, 0x19, 0x43])
                response = data_without_checksum + generate_checksum(
                    data_without_checksum
                )
                _ = port.write(response)
                if not echo:
                    _ = port.read(len(response))  # read own echo
                rich_print(f"Responded with: {response.hex(" ")}\n")
            case header if header == 0x15:
                data = port.read(length[0] - 3)
                checksum = port.read(1)
                rich_print("Received")
                rich_print((bytes([header]) + length + data + checksum).hex(" "))
                if sum(bytes([header]) + length + data + checksum) & 0xFF != 0:
                    raise RuntimeError("Invalid checksum")
                if echo:
                    _ = port.write(bytes([header]) + length + data + checksum)  # echo
                response = bytes([0x15, 0x03, 0xE8])
                _ = port.write(response)
                if not echo:
                    _ = port.read(len(response))  # read own echo
                rich_print(f"Responded with: {response.hex(" ")}\n")
            case header if header == 0x13:
                data = port.read(length[0] - 3)
                checksum = port.read(1)
                rich_print("Received")
                rich_print((bytes([header]) + length + data + checksum).hex(" "))
                if sum(bytes([header]) + length + data + checksum) & 0xFF != 0:
                    rich_print("WARNING: Invalid checksum")
                    sleep(1)
                    continue
                payload_response = generate_payload_response(data, fixed)
                data_without_checksum = (
                    bytes([header, 1 + 1 + len(payload_response) + 1])
                    + payload_response
                )
                response = data_without_checksum + generate_checksum(
                    data_without_checksum
                )
                if echo:
                    _ = port.write(bytes([header]) + length + data + checksum)  # echo
                _ = port.write(response)
                if not echo:
                    _ = port.read(len(response))  # read own echo
                rich_print("\nResponse")
                rich_print(f"{response.hex(' ')}\n")
            case header:
                rich_print(f"Unknown header: {hex(header)}")
                data_and_checksum_skip = port.read(length[0] - 2)
                rich_print(f"skipped: {data_and_checksum_skip.hex(" ")}")
                continue


if __name__ == "__main__":
    typer.run(main)
