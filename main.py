"""
Suzuki SDL protocol viewer for baleno G13BB 33920-65GP.
"""

# pylint: disable=too-many-positional-arguments,too-many-arguments,too-many-statements

import csv
import math
from datetime import datetime, timezone
from decimal import ROUND_HALF_UP, Decimal
from enum import Enum
from pathlib import Path
from time import sleep
from typing import Annotated, Literal

import typer
from rich import print as rich_print
from rich.columns import Columns
from rich.live import Live
from rich.table import Table
from serial import Serial

REFRESH_RATE = 0.1
TABLE_REFRESH_PER_SECOND = 2


class Actuate(str, Enum):
    """
    Actuator tests.
    """

    FIXED_SPARK = "FIXED_SPARK"
    ISC = "ISC"
    NONE = "NONE"


class ActuateValues(int, Enum):
    """
    Actuator tests.
    """

    FIXED_SPARK = 0x10
    ISC = 0xC0
    NONE = 0x00


def round_half_up(v: float):
    """
    Round up float using normal rounding.
    """
    return int(Decimal(v).to_integral_value(rounding=ROUND_HALF_UP))


class DisplayTable(str, Enum):
    """
    Table to display.
    """

    RAW = "RAW"
    PROCESSED_VALUES = "PROCESSED_VALUES"
    PROCESSED_FLAGS = "PROCESSED_FLAGS"
    ALL = "ALL"


class ScanToolParameters(str, Enum):
    """
    Parameters supported by scan tools.
    """

    # actual (current) intake air volume / maximum possible intake air volume * 100%
    DESIRED_IDLE = "Desired idle"
    ENGINE_SPEED = "Engine speed"
    IAC_FLOW_DUTY_CYCLE = "ISC flow duty"
    ABSOLUTE_THROTTLE_POSITION = "Absolute throttle position"
    INJ_PULSE_WIDTH_CYL_1 = "Inj. pulse width (#1 cylinder)"
    COOLANT_TEMP = "Coolant temperature"
    VEHICLE_SPEED = "Vehicle speed"
    INTAKE_AIR_TEMP = "Intake air temperature"
    MAP_VOLT = "MAP sensor voltage"
    MAP = "Manifold absolute pressure*"
    BAROMETRIC_PRESSURE = "Barometric Pressure"
    TP_SENSOR_VOLT = "TP sensor voltage"
    BATTERY_VOLTAGE = "Battery voltage"

    CLOSED_THROTTLE_POS = "Closed throttle position"
    ELECTRIC_LOAD = "Electric load"
    FUEL_CUT = "Fuel cut"
    AC_SWITCH = "A/C switch"
    PSP_SWITCH = "PSP switch"
    RADIATOR_FAN = "Radiator fan"
    IGNITION_ADVANCE = "Ignition advance"

    # CALC_LOAD = "CALC_LOAD"


class SDLHeader(int, Enum):
    """
    Supported headers for SDL for this program.
    """

    ECU_ID = 0x10
    DATA_REQUEST = 0x13
    ACTUATE = 0x15


class ObdParameter(str, Enum):
    """
    OBD parameters for SDL.
    """

    FAULT_CODES_1 = "FAULT_CODES_1"
    FAULT_CODES_2 = "FAULT_CODES_2"
    FAULT_CODES_3 = "FAULT_CODES_3"
    FAULT_CODES_4 = "FAULT_CODES_4"
    RPM_HIGH = "RPM_HIGH"
    RPM_LOW = "RPM_LOW"
    TARGET_IDLE = "TARGET_IDLE"
    VSS = "VSS"
    ECT = "ECT"
    IAT = "IAT"
    TPS_ANGLE = "TPS_ANGLE"
    TPS_VOLTAGE = "TPS_VOLTAGE"
    INJ_PULSE_WIDTH_HIGH = "INJ_PULSE_WIDTH_HIGH"
    INJ_PULSE_WIDTH_LOW = "INJ_PULSE_WIDTH_LOW"
    IGNITION_ADVANCE = "IGNITION_ADVANCE"
    MAP_SENSOR = "MAP_SENSOR"
    BAROMETRIC_PRESSURE = "BAROMETRIC_PRESSURE"
    ISC = "ISC"
    MIX_CONTROL_DWELL = "MIX_CONTROL_DWELL"
    MIX_CONTROL_LEARNING = "MIX_CONTROL_LEARNING"
    MIX_CONTROL_MONITOR = "MIX_CONTROL_MONITOR"
    BATTERY_VOLTAGE = "BATTERY_VOLTAGE"
    RADIATOR_FAN = "RADIATOR_FAN"
    STATUS_FLAGS_1 = "STATUS_FLAGS_1"
    FAULT_CODES_5 = "FAULT_CODES_5"
    FAULT_CODES_6 = "FAULT_CODES_6"


class ObdAddress(int, Enum):
    """
    OBD address of parameters.
    """

    FAULT_CODES_1 = 0x00
    FAULT_CODES_2 = 0x01
    FAULT_CODES_3 = 0x02
    FAULT_CODES_4 = 0x03
    RPM_HIGH = 0x04
    RPM_LOW = 0x05
    TARGET_IDLE = 0x06
    VSS = 0x07
    ECT = 0x08
    IAT = 0x09
    TPS_ANGLE = 0x0A
    TPS_VOLTAGE = 0x0B
    INJ_PULSE_WIDTH_HIGH = 0x0D
    INJ_PULSE_WIDTH_LOW = 0x0E
    IGNITION_ADVANCE = 0x0F
    MAP_SENSOR = 0x10
    BAROMETRIC_PRESSURE = 0x11
    ISC = 0x12
    MIX_CONTROL_DWELL = 0x13
    MIX_CONTROL_LEARNING = 0x14
    MIX_CONTROL_MONITOR = 0x15
    BATTERY_VOLTAGE = 0x16
    RADIATOR_FAN = 0x19
    STATUS_FLAGS_1 = 0x1A
    FAULT_CODES_5 = 0x20
    FAULT_CODES_6 = 0x21


class BalenoSDLInterface:
    """
    Handles serial port comms. for Baleno SDL.
    """

    serial_port: Serial  # interface with SDL
    raw_values: dict[ObdParameter, int]
    processed_values: dict[ScanToolParameters, tuple[str, str, str | None]]
    obd_parameters: list[ObdParameter]
    obd_addresses: list[ObdAddress]

    def _generate_checksum(self, input_bytes: bytes) -> bytes:
        """
        Create 8-bit checksum (2's compliment).
        """
        return bytes([-sum(input_bytes) & 0xFF])

    def _create_sdl_message(
        self,
        header: SDLHeader,
        data: bytes | None = None,
    ) -> bytes:
        """
        Create SDL message from provided params.

        Args:
            header: header byte
            data: Data byte(s) if any

        Returns:
            Complete SDL message
        """
        if not data:
            # Mostly for requesting ECU ID for now, could maybe be for
            # immobilizer too.
            input_bytes = bytes([header]) + bytes(
                [0x03]
            )  # 0x03 as [header, length, CS]
            checksum = self._generate_checksum(input_bytes)
            sdl_message = input_bytes + checksum
            return sdl_message

        input_bytes = bytes([header]) + bytes([1 + 1 + len(data) + 1]) + data
        checksum = self._generate_checksum(input_bytes)
        sdl_message = input_bytes + checksum
        return sdl_message

    def _open_port(self, serial_port: str, baud_rate: int, timeout: float) -> Serial:
        """
        Open port.
        """
        port = Serial(serial_port, baud_rate, timeout=timeout)
        return port

    def __init__(
        self,
        serial_port: str,
        baud_rate: int,
        timeout: float,
        obd_params: list[ObdParameter],
    ):
        self.obd_parameters = obd_params
        self.obd_addresses = [ObdAddress[obd_param] for obd_param in obd_params]
        self.raw_values = {obd_parameter: 0x00 for obd_parameter in obd_params}
        self.processed_values = {
            parameter: (str(0x00), "", None) for parameter in ScanToolParameters
        }
        self.serial_port = self._open_port(serial_port, baud_rate, timeout)

    def _read_message(self, echo_bytes: int) -> bytes:
        """
        Read on serial port.

        Args:
            echo_bytes: Number of bytes that are echoed back.

        Returns:
            Data from response SDL message.
        """

        echo = self.serial_port.read(echo_bytes)
        echo_header = echo[0]

        while True:
            byte_read = self.serial_port.read(1)

            if byte_read[0] != echo_header:
                raise RuntimeError(
                    "header byte of response should be matched to request"
                )

            header = byte_read
            length = self.serial_port.read(1)
            data = self.serial_port.read(length[0] - 3)
            checksum = self.serial_port.read(1)
            if sum(header + length + data + checksum) & 0xFF != 0:
                raise RuntimeError("ECU message checksum should be correct")

            return data

    def _write_message(self, write_bytes: bytes) -> int:
        write_result = self.serial_port.write(write_bytes)
        if write_result is None:
            raise RuntimeError("should have sent bytes over serial port")
        return write_result

    def get_ecu_id(self) -> bytes:
        """
        Request ECU ID.
        """
        sdl_message = self._create_sdl_message(SDLHeader.ECU_ID)
        write_result = self._write_message(sdl_message)
        return self._read_message(write_result)

    def _calculate_desired_idle(self) -> str:
        return f"{math.ceil(self.raw_values[ObdParameter.TARGET_IDLE] * 7.84375)}"

    def _calculate_boolean_flag(
        self, scan_tool_parameter: ScanToolParameters
    ) -> Literal["ON", "OFF"]:
        return_value = "OFF"

        match scan_tool_parameter:
            case _ if scan_tool_parameter == ScanToolParameters.PSP_SWITCH:
                raw_value = self.raw_values[ObdParameter.STATUS_FLAGS_1]
                return_value = "ON" if (raw_value & (1 << 1)) != 0 else "OFF"
            case _ if scan_tool_parameter == ScanToolParameters.AC_SWITCH:
                raw_value = self.raw_values[ObdParameter.STATUS_FLAGS_1]
                return_value = "ON" if (raw_value & (1 << 2)) != 0 else "OFF"
            case _ if scan_tool_parameter == ScanToolParameters.CLOSED_THROTTLE_POS:
                raw_value = self.raw_values[ObdParameter.STATUS_FLAGS_1]
                return_value = "ON" if (raw_value & (1 << 4)) != 0 else "OFF"
            case _ if scan_tool_parameter == ScanToolParameters.ELECTRIC_LOAD:
                raw_value = self.raw_values[ObdParameter.STATUS_FLAGS_1]
                return_value = "ON" if (raw_value & (1 << 6)) != 0 else "OFF"
            case _ if scan_tool_parameter == ScanToolParameters.RADIATOR_FAN:
                raw_value = self.raw_values[ObdParameter.RADIATOR_FAN]
                return_value = "ON" if raw_value == 128 else "OFF"
            # case _ if scan_tool_parameter == ScanToolParameters.OPERATING_TEMP:
            #    raw_value = self.raw_values[ObdParameter.ECT]
            #    value = round_half_up((raw_value / 255) * 159 - 40)
            #    op_range = [88, 100]
            #    return_value = (
            #        "ON" if value in range(op_range[0], op_range[1] + 1) else "OFF"
            #    )
            case _ if scan_tool_parameter == ScanToolParameters.FUEL_CUT:
                high_byte = self.raw_values[ObdParameter.INJ_PULSE_WIDTH_HIGH]
                low_byte = self.raw_values[ObdParameter.INJ_PULSE_WIDTH_LOW]
                return_value = "ON" if high_byte == 0 and low_byte == 0 else "OFF"
            case v:
                raise RuntimeError(f"{v} should be calculated to boolean")

        return return_value

    def _calculate_processed_values(self):
        for scan_tool_parameter, _ in self.processed_values.items():
            match scan_tool_parameter:
                case v if scan_tool_parameter == ScanToolParameters.ENGINE_SPEED:
                    rpm_value = math.ceil(
                        (
                            self.raw_values[ObdParameter.RPM_HIGH] * 256
                            + self.raw_values[ObdParameter.RPM_LOW]
                        )
                        / 5.1
                    )
                    self.processed_values[v] = (
                        f"{rpm_value}",
                        "RPM",
                        "red" if rpm_value < 500 or rpm_value > 6500 else None,
                    )
                case v if scan_tool_parameter == ScanToolParameters.DESIRED_IDLE:
                    self.processed_values[v] = (
                        self._calculate_desired_idle(),
                        "RPM",
                        None,
                    )
                case v if scan_tool_parameter in [
                    ScanToolParameters.PSP_SWITCH,
                    ScanToolParameters.AC_SWITCH,
                    ScanToolParameters.CLOSED_THROTTLE_POS,
                    ScanToolParameters.ELECTRIC_LOAD,
                    ScanToolParameters.RADIATOR_FAN,
                    ScanToolParameters.FUEL_CUT,
                    # ScanToolParameters.OPERATING_TEMP,
                ]:
                    value = self._calculate_boolean_flag(v)
                    self.processed_values[v] = (
                        value,
                        "",
                        "green" if value == "ON" else None,
                    )
                case v if scan_tool_parameter == ScanToolParameters.IAC_FLOW_DUTY_CYCLE:
                    value = str(
                        math.ceil((self.raw_values[ObdParameter.ISC] / 255) * 100)
                    )
                    self.processed_values[v] = (
                        f"{value}",
                        "%",
                        None,
                    )
                case v if scan_tool_parameter == ScanToolParameters.TP_SENSOR_VOLT:
                    raw_value = self.raw_values[ObdParameter.TPS_VOLTAGE]
                    processed_value = (raw_value / 255) * 5
                    value = f"{processed_value:.2f}"
                    self.processed_values[v] = (
                        value,
                        "V",
                        "green" if 0.3 < processed_value < 4.7 else "red",
                    )
                case v if scan_tool_parameter == ScanToolParameters.BATTERY_VOLTAGE:
                    raw_value = self.raw_values[ObdParameter.BATTERY_VOLTAGE]
                    processed_value = raw_value * 0.0787
                    value = f"{processed_value:.2f}"
                    style = None
                    engine_running = self.raw_values[ObdParameter.RPM_HIGH] > 0
                    if (engine_running and 13.6 < processed_value < 14.8) or (
                        not engine_running and processed_value > 12.4
                    ):
                        style = "green"
                    else:
                        style = "red"
                    self.processed_values[v] = (
                        value,
                        "V",
                        style,
                    )
                case v if scan_tool_parameter == ScanToolParameters.COOLANT_TEMP:
                    raw_value = self.raw_values[ObdParameter.ECT]
                    processed_value = round_half_up((raw_value / 255) * 159 - 40)
                    value = f"{processed_value}"
                    style = None
                    if 86 < processed_value < 101:
                        style = "green"
                    elif processed_value < 86:
                        style = "blue"
                    elif processed_value > 100:
                        style = "red"
                    self.processed_values[v] = (
                        value,
                        "C",
                        style,
                    )
                case v if scan_tool_parameter == ScanToolParameters.INTAKE_AIR_TEMP:
                    raw_value = self.raw_values[ObdParameter.IAT]
                    processed_value = round_half_up((raw_value / 255) * 159 - 40)
                    value = f"{processed_value}"
                    style = None
                    if -18 <= processed_value <= 12:
                        style = "blue"
                    elif 13 <= processed_value <= 32:
                        style = "green"
                    elif 33 <= processed_value <= 49:
                        style = "yellow"
                    elif processed_value > 50:
                        style = "red"
                    self.processed_values[v] = (
                        value,
                        "C",
                        style,
                    )
                case v if (
                    scan_tool_parameter == ScanToolParameters.INJ_PULSE_WIDTH_CYL_1
                ):
                    high_byte = self.raw_values[ObdParameter.INJ_PULSE_WIDTH_HIGH]
                    low_byte = self.raw_values[ObdParameter.INJ_PULSE_WIDTH_LOW]
                    value = f"{(((high_byte << 8) | low_byte) * 0.002):.2f}"
                    self.processed_values[v] = (
                        value,
                        "ms",
                        None,
                    )
                case v if scan_tool_parameter == ScanToolParameters.MAP:
                    raw_value = self.raw_values[ObdParameter.MAP_SENSOR]
                    value = f"{((raw_value / 255) * (146.63 - (-20)) + (-20)):.1f}"
                    self.processed_values[v] = (
                        value,
                        "kPa",
                        None,
                    )
                case v if scan_tool_parameter == ScanToolParameters.BAROMETRIC_PRESSURE:
                    raw_value = self.raw_values[ObdParameter.BAROMETRIC_PRESSURE]
                    value = f"{((raw_value / 255) * (146.63 - (-20)) + (-20)):.1f}"
                    self.processed_values[v] = (
                        value,
                        "kPa",
                        None,
                    )
                case v if scan_tool_parameter == ScanToolParameters.MAP_VOLT:
                    raw_value = self.raw_values[ObdParameter.MAP_SENSOR]
                    value = f"{((raw_value / 255) * 5):.2f}"
                    self.processed_values[v] = (
                        value,
                        "V",
                        None,
                    )
                case v if (
                    scan_tool_parameter == ScanToolParameters.ABSOLUTE_THROTTLE_POSITION
                ):
                    raw_value = self.raw_values[ObdParameter.TPS_VOLTAGE]
                    value = f"{round_half_up((raw_value / 255) * 100)}"
                    self.processed_values[v] = (
                        value,
                        "%",
                        None,
                    )
                case v if scan_tool_parameter == ScanToolParameters.VEHICLE_SPEED:
                    raw_value = self.raw_values[ObdParameter.VSS]
                    value = f"{raw_value}"
                    self.processed_values[v] = (
                        value,
                        "km/h",
                        None,
                    )
                case v if scan_tool_parameter == ScanToolParameters.IGNITION_ADVANCE:
                    raw_value = self.raw_values[ObdParameter.IGNITION_ADVANCE]
                    value = (
                        f"{round_half_up(((raw_value / 255) * (78 - (-12)) + (-12)))}"
                    )
                    self.processed_values[v] = (
                        value,
                        "BTDC",
                        None,
                    )
                case v:
                    raise RuntimeError(f"{v} should be handled")

    def actuate(self, actuator: Actuate, value: int | None):
        """
        Perform actuation.
        """

        if actuator == Actuate.ISC:
            if value is None:
                raise RuntimeError("ISC actuation value should be set")
            sdl_message = self._create_sdl_message(
                SDLHeader.ACTUATE,
                bytes(
                    [
                        ActuateValues[actuator.name].value,
                        value,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                    ]
                ),
            )
            write_result = self._write_message(sdl_message)
            read_result = self._read_message(write_result)
            if read_result.hex() != "":  # no data in OK response
                rich_print(read_result)
                raise RuntimeError(f"Unexpected actuator response: {read_result}")
            return

        if actuator == Actuate.FIXED_SPARK:
            sdl_message = self._create_sdl_message(
                SDLHeader.ACTUATE,
                bytes(
                    [
                        ActuateValues[actuator.name].value,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                    ]
                ),
            )
            write_result = self._write_message(sdl_message)
            read_result = self._read_message(write_result)
            if read_result.hex() != "":  # no data in OK response
                rich_print(read_result)
                raise RuntimeError(f"Unexpected actuator response: {read_result}")
            return

        if actuator == Actuate.NONE:
            sdl_message = self._create_sdl_message(
                SDLHeader.ACTUATE,
                bytes(
                    [
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                    ]
                ),
            )
            write_result = self._write_message(sdl_message)
            read_result = self._read_message(write_result)
            if read_result.hex() != "":
                raise RuntimeError(f"Unexpected actuator response: {read_result}")
            return

    def stream(self, display_table: DisplayTable, log_file_name: str | None):
        """
        Do data stream. Does not return. Keeps updating table always.
        """
        obd_addresses = self.obd_addresses

        if display_table == DisplayTable.RAW:
            table = build_raw_values_table(self.raw_values)
        elif display_table == DisplayTable.PROCESSED_VALUES:
            table = build_scan_tool_table(self.processed_values)
        elif display_table == DisplayTable.PROCESSED_FLAGS:
            table = build_scan_tool_flags_table(self.processed_values)
        else:
            table = Columns(
                [
                    build_raw_values_table(self.raw_values),
                    build_scan_tool_table(self.processed_values),
                    build_scan_tool_flags_table(self.processed_values),
                ]
            )

        ecu_id = self.get_ecu_id()
        rich_print(f"ECU ID: {ecu_id[0]:02d}{ecu_id[1]:02d}")

        with Live(
            table,
            refresh_per_second=TABLE_REFRESH_PER_SECOND,
            screen=False,
        ) as live:
            with open(
                (
                    Path(
                        Path.cwd(),
                        f"sdl_log_{datetime.now(timezone.utc).strftime("%y_%m_%d_%H_%M_%S")}.csv",
                    )
                    if not log_file_name
                    else log_file_name
                ),
                mode="w",
                encoding="UTF-8",
                newline="",
            ) as file:
                fieldnames = ["timestamp"] + [
                    key.value.replace(" ", "_")
                    .replace(".", "")
                    .replace("#", "")
                    .replace("(", "")
                    .replace(")", "")
                    .replace("/", "")
                    .lower()
                    for key in ScanToolParameters
                ]
                writer = csv.writer(file)
                writer.writerow(fieldnames)
                while True:
                    # create sdl message
                    sdl_message = self._create_sdl_message(
                        SDLHeader.DATA_REQUEST, bytes(obd_addresses)
                    )
                    # make a request for that obd address
                    write_result = self._write_message(sdl_message)
                    # read response for that obd address
                    data = self._read_message(write_result)

                    # Update raw values
                    for idx, obd_address in enumerate(obd_addresses):
                        self.raw_values[ObdParameter[obd_address.name]] = data[idx]

                    # Refresh table
                    if display_table == DisplayTable.RAW:
                        table = build_raw_values_table(self.raw_values)
                    elif display_table == DisplayTable.PROCESSED_VALUES:
                        self._calculate_processed_values()
                        table = build_scan_tool_table(self.processed_values)
                    elif display_table == DisplayTable.PROCESSED_FLAGS:
                        self._calculate_processed_values()
                        table = build_scan_tool_flags_table(self.processed_values)
                    else:
                        self._calculate_processed_values()
                        table = Columns(
                            [
                                build_raw_values_table(self.raw_values),
                                build_scan_tool_table(self.processed_values),
                                build_scan_tool_flags_table(self.processed_values),
                            ]
                        )
                    live.update(table)
                    row = [datetime.now(timezone.utc).isoformat()] + [
                        self.processed_values.get(key, (None, None, None))[0]
                        for key in ScanToolParameters
                    ]
                    writer.writerow(row)
                    sleep(REFRESH_RATE)


def build_raw_values_table(
    raw_values: dict[ObdParameter, int],
):
    """
    Table for displaying live SDL data.
    """
    table = Table(title="SDL live data (raw)")
    table.add_column("Address", style="white")
    table.add_column("Parameter", style="white")
    table.add_column("Value", style="yellow", justify="right")

    for obd_parameter, raw_value in raw_values.items():
        row_data = [
            f"{hex(ObdAddress[obd_parameter.name].value)}",
            f"{obd_parameter.name}",
            raw_value,
        ]
        table.add_row(*map(str, row_data))
    return table


def build_scan_tool_table(
    processed_values: dict[ScanToolParameters, tuple[str, str, str | None]],
):
    """
    Scan tool data formed from sdl live data.
    """

    table = Table(title="SDL live processed values", show_lines=True)
    table.add_column("Parameter", style="white")
    table.add_column("Value", style="yellow", justify="right")
    table.add_column("Unit", style="yellow", justify="right")

    for obd_parameter, (processed_value, unit, style) in processed_values.items():
        if processed_value in ["ON", "OFF"]:
            continue
        row_data = [obd_parameter.value, processed_value, unit]
        table.add_row(*map(str, row_data), style=style)

    return table


def build_scan_tool_flags_table(
    processed_values: dict[ScanToolParameters, tuple[str, str, str | None]],
):
    """
    Scan tool data formed from sdl live data.
    """

    table = Table(title="SDL live processed flags", show_lines=True)
    table.add_column("Flag", style="white")
    table.add_column("Value", style="yellow", justify="right")

    for obd_parameter, (processed_value, _, style) in processed_values.items():
        if processed_value not in ["ON", "OFF"]:
            continue
        row_data = [obd_parameter.value, processed_value]
        table.add_row(*map(str, row_data), style=style)

    return table


def main(
    obd_param: Annotated[list[ObdParameter] | None, typer.Option()] = None,
    serial_port: Annotated[str, typer.Option()] = "/dev/ttyUSB0",
    baud_rate: Annotated[int, typer.Option()] = 7812,
    timeout: Annotated[float, typer.Option()] = 1,
    table: Annotated[DisplayTable, typer.Option()] = DisplayTable.ALL,
    actuate: Annotated[Actuate | None, typer.Option()] = None,
    actuate_value: Annotated[int | None, typer.Option()] = None,
    log_file_name: Annotated[str | None, typer.Option()] = None,
    dtc: Annotated[bool, typer.Option()] = False,
    dtc_only: Annotated[bool, typer.Option()] = False,
):
    """
    Entry point.
    """
    if obd_param is not None and table != DisplayTable.RAW:
        rich_print("Single OBD parameter is only available on 'RAW' table")
        return

    # Request ALL obd addresses if none provided.
    if not obd_param:
        obd_param = list(ObdParameter)
        if not dtc:
            for i in [
                ObdParameter.FAULT_CODES_1,
                ObdParameter.FAULT_CODES_2,
                ObdParameter.FAULT_CODES_3,
                ObdParameter.FAULT_CODES_4,
                ObdParameter.FAULT_CODES_5,
                ObdParameter.FAULT_CODES_6,
            ]:
                obd_param.remove(i)
        if dtc and dtc_only:
            if dtc_only and table != DisplayTable.RAW:
                rich_print("Only select RAW table with dtc only")
                return
            obd_param = [
                ObdParameter.FAULT_CODES_1,
                ObdParameter.FAULT_CODES_2,
                ObdParameter.FAULT_CODES_3,
                ObdParameter.FAULT_CODES_4,
                ObdParameter.FAULT_CODES_5,
                ObdParameter.FAULT_CODES_6,
            ]

    sdl_interface = BalenoSDLInterface(serial_port, baud_rate, timeout, obd_param)
    if actuate:
        sdl_interface.actuate(actuate, actuate_value)
    sdl_interface.stream(table, log_file_name)


if __name__ == "__main__":
    typer.run(main)
# port = Serial("/dev/ttyUSB0", 7812, timeout=15)
# port.write(b'hello')
# read_data = port.read(4)
# print(read_data)
