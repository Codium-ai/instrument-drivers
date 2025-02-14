# This Device Class is published under the terms of the MIT License.
# Required Third Party Libraries, which are included in the Device Class
# package for convenience purposes, may have a different license. You can
# find those in the corresponding folders or contact the maintainer.
#
# MIT License
#
# Copyright (c) 2024 SweepMe! GmbH (sweep-me.net)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


# SweepMe! driver
# * Module: Logger
# * Instrument: Rohde&Schwarz RT-Series (RTA, RTB, ...)

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any, TypeVar

from pysweepme.EmptyDeviceClass import EmptyDevice

T = TypeVar("T")


class Device(EmptyDevice):
    """Logger driver to read out MEAS channels of Rohde&Schwarz Oscilloscope RT-Series (RTA, RTB, ...)."""

    description = """
                    <h3>Rohde & Schwarz Oscilloscope Meas Slots</h3>
                    <p>This driver reads out the measurement 'places' of Rohde & Schwarz RT Oscilloscopes. Measurement
                    places are measurements such as as Maximum peak height, periodicity, rise time, and much more. The
                    measurement places can be definied on the device or via SweepMe! for given mode and source
                    (e.g. Channel 1-4).
                    <p>The driver has been tested with RTB2004, but should also work with other RT-Series devices.</p>
                    </p>
                    <p>Setup:</p>
                    <ul>
                    <li>Enable remote control at device.</li>
                    <li>If use preset, the defined measurement places from the devices are used. Note that currently
                    SweepMe! is unable to display the used measurement mode and units in preset mode.</li>
                    <li>Waveform count: Number of waveforms used to average. Set to 1 to retrieve current values.</li>
                    </ul>
                    """

    def __init__(self) -> None:
        """Initialize the device class and the instrument parameters."""
        super().__init__()

        self.shortname = "RTX"

        # Communication Parameters
        self.port_manager = True
        self.port_types = ["GPIB", "TCPIP", "USB"]
        self.port_properties = {
            "timeout": 20,  # higher timeout if noise is measured, some modes need more time
        }

        # SweepMe return parameters
        self.variables = []
        self.units = []
        self.plottype = []
        self.savetype = []

        # Device parameters
        self.modes = {
            "None": "NONE",
            "Frequency in Hz": "FREQ",
            "Period time in s": "PER",
            "Peak to peak in V": "PEAK",
            "Maximum peak in V": "UPE",
            "Minimum peak in V": "LPE",
            "Positive pulse count": "PPC",
            "Negative pulse count": "NPC",
            "Rising edge count": "REC",
            "Falling edge count": "FEC",
            "High reference in V": "HIGH",
            "Low reference in V": "LOW",
            "Amplitude in V": "AMPL",
            "Mean in V": "MEAN",
            "RMS in V": "RMS",
            "Rise time in s": "RTIM",
            "Fall time in s": "FTIM",
            "Positive duty cycle in %": "PDCY",
            "Negative duty cycle in %": "NDCY",
            "Positive pulse width in s": "PPW",
            "Negative pulse width in s": "NPW",
            "Cycle mean in V": "CYCM",
            "Cycle RMS in V": "CYCR",
            "STDDev": "STDD",
            "CYCStddev": "CYCS",
            "Delay in s": "DEL",
            "Phase difference in °": "PHAS",
            "Burst width in s": "BWID",
            "Positive overshoot in %": "POV",
            "Negative overshoot in %": "NOV",
        }
        self.statistic_modes = [
            "None",
            "Current",
            "Minimum",
            "Maximum",
            "Average",
        ]

        # Measurement places - The devices can have up to 6 measurement 'places' with predefined modes and sources to
        # read out.
        self.maximum_measurement_places: int = 6  # For RTB2004, might differ for other devices
        self.measurement_places: dict[int, tuple[int, str, str]] = {}
        """A dictionary with the place number as key and a tuple of channel, mode code, and statistics as value."""

        # If True, the measurement places are defined on the device and used as is. If False, the places are defined in
        # the GUI. This mode is currently disabled as SweepMe cannot update the number of variables during runtime, so
        # the variables and units are always empty strings.
        self.use_preset: bool = False
        """Currently not used. If True, the measurement places are defined on the device and used as is."""

        self.waveform_count: int = 1
        """Number of waveform to average. If set to 1, no averaging is performed."""

    @staticmethod
    def parse_parameter(func: Callable[[Any], T], parameters: dict[str, Any], key: str, fallback: T) -> T:
        """Parse and convert a parameter from the given dictionary and return the value or a fallback value."""
        try:
            return func(parameters[key])
        except Exception:
            return fallback

    def update_gui_parameters(self, parameters: dict[str, Any]) -> dict[str, Any]:
        """Returns a dictionary with keys and values to generate GUI elements in the SweepMe! GUI."""
        # The number of places can be set by the user
        number_of_places = self.parse_parameter(int, parameters, "Number of places", self.maximum_measurement_places)

        new_parameters = {
            # "Use preset": False,  # Do not offer this option for now
            "Number of places": list(range(1, self.maximum_measurement_places + 1)),
            "Waveform count": 1,
        }
        # Each measurement place can be configured with a channel and a mode
        for place in range(1, number_of_places + 1):
            new_parameters[" " * place] = None  # empty line
            new_parameters[f"Place {place} channel"] = ["None", "CH1", "CH2", "CH3", "CH4"]
            current_channel = parameters.get(f"Place {place} channel", "None")
            if current_channel != "None":
                new_parameters[f"Place {place} mode"] = list(self.modes.keys())
                new_parameters[f"Place {place} statistics"] = self.statistic_modes

        return new_parameters

    def apply_gui_parameters(self, parameters: dict[str, Any]) -> None:
        """Receive the values of the GUI parameters that were set by the user in the SweepMe! GUI."""
        # self.use_preset = bool(parameters["Use preset"])
        self.waveform_count = self.parse_parameter(int, parameters, "Waveform count", 1)

        self.measurement_places = {}  # Reset measurement places

        # Must reset
        self.variables = []
        self.units = []
        self.plottype = []
        self.savetype = []

        if self.use_preset:
            # Define placeholder variables, because SweepMe! currently does not allow updating variables during runtime
            self.variables = [f"Place {n}" for n in range(1, self.maximum_measurement_places + 1)]
            self.units = [""] * self.maximum_measurement_places
            self.plottype = [True] * self.maximum_measurement_places
            self.savetype = [True] * self.maximum_measurement_places

        else:
            number_of_places = self.parse_parameter(
                int, parameters, "Number of places", self.maximum_measurement_places,
            )
            for place in range(1, number_of_places + 1):
                channel = parameters.get(f"Place {place} channel", "None")
                mode = parameters.get(f"Place {place} mode", "None")
                statistics = parameters.get(f"Place {place} statistics", "None")
                if channel != "None" and mode != "None":
                    channel_num = int(channel[-1])
                    self.measurement_places[place] = (channel_num, self.modes[mode], statistics)

                    mode_short = mode.split(" in ")[0]
                    self.variables.append(f"{channel} {mode_short}")
                    self.units.append(mode.split(" in ")[-1])
                    self.plottype.append(True)
                    self.savetype.append(True)

    def initialize(self) -> None:
        """Initialize the device. This function is called only once at the start of the measurement."""
        # do not use "SYST:PRES" as it will destroy all settings which is in conflict with using 'As is'
        self.port.write("*CLS")

    def connect(self) -> None:
        """Connect to the device. This function is called only once at the start of the measurement."""
        if self.use_preset:
            self.measurement_places = {}
            for place in range(1, self.maximum_measurement_places + 1):
                mode = self.get_measurement_mode(place)

                if mode != "NONE":
                    source = int(self.get_source(place))
                    self.measurement_places[place] = (source, mode, "Average")
                    # If SweepMe! allows updating variables during runtime, this could be done here

    def configure(self) -> None:
        """Configure the device. This function is called every time the device is used in the sequencer."""
        if not self.use_preset:
            for place, (channel, mode, _) in self.measurement_places.items():
                self.set_source(place, channel)
                self.set_measurement_mode(place, mode)

        if self.waveform_count > 1 or self.use_preset:
            # Enable statistical evaluation for all places. Place number is irrelevant.
            self.port.write("MEAS1:STAT:ENAB ON")

    def measure(self) -> None:
        """Reset the averaged values at the start of the measurement."""
        if self.waveform_count > 1 or self.use_preset:
            for place in self.measurement_places:
                self.reset_statistical_measurement(place)

    def request_result(self) -> None:
        """Wait until the given number of waveforms are acquired."""
        if self.waveform_count > 1:
            for place in self.measurement_places:
                while self.get_waveform_count(place) < self.waveform_count:
                    time.sleep(0.1)

    def call(self) -> list[float]:
        """'call' is a mandatory function that must be used to return as many values as defined in self.variables.

        This function can only be omitted if no variables are defined in self.variables.
        """
        measured_results = []
        for place, (_, mode, statistics) in self.measurement_places.items():
            if statistics == "Minimum":
                result = self.get_measurement_minimum(place)
            elif statistics == "Maximum":
                result = self.get_measurement_maximum(place)
            elif statistics == "Average":
                result = self.get_averaged_measurement(place)
            elif statistics == "Current":
                result = self.get_measurement(place, mode)
            else:
                # If no statistics are defined, return the current measurement
                result = self.get_measurement(place, mode)

            measured_results.append(result)

        # If the preset uses less than the maximum number of places, fill the measured results with None as placeholder
        if self.use_preset and len(measured_results) < self.maximum_measurement_places:
            diff = self.maximum_measurement_places - len(measured_results)
            measured_results += [float("nan")] * diff

        return measured_results

    """ Wrapped Functions """

    def set_source(self, place: int, channel: int) -> None:
        """Select the source channel for the measurement place."""
        self.port.write(f"MEAS{place}:SOUR CH{channel}")

    def get_source(self, place: int) -> str:
        """Read out the source of the given place."""
        self.port.write(f"MEAS{place}:SOUR?")
        return self.port.read()

    def get_measurement_mode(self, place: int) -> str:
        """Read out the measurement mode of the given place."""
        self.port.write(f"MEAS{place}:MAIN?")
        return self.port.read()

    def set_measurement_mode(self, place: int, mode: str) -> None:
        """Select the measurement mode for the measurement place."""
        self.port.write(f"MEAS{place}:MAIN {mode}")

        # activate measurement
        self.port.write(f"MEAS{place}:ENAB ON")

    def reset_statistical_measurement(self, place: int) -> None:
        """Deletes the statistical results of the indicated measurement.

        Starts a new statistical evaluation if the acquisition is running.
        """
        self.port.write(f"MEAS{place}:STAT:RES")

    def get_waveform_count(self, place: int) -> int:
        """Read out the number of waveforms used for averaging."""
        self.port.write(f"MEAS{place}:RES:WFMCount?")
        return int(self.port.read())

    def get_measurement(self, place: int, mode: str) -> float:
        """Read out the measurement value of the given place."""
        self.port.write(f"MEAS{place}:RES:ACT? {mode}")
        return self.read_result_and_handle_error()

    def get_averaged_measurement(self, place: int) -> float:
        """Read out the averaged measurement value of the given place."""
        self.port.write(f"MEAS{place}:RES:AVG?")
        return self.read_result_and_handle_error()

    def get_measurement_minimum(self, place: int) -> float:
        """Read out the minimum measurement result of the current measurement series."""
        self.port.write(f"MEAS{place}:RES:NPE?")  # Negative Peak
        return self.read_result_and_handle_error()

    def get_measurement_maximum(self, place: int) -> float:
        """Read out the maximum measurement result of the current measurement series."""
        self.port.write(f"MEAS{place}:RES:PPE?")
        return self.read_result_and_handle_error()

    def read_result_and_handle_error(self) -> float:
        """Read out the buffer and handle the error code."""
        ret = self.port.read()
        # 9.91E+37 is the error code
        return float(ret) if ret != "9.91E+37" else float("nan")
