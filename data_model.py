from dataclasses import dataclass


@dataclass
class ThermalSample:
    time_ms: int
    t_up: float
    t_down: float
    t_heater: float
    delta_t: float
    delta_t_corr: float
    heater_duty: int