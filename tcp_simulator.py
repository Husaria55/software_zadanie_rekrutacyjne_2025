import random
import sys
import time
import os
import yaml
from enum import Enum
import math

from communication_library.frame import ids, Frame
from communication_library.communication_manager import CommunicationManager, TransportType
from communication_library.exceptions import UnregisteredCallbackError

from communication_library.exceptions import TransportTimeoutError
from communication_library.tcp_transport import TcpSettings

from argparse import ArgumentParser

import logging


class SimulationState(Enum):
    IDLE = "IDLE"
    FILLING_OXIDIZER = "FILLING_OXIDIZER"
    OXIDIZER_FILLED = "OXIDIZER_FILLED"
    FILLING_FUEL = "FILLING_FUEL"
    FUEL_FILLED = "FUEL_FILLED"
    HEATING = "HEATING"
    READY_TO_LAUNCH = "READY_TO_LAUNCH"
    IGNITION_SEQUENCE = "IGNITION_SEQUENCE"
    FLIGHT = "FLIGHT"
    APOGEE = "APOGEE"
    PARACHUTE_DEPLOYED = "PARACHUTE_DEPLOYED"
    FREEFALL = "FREEFALL"
    EXPLOSION = "EXPLOSION"
    LANDED = "LANDED"


class StandaloneMock:
    def __init__(self, proxy_address: str,
                 proxy_port: int,
                 hardware_config: str,
                 feed_send_interval: float,
                 no_print: bool,
                 verbose: bool,
                 time_multiplier: float):
        
        with open(hardware_config, 'r') as config_file:
            self.config = yaml.safe_load(config_file)
        
        self.manager = CommunicationManager()
        self.manager.change_transport_type(TransportType.TCP)
        self.manager.connect(TcpSettings(address=proxy_address, port=proxy_port))
        self.setup_loggers()
        self._logger = logging.getLogger("main")
        self.feed_send_delay = feed_send_interval
        self.no_print = no_print
        self.verbose = verbose
        self.time_multiplier = time_multiplier
        self.last_feed_update = time.perf_counter()
        self.last_physics_update = time.perf_counter()
        self.last_status_print = time.perf_counter()
        self.should_run = True
        
        self.state = SimulationState.IDLE
        
        self.servos = {}
        for servo_name, servo_config in self.config['devices']['servo'].items():
            self.servos[servo_name] = servo_config['closed_pos']
        
        self.relays = {}
        for relay_name in self.config['devices']['relay'].keys():
            self.relays[relay_name] = 0
        
        self.sensors = {
            'fuel_level': 0.0,
            'oxidizer_level': 0.0,
            'altitude': 0.0,
            'oxidizer_pressure': 0.0,
            'angle': 2.0
        }
        
        self.oxidizer_filled = False
        self.fuel_filled = False
        self.fuel_main_open_time = None
        self.oxidizer_main_open_time = None
        self.igniter_start_time = None
        self.apogee_reached_time = None
        self.max_altitude = 0.0
        self.velocity = 0.0
        self.thrust_multiplier = 1.0
        self.flight_start_time = None

        # Physics parameters with default values similar to AGH Space Systems rocket SKYLARK. These can be adjusted via terminal dialog.
        # All units are SI unless stated otherwise.
        self.physics = {
            'm_initial': 57.0,          # Initial mass of rocket with propellant [kg]
            'm_dry': 44.3,              # Dry mass of rocket (no propellant) [kg]
            't_burn': 6.0,              # Engine burn time [s] (only guessed)
            'Isp': 140.0,               # Specific impulse [s] (only guessed)
            'A_ref': 0.02,            # Reference cross-sectional area [m^2]
            'C_d': 0.6,                 # Drag coefficient during powered/unpowered flight [-] (only guessed)
            'rho0': 1.225,              # Sea-level air density [kg/m^3]
            'H_scale': 8500.0,          # Atmospheric scale height [m]
            'g0': 9.80665,              # Standard gravity [m/s^2]
            'G': 6.67430e-11,           # Gravitational constant [m^3 kg^-1 s^-2]
            'M_earth': 5.97219e24,      # Mass of Earth [kg]
            'R_earth': 6_371_000.0,     # Radius of Earth [m]
            # Parachute model: increase drag area and coefficient when deployed
            'C_d_parachute': 5.0,       # Effective drag coefficient with parachute [-] (only guessed, to ensure some sensible landing speed)
            'A_parachute': 1          # Effective area with parachute deployed [m^2] (only guessed, to ensure some sensible landing speed)
        }

        # Ask user if they want to override defaults before simulation starts
        try:
            self.configure_physics_via_terminal()
        except Exception as e:
            # Fallback silently to defaults if stdin not interactive or any error occurs
            self._logger.warning(f"Physics configuration skipped: {e}")

        self._logger.info(
            f'Rocket simulator is running connected to {proxy_address}:{proxy_port}')
        self._logger.info(f'State: {self.state.value}')

    def setup_loggers(self):
        logger_main = logging.getLogger("main")
        logger_main.setLevel(logging.DEBUG)

        fmt = '[%(asctime)s] [%(levelname)s] %(message)s'
        log_formatter = logging.Formatter(fmt=fmt)

        console_handler = logging.StreamHandler(sys.stdout)

        console_handler.setFormatter(log_formatter)
        logger_main.addHandler(console_handler)

    def print_rocket_status(self):
        self._logger.info("=" * 60)
        self._logger.info("ROCKET STATUS:")
        self._logger.info(f"  State: {self.state.value}")
        self._logger.info(f"  Sensors:")
        self._logger.info(f"    - Fuel Level: {self.sensors['fuel_level']:.1f}%")
        self._logger.info(f"    - Oxidizer Level: {self.sensors['oxidizer_level']:.1f}%")
        self._logger.info(f"    - Oxidizer Pressure: {self.sensors['oxidizer_pressure']:.1f} bar")
        self._logger.info(f"    - Altitude: {self.sensors['altitude']:.1f} m")
        self._logger.info(f"    - Angle: {self.sensors['angle']:.1f}°")
        self._logger.info(f"  Servos:")
        for servo_name, position in self.servos.items():
            self._logger.info(f"    - {servo_name}: {position}")
        self._logger.info(f"  Relays:")
        for relay_name, state in self.relays.items():
            self._logger.info(f"    - {relay_name}: {'OPEN' if state else 'CLOSED'}")
        self._logger.info(f"  Velocity: {self.velocity:.2f} m/s")
        self._logger.info("=" * 60)

    def explode(self, reason: str):
        self.state = SimulationState.EXPLOSION
        self._logger.error(f'EXPLOSION: {reason}')
        self.print_rocket_status()
        self._logger.error('Simulation ended.')
        time.sleep(2)
        self.should_run = False

    def handle_frame(self, _frame) -> list[Frame]:
        output_frames = []
        handled = False
        
        if self.verbose:
            self._logger.info(f'Received frame: {_frame}')
        
        if _frame.device_type == ids.DeviceID.SERVO:
            servo_name = self.get_servo_name(_frame.device_id)
            if servo_name:
                if _frame.operation == ids.OperationID.SERVO.value.POSITION:
                    old_val = self.servos[servo_name]
                    new_position = int(_frame.data)
                    self.servos[servo_name] = new_position
                    
                    servo_config = self.config['devices']['servo'][servo_name]
                    open_pos = servo_config['open_pos']
                    closed_pos = servo_config['closed_pos']
                    
                    self._logger.info(f'{servo_name} position set to {new_position} (was {old_val})')
                    
                    if abs(new_position - open_pos) < abs(new_position - closed_pos):
                        if servo_name == 'fuel_main':
                            self.fuel_main_open_time = time.perf_counter()
                        elif servo_name == 'oxidizer_main':
                            self.oxidizer_main_open_time = time.perf_counter()
                    else:
                        if servo_name == 'fuel_main':
                            self.fuel_main_open_time = None
                        elif servo_name == 'oxidizer_main':
                            self.oxidizer_main_open_time = None
                    
                    handled = True
                else:
                    self._logger.warning(f'Unknown servo operation {_frame.operation} for {servo_name}')
            else:
                self._logger.warning(f'Unknown servo device_id {_frame.device_id}')
            
            if handled:
                replacements = {
                    'destination': _frame.source,
                    'source': _frame.destination,
                    'action': ids.ActionID.ACK
                }
                output_frames.append(Frame(**{**_frame.as_dict(), **replacements}))
        
        elif _frame.device_type == ids.DeviceID.RELAY:
            relay_name = self.get_relay_name(_frame.device_id)
            if relay_name:
                if _frame.operation == ids.OperationID.RELAY.value.OPEN:
                    old_val = self.relays[relay_name]
                    self.relays[relay_name] = 1
                    self._logger.info(f'{relay_name} relay opened (was {old_val}, now 1)')
                    
                    if relay_name == 'igniter':
                        self.igniter_start_time = time.perf_counter()
                    
                    handled = True
                        
                elif _frame.operation == ids.OperationID.RELAY.value.CLOSE:
                    old_val = self.relays[relay_name]
                    self.relays[relay_name] = 0
                    self._logger.info(f'{relay_name} relay closed (was {old_val}, now 0)')
                    
                    if relay_name == 'igniter':
                        self.igniter_start_time = None
                    
                    handled = True
                else:
                    self._logger.warning(f'Unknown relay operation {_frame.operation} for {relay_name}')
            else:
                self._logger.warning(f'Unknown relay device_id {_frame.device_id}')
            
            if handled:
                replacements = {
                    'destination': _frame.source,
                    'source': _frame.destination,
                    'action': ids.ActionID.ACK
                }
                output_frames.append(Frame(**{**_frame.as_dict(), **replacements}))
        
        else:
            self._logger.warning(f'Unknown device_type {_frame.device_type}')
        
        return output_frames

    def get_servo_name(self, device_id):
        for name, settings in self.config['devices']['servo'].items():
            if settings['device_id'] == device_id:
                return name
        return None
    
    def get_relay_name(self, device_id):
        for name, settings in self.config['devices']['relay'].items():
            if settings['device_id'] == device_id:
                return name
        return None

    def is_servo_open(self, servo_name: str) -> bool:
        servo_config = self.config['devices']['servo'][servo_name]
        open_pos = servo_config['open_pos']
        closed_pos = servo_config['closed_pos']
        current_pos = self.servos[servo_name]
        
        threshold = abs(open_pos - closed_pos) * 0.3
        return abs(current_pos - open_pos) < threshold

    def update_physics(self, dt: float):
        old_state = self.state
        
        if self.state == SimulationState.IDLE:
            if self.is_servo_open('fuel_intake'):
                self._logger.warning('PROPELLANT LOADING VIOLATION: Fuel intake opened before oxidizer is filled!')
                self._logger.warning('Correct procedure: Fill oxidizer tank first, then fuel tank.')
            elif self.is_servo_open('oxidizer_intake'):
                self.state = SimulationState.FILLING_OXIDIZER
                self._logger.info(f'State: {self.state.value}')
                self.print_rocket_status()
        
        elif self.state == SimulationState.FILLING_OXIDIZER:
            if self.is_servo_open('fuel_intake'):
                self._logger.warning('PROPELLANT LOADING VIOLATION: Fuel intake opened before oxidizer is fully filled!')
                self._logger.warning('Correct procedure: Complete oxidizer filling first.')
            
            if self.is_servo_open('oxidizer_intake'):
                self.sensors['oxidizer_level'] = min(100.0, self.sensors['oxidizer_level'] + dt * 10.0)
                self.sensors['oxidizer_pressure'] = min(40.0, self.sensors['oxidizer_pressure'] + dt * 2.0)
                
                if self.sensors['oxidizer_level'] >= 100.0:
                    self.state = SimulationState.OXIDIZER_FILLED
                    self._logger.info(f'State: {self.state.value}')
                    self.print_rocket_status()
            else:
                if self.sensors['oxidizer_level'] >= 100.0:
                    self.state = SimulationState.OXIDIZER_FILLED
                    self._logger.info(f'State: {self.state.value}')
                    self.print_rocket_status()
                else:
                    self.sensors['oxidizer_pressure'] = max(0.0, self.sensors['oxidizer_pressure'] - dt * 1.0)
        
        elif self.state == SimulationState.OXIDIZER_FILLED:
            if self.relays['oxidizer_heater'] == 1:
                self.sensors['oxidizer_pressure'] = min(90.0, self.sensors['oxidizer_pressure'] + dt * 2.5)
                if self.sensors['oxidizer_pressure'] >= 90.0:
                    self.explode("Oxidizer pressure too high (90 bars) - tank explosion")
                    return
            else:
                self.sensors['oxidizer_pressure'] = max(30.0, self.sensors['oxidizer_pressure'] - dt * 1.0)
            
            if self.is_servo_open('fuel_intake'):
                self.state = SimulationState.FILLING_FUEL
                self._logger.info(f'State: {self.state.value}')
                self.print_rocket_status()
        
        elif self.state == SimulationState.FILLING_FUEL:
            if self.relays['oxidizer_heater'] == 1:
                self.sensors['oxidizer_pressure'] = min(90.0, self.sensors['oxidizer_pressure'] + dt * 2.5)
                if self.sensors['oxidizer_pressure'] >= 90.0:
                    self.explode("Oxidizer pressure too high (90 bars) - tank explosion")
                    return
            else:
                self.sensors['oxidizer_pressure'] = max(30.0, self.sensors['oxidizer_pressure'] - dt * 1.0)
            
            if self.is_servo_open('fuel_intake'):
                self.sensors['fuel_level'] = min(100.0, self.sensors['fuel_level'] + dt * 10.0)
                
                if self.sensors['fuel_level'] >= 100.0:
                    self.state = SimulationState.FUEL_FILLED
                    self._logger.info(f'State: {self.state.value}')
                    self.print_rocket_status()
            else:
                if self.sensors['fuel_level'] >= 100.0:
                    self.state = SimulationState.FUEL_FILLED
                    self._logger.info(f'State: {self.state.value}')
                    self.print_rocket_status()
        
        elif self.state == SimulationState.FUEL_FILLED:
            if self.relays['oxidizer_heater'] == 1:
                self.sensors['oxidizer_pressure'] = min(90.0, self.sensors['oxidizer_pressure'] + dt * 2.5)
                if self.sensors['oxidizer_pressure'] >= 90.0:
                    self.explode("Oxidizer pressure too high (90 bars) - tank explosion")
                    return
            else:
                self.sensors['oxidizer_pressure'] = max(30.0, self.sensors['oxidizer_pressure'] - dt * 1.0)
            
            if self.fuel_main_open_time and self.oxidizer_main_open_time:
                time_diff = abs(self.fuel_main_open_time - self.oxidizer_main_open_time)
                if time_diff > 1.0:
                    if self.igniter_start_time:
                        self.explode("Main valves opened with >1s difference - propellant imbalance explosion")
                        return
                
                if self.igniter_start_time:
                    igniter_delay_fuel = abs(self.igniter_start_time - self.fuel_main_open_time)
                    igniter_delay_ox = abs(self.igniter_start_time - self.oxidizer_main_open_time)
                    
                    if igniter_delay_fuel > 1.0 or igniter_delay_ox > 1.0:
                        self.explode("Igniter started >1s after main valves - engine flooded")
                        return
                    
                    if self.igniter_start_time < min(self.fuel_main_open_time, self.oxidizer_main_open_time):
                        self.explode("Igniter started before main valves - single propellant combustion")
                        return
                    
                    if self.is_servo_open('fuel_intake') or self.is_servo_open('oxidizer_intake'):
                        self.explode("Intake valves still open during ignition - catastrophic pressure loss")
                        return
                    
                    pressure = self.sensors['oxidizer_pressure']
                    
                    if pressure < 40.0:
                        self._logger.error(f"Ignition failed: Oxidizer pressure too low ({pressure:.1f} bars) - engine won't ignite")
                        self.igniter_start_time = None
                        return
                    
                    if pressure > 65.0:
                        self.explode(f"Oxidizer pressure too high at ignition ({pressure:.1f} bars) - engine explosion")
                        return
                    
                    if 55.0 <= pressure <= 65.0:
                        self.thrust_multiplier = 1.0
                        self._logger.info(f"Optimal pressure {pressure:.1f} bars - full thrust!")
                    else:
                        pressure_deviation = min(abs(pressure - 55.0), abs(pressure - 65.0))
                        self.thrust_multiplier = max(0.5, 1.0 - (pressure_deviation / 15.0) * 0.5)
                        self._logger.warning(f"Suboptimal pressure {pressure:.1f} bars - thrust reduced to {self.thrust_multiplier*100:.0f}%")
                    
                    self.state = SimulationState.FLIGHT
                    # Mark the start of powered flight for mass/thrust integration
                    self.flight_start_time = time.perf_counter()
                    self._logger.info(f'State: {self.state.value} - Engine ignited successfully!')
                    self.print_rocket_status()
        
        elif self.state == SimulationState.FLIGHT:
            # Unified physics model for powered/coast ascent using Newton's 2nd law with variable mass.
            # Positive direction is upwards.
            if self.relays['parachute'] == 1:
                # Opening parachute under thrust risks structural failure
                self.explode("Parachute opened while engine is running - structural failure")
                return
            # Update simple tank indicators and pressure bleeding while engine runs
            burn_progress = 0.0
            if self.flight_start_time is not None:
                t_since_launch = max(0.0, time.perf_counter() - self.flight_start_time)
            else:
                t_since_launch = 0.0
            m_initial = self.physics['m_initial']
            m_dry = self.physics['m_dry']
            t_burn = max(1e-6, self.physics['t_burn'])
            Isp = self.physics['Isp']
            g0 = self.physics['g0']
            # Mass flow rate during burn
            m_propellant = max(0.0, m_initial - m_dry)
            mdot = m_propellant / t_burn if t_since_launch < t_burn else 0.0
            burn_progress = min(1.0, t_since_launch / t_burn)
            # Instantaneous mass
            m = max(m_dry, m_initial - (m_propellant * burn_progress))
            # Thrust only during burn; scaled by pressure-quality multiplier
            F_thrust = (mdot * Isp * g0) * self.thrust_multiplier
            # Gravity using inverse-square law
            R = self.physics['R_earth']
            M = self.physics['M_earth']
            G = self.physics['G']
            y = max(0.0, self.sensors['altitude'])
            r = R + y
            g_local = G * M / (r * r)
            F_gravity = m * g_local
            # Atmospheric density model with exponential decay
            rho0 = self.physics['rho0']
            H = self.physics['H_scale']
            rho = rho0 * math.exp(-max(0.0, y) / max(1.0, H))
            # Drag area and coefficient; boost if parachute state later
            C_d = self.physics['C_d']
            A = self.physics['A_ref']
            v = self.velocity
            # Aerodynamic drag opposes motion: F_d = -0.5 * rho * C_d * A * v * |v|
            F_drag = -0.5 * rho * C_d * A * v * abs(v)
            # Net force (upwards positive)
            F_net = F_thrust + F_drag - F_gravity
            # Integrate motion
            a = F_net / m
            self.velocity += a * dt
            self.sensors['altitude'] += self.velocity * dt
            # Update simple tanks/pressure visuals to keep UI meaningful
            if mdot > 0.0:
                # Convert mass burn to percentage indicators heuristically
                level_drop = (mdot * dt) / max(1e-6, m_propellant) * 100.0
                self.sensors['fuel_level'] = max(0.0, self.sensors['fuel_level'] - level_drop)
                self.sensors['oxidizer_level'] = max(0.0, self.sensors['oxidizer_level'] - level_drop)
                self.sensors['oxidizer_pressure'] = max(30.0, self.sensors['oxidizer_pressure'] - dt * 3.0)
            # Angle increases slowly during ascent
            self.sensors['angle'] = min(90.0, self.sensors['angle'] + dt * 5.0)
            # Apogee detection when vertical velocity crosses zero upward to downward
            if self.sensors['altitude'] > self.max_altitude:
                self.max_altitude = self.sensors['altitude']
            if self.velocity <= 0 and self.apogee_reached_time is None:
                self.apogee_reached_time = time.perf_counter()
                self.state = SimulationState.APOGEE
                self._logger.info(f'State: {self.state.value} - Maximum altitude: {self.sensors["altitude"]:.2f}m')
                self.print_rocket_status()

        elif self.state == SimulationState.APOGEE:
            time_since_apogee = time.perf_counter() - self.apogee_reached_time
            
            # After apogee, no thrust, only gravity and drag; allow parachute deployment to affect drag.
            self.sensors['angle'] = min(180.0, self.sensors['angle'] + dt * 20.0)
            if self.relays['parachute'] == 1:
                self.state = SimulationState.PARACHUTE_DEPLOYED
                self._logger.info(f'State: {self.state.value}')
                self.print_rocket_status()
            elif time_since_apogee > 10.0:
                self.state = SimulationState.FREEFALL
                self._logger.info(f'State: {self.state.value} - Parachute not deployed in time!')
                self.print_rocket_status()
            else:
                # Physics update with zero thrust
                m = self.physics['m_dry']
                R = self.physics['R_earth']
                M = self.physics['M_earth']
                G = self.physics['G']
                y = max(0.0, self.sensors['altitude'])
                r = R + y
                g_local = G * M / (r * r)
                F_gravity = m * g_local
                rho0 = self.physics['rho0']
                H = self.physics['H_scale']
                rho = rho0 * math.exp(-max(0.0, y) / max(1.0, H))
                C_d = self.physics['C_d']
                A = self.physics['A_ref']
                v = self.velocity
                F_drag = -0.5 * rho * C_d * A * v * abs(v)
                a = (0.0 + F_drag - F_gravity) / m
                self.velocity += a * dt
                self.sensors['altitude'] += self.velocity * dt
        
        elif self.state == SimulationState.PARACHUTE_DEPLOYED:
            # Increase drag dramatically to model parachute and integrate physics.
            m = self.physics['m_dry']
            R = self.physics['R_earth']
            M = self.physics['M_earth']
            G = self.physics['G']
            y = max(0.0, self.sensors['altitude'])
            r = R + y
            g_local = G * M / (r * r)
            F_gravity = m * g_local
            rho0 = self.physics['rho0']
            H = self.physics['H_scale']
            rho = rho0 * math.exp(-max(0.0, y) / max(1.0, H))
            C_d = self.physics['C_d_parachute']
            A = self.physics['A_parachute']
            v = self.velocity
            F_drag = -0.5 * rho * C_d * A * v * abs(v)
            a = (0.0 + F_drag - F_gravity) / m
            self.velocity += a * dt
            self.sensors['altitude'] += self.velocity * dt
            # Orient towards vertical during descent under canopy
            if self.sensors['angle'] > 0:
                self.sensors['angle'] = max(0.0, self.sensors['angle'] - dt * 30.0)
            elif self.sensors['angle'] < 0:
                self.sensors['angle'] = min(0.0, self.sensors['angle'] + dt * 30.0)
            
            if self.sensors['altitude'] <= 0:
                self.sensors['altitude'] = 0.0
                self.velocity = 0.0
                self.state = SimulationState.LANDED
                self._logger.info(f'State: {self.state.value} - Successful landing!')
                self.print_rocket_status()
                time.sleep(2)
                self.should_run = False
        
        elif self.state == SimulationState.FREEFALL:
            # Freefall with aerodynamic drag but no parachute.
            m = self.physics['m_dry']
            R = self.physics['R_earth']
            M = self.physics['M_earth']
            G = self.physics['G']
            y = max(0.0, self.sensors['altitude'])
            r = R + y
            g_local = G * M / (r * r)
            F_gravity = m * g_local
            rho0 = self.physics['rho0']
            H = self.physics['H_scale']
            rho = rho0 * math.exp(-max(0.0, y) / max(1.0, H))
            C_d = self.physics['C_d']
            A = self.physics['A_ref']
            v = self.velocity
            F_drag = -0.5 * rho * C_d * A * v * abs(v)
            a = (0.0 + F_drag - F_gravity) / m
            self.velocity += a * dt
            self.sensors['altitude'] += self.velocity * dt
            # Tumble towards downward orientation
            self.sensors['angle'] = min(180.0, self.sensors['angle'] + dt * 20.0)
            
            if self.relays['parachute'] == 1:
                if abs(self.velocity) > 30.0:
                    self._logger.error(f'Parachute deployed at too high velocity ({abs(self.velocity):.1f} m/s) - parachute ripped!')
                    self._logger.error('Continuing freefall...')
                else:
                    self.state = SimulationState.PARACHUTE_DEPLOYED
                    self._logger.info(f'State: {self.state.value} - Late parachute deployment successful')
                    self.print_rocket_status()
            
            if self.sensors['altitude'] <= 0:
                self.sensors['altitude'] = 0.0
                self.velocity = 0.0
                self.state = SimulationState.LANDED
                self._logger.error(f'State: {self.state.value} - CRASH LANDING!')
                self.print_rocket_status()
                time.sleep(2)
                self.should_run = False

    def configure_physics_via_terminal(self):
        """
        Simple terminal dialog to let the user adjust physics parameters before simulation starts.
        Empty input keeps default. Values are validated to be sensible (e.g., masses > 0, dry < initial, etc.).
        """
        if not sys.stdin or not sys.stdin.isatty():
            # Non-interactive environment; keep defaults
            return
        print("\n=== Rocket Physics Configuration ===")
        print("Press Enter to keep default shown in [brackets]. Units: SI.")
        # Query primary masses first
        m_initial = self._prompt_float("Initial mass m_initial [kg]", self.physics['m_initial'], min_val=0.1)
        m_dry = self._prompt_float("Dry mass m_dry [kg]", self.physics['m_dry'], min_val=0.1, max_val=m_initial - 0.01)
        t_burn = self._prompt_float("Burn time t_burn [s]", self.physics['t_burn'], min_val=0.01)
        Isp = self._prompt_float("Specific impulse Isp [s]", self.physics['Isp'], min_val=1.0)
        A_ref = self._prompt_float("Reference area A_ref [m^2]", self.physics['A_ref'], min_val=1e-5)
        C_d = self._prompt_float("Drag coefficient C_d [-]", self.physics['C_d'], min_val=0.0)
        rho0 = self._prompt_float("Sea-level density rho0 [kg/m^3]", self.physics['rho0'], min_val=0.1)
        H = self._prompt_float("Atmospheric scale height H [m]", self.physics['H_scale'], min_val=100.0)
        g0 = self._prompt_float("Standard gravity g0 [m/s^2]", self.physics['g0'], min_val=1.0)
        R_earth = self._prompt_float("Planet radius R [m]", self.physics['R_earth'], min_val=1000.0)
        M_earth = self._prompt_float("Planet mass M [kg]", self.physics['M_earth'], min_val=1e10)
        # Parachute parameters
        C_d_p = self._prompt_float("Parachute C_d [-]", self.physics['C_d_parachute'], min_val=0.1)
        A_p = self._prompt_float("Parachute area [m^2]", self.physics['A_parachute'], min_val=1e-3)
        # Assign validated values
        self.physics.update({
            'm_initial': m_initial,
            'm_dry': m_dry,
            't_burn': t_burn,
            'Isp': Isp,
            'A_ref': A_ref,
            'C_d': C_d,
            'rho0': rho0,
            'H_scale': H,
            'g0': g0,
            'R_earth': R_earth,
            'M_earth': M_earth,
            'C_d_parachute': C_d_p,
            'A_parachute': A_p,
        })
        m_propellant = m_initial - m_dry
        print(f"Computed propellant mass: {m_propellant:.2f} kg; mass flow rate: {m_propellant/t_burn:.2f} kg/s")
        print("====================================\n")

    def _prompt_float(self, label: str, default: float, min_val: float | None = None, max_val: float | None = None) -> float:
        """Prompt helper that enforces numeric input with optional bounds; Enter keeps default."""
        while True:
            inp = input(f"{label} [{default}]: ").strip()
            if inp == "":
                value = default
            else:
                try:
                    value = float(inp)
                except ValueError:
                    print("Invalid number. Try again.")
                    continue
            if min_val is not None and value < min_val:
                print(f"Value must be >= {min_val}.")
                continue
            if max_val is not None and value > max_val:
                print(f"Value must be <= {max_val}.")
                continue
            return value

    def send_feed_frame(self):
        conf_dict = self.config
        sensors_config: dict = conf_dict["devices"]["sensor"]

        for sensor_name, sensor_settings in sensors_config.items():
            source = ids.BoardID[sensor_settings["board"].upper()]
            device_id = sensor_settings["device_id"]
            data_type = ids.DataTypeID[sensor_settings["data_type"].upper()]
            
            if sensor_name in self.sensors:
                value = self.sensors[sensor_name]
            else:
                value = 0.0

            frame = Frame(destination=ids.BoardID.SOFTWARE,
                          priority=ids.PriorityID.LOW,
                          action=ids.ActionID.FEED,
                          source=source,
                          device_type=ids.DeviceID.SENSOR,
                          device_id=device_id,
                          data_type=data_type,
                          operation=ids.OperationID.SENSOR.value.READ,
                          payload=(value,))
            self.manager.push(frame)
            try:
                self.manager.send()
            except TransportTimeoutError:
                break

            if self.verbose:
                self._logger.info(f"sent feed frame: {frame}")

        servos_config: dict = conf_dict["devices"]["servo"]
        for servo_name, servo_settings in servos_config.items():
            source = ids.BoardID[servo_settings["board"].upper()]
            device_id = servo_settings["device_id"]
            data_type = ids.DataTypeID.INT16
            
            if servo_name in self.servos:
                value = int(self.servos[servo_name])
            else:
                value = 0

            frame = Frame(destination=ids.BoardID.SOFTWARE,
                          priority=ids.PriorityID.LOW,
                          action=ids.ActionID.FEED,
                          source=source,
                          device_type=ids.DeviceID.SERVO,
                          device_id=device_id,
                          data_type=data_type,
                          operation=ids.OperationID.SERVO.value.POSITION,
                          payload=(value,))
            self.manager.push(frame)
            try:
                self.manager.send()
            except TransportTimeoutError:
                break

            if self.verbose:
                self._logger.info(f"sent feed frame: {frame}")

    def receive_send_loop(self):
        while self.should_run:
            current_time = time.perf_counter()
            
            if current_time > self.last_physics_update + 0.1:
                dt = (current_time - self.last_physics_update) * self.time_multiplier
                self.update_physics(dt)
                self.last_physics_update = current_time
            
            if not self.verbose and current_time > self.last_status_print + 1.0:
                self.print_rocket_status()
                self.last_status_print = current_time
            
            try:
                frame = self.manager.receive()
            except TransportTimeoutError:
                if current_time > self.last_feed_update + float(self.feed_send_delay):
                    self.send_feed_frame()
                    self.last_feed_update = current_time
                continue
            except UnregisteredCallbackError as e:
                frame = e.frame
            except KeyboardInterrupt:
                sys.exit()

            for response_frame in self.handle_frame(frame):
                self.manager.push(response_frame)
                if self.verbose:
                    self._logger.info(f"pushed frame: {response_frame}")
                try:
                    self.manager.send()
                except TransportTimeoutError:
                    continue
            
            if current_time > self.last_feed_update + float(self.feed_send_delay):
                self.send_feed_frame()
                self.last_feed_update = current_time


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--proxy-address', default="127.0.0.1")
    parser.add_argument('--proxy-port', default=3001)
    parser.add_argument('--feed-interval', default=1)
    parser.add_argument('--hardware-config', default='simulator_config.yaml')
    parser.add_argument('--no-print', default=False, action='store_true')
    parser.add_argument('--verbose', default=False, action='store_true', 
                        help='Print all frames sent/received. If disabled, prints rocket status every second.')
    parser.add_argument('--time-multiplier', default=1.0, type=float,
                        help='Simulation speed multiplier. 1.0 = real-time, 2.0 = 2x faster, 0.5 = 2x slower.')
    cl_args = parser.parse_args()
    standalone_mock = StandaloneMock(cl_args.proxy_address,
                                     int(cl_args.proxy_port),
                                     cl_args.hardware_config,
                                     cl_args.feed_interval,
                                     cl_args.no_print,
                                     cl_args.verbose,
                                     cl_args.time_multiplier)
    standalone_mock.receive_send_loop()