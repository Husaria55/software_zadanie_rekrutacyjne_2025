from communication_library.communication_manager import CommunicationManager, TransportType
from communication_library.tcp_transport import TcpSettings
from communication_library.frame import Frame
from communication_library import ids
from communication_library.exceptions import TransportTimeoutError, TransportError, UnregisteredCallbackError

import matplotlib.pyplot as plt

#Functions for handling flight parameters

def on_altitude(frame: Frame):
    global altitude_value
    altitude_value = frame.data
    print(f"altitude FEED: {altitude_value} m")


def on_oxidizer_level(frame: Frame):
    global oxidizer_level_value
    oxidizer_level_value = frame.data
    print(f"oxidizer_level FEED: {oxidizer_level_value}%")


def on_angle(frame: Frame):
    global angle_value
    angle_value = frame.data
    print(f"angle FEED: {angle_value} degrees")


def on_fuel_level(frame: Frame):
    global fuel_level_value
    fuel_level_value = frame.data
    print(f"fuel_level FEED: {fuel_level_value}%")


def on_oxidizer_pressure(frame: Frame):
    global oxidizer_pressure_value
    oxidizer_pressure_value = frame.data
    print(f"oxidizer_pressure FEED: {oxidizer_pressure_value} bar")


if __name__ == "__main__":
    cm = CommunicationManager() # Class responsible for communication handling
    cm.change_transport_type(TransportType.TCP)
    # We must create a frame that will serve as a pattern indicating what kind of frames we want to receive
    # During frame equality comparison the following fields are excluded: priority, data_type, payload
    # You can find more information in communication_library/frame.py

    #Global variables for flight parameters
    # altitude_value (float)
    altitude_value = None
    # oxidizer_pressure_value (float)
    oxidizer_pressure_value = None
    # oxidizer_level_value (float)
    oxidizer_level_value = None
    # fuel_level_value (float)
    fuel_level_value = None
    # angle_value (float)
    angle_value = None

    #Lists for storing data
    altitude_list = []
    oxidizer_pressure_list = []
    oxidizer_level_list = []
    fuel_level_list = []
    angle_list = []

    altitude_frame = Frame(ids.BoardID.SOFTWARE, 
                           ids.PriorityID.LOW, 
                           ids.ActionID.FEED, 
                           ids.BoardID.ROCKET, 
                           ids.DeviceID.SENSOR, 
                           2, # altitude sensor
                           ids.DataTypeID.FLOAT,
                           ids.OperationID.SENSOR.value.READ)
    cm.register_callback(on_altitude, altitude_frame)
    cm.connect(TcpSettings("127.0.0.1", 3000))
    
    #Step 1 Opening the oxidizer intake valve

    oxidizer_valve_frame = Frame(ids.BoardID.SOFTWARE, 
                           ids.PriorityID.LOW, 
                           ids.ActionID.SERVICE, 
                           ids.BoardID.ROCKET, 
                           ids.DeviceID.SERVO, 
                           1, # oxidizer valve
                           ids.DataTypeID.INT16,
                           ids.OperationID.SERVO.value.POSITION,
                           (0,))

    cm.push(oxidizer_valve_frame) 
    cm.send() 
    

    #Frame for closing the oxidizer intake valve
    oxidizer_intake_close_frame = Frame(
        ids.BoardID.ROCKET,                    
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 
        ids.BoardID.SOFTWARE,                 
        ids.DeviceID.SERVO,                   
        1,                                    
        ids.DataTypeID.INT16,                 
        ids.OperationID.SERVO.value.POSITION, 
        (100,)                                # 100 = closed
    )
    

    #Frame for registering the oxidizer level feed pattern
    oxidizer_level_feed_pattern = Frame(
        ids.BoardID.SOFTWARE,               
        ids.PriorityID.LOW,
        ids.ActionID.FEED,                  
        ids.BoardID.ROCKET,                 
        ids.DeviceID.SENSOR,                
        1,                                  
        ids.DataTypeID.FLOAT,               
        ids.OperationID.SENSOR.value.READ   
    )
    cm.register_callback(on_oxidizer_level, oxidizer_level_feed_pattern)
    

    #Angle feed frame
    angle_feed_pattern = Frame(
        ids.BoardID.SOFTWARE,               
        ids.PriorityID.LOW,
        ids.ActionID.FEED,                  
        ids.BoardID.ROCKET,                 
        ids.DeviceID.SENSOR,                
        4,                                  
        ids.DataTypeID.FLOAT,               # float
        ids.OperationID.SENSOR.value.READ   # READ
    )
    cm.register_callback(on_angle, angle_feed_pattern)
    

    #Fuel valve frame
    fuel_valve_frame = Frame(
        ids.BoardID.ROCKET,                   
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 
        ids.BoardID.SOFTWARE,                 
        ids.DeviceID.SERVO,                   
        0,                                    # device_id=0 -> fuel_valve (not main)
        ids.DataTypeID.INT16,                 # POSITION is INT16
        ids.OperationID.SERVO.value.POSITION, # operacja: set position
        (0,)                                # 0 = open
    )


    #Frame for registering the fuel level feed pattern
    fuel_level_feed_pattern = Frame(
        ids.BoardID.SOFTWARE,               
        ids.PriorityID.LOW,
        ids.ActionID.FEED,                  
        ids.BoardID.ROCKET,                 
        ids.DeviceID.SENSOR,                
        0,                                  
        ids.DataTypeID.FLOAT,               # poziom to float
        ids.OperationID.SENSOR.value.READ   # READ
    )
    cm.register_callback(on_fuel_level, fuel_level_feed_pattern)
    
    
    #Frame for registering the oxidizer pressure feed pattern
    oxidizer_pressure_feed_pattern = Frame(
        ids.BoardID.SOFTWARE,               
        ids.PriorityID.LOW,
        ids.ActionID.FEED,                  
        ids.BoardID.ROCKET,                 
        ids.DeviceID.SENSOR,                
        3,                                  
        ids.DataTypeID.FLOAT,               
        ids.OperationID.SENSOR.value.READ   
    )
    cm.register_callback(on_oxidizer_pressure, oxidizer_pressure_feed_pattern)


    #Frame for closing the fuel valve
    fuel_valve_close_frame = Frame(
        ids.BoardID.ROCKET,                   
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 
        ids.BoardID.SOFTWARE,                 
        ids.DeviceID.SERVO,                   
        0,                                    
        ids.DataTypeID.INT16,                 # POSITION is INT16
        ids.OperationID.SERVO.value.POSITION, # operacja: set position
        (100,)                                # 100 = closed
    )

    # Oxidizer heater ON (relay OPEN, no payload)
    oxidizer_heater_on_frame = Frame(
        ids.BoardID.ROCKET,                   
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 
        ids.BoardID.SOFTWARE,                 
        ids.DeviceID.RELAY,                   
        0,                                    
        ids.DataTypeID.NO_DATA,               # relay doesn't need a value
        ids.OperationID.RELAY.value.OPEN,     # operation: OPEN (turn on)
        ()                                    # empty payload
    )
    
    # Oxidizer heater OFF (relay CLOSE, no payload)
    oxidizer_heater_off_frame = Frame(
        ids.BoardID.ROCKET,                   
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 
        ids.BoardID.SOFTWARE,                 
        ids.DeviceID.RELAY,                   
        0,                                    
        ids.DataTypeID.NO_DATA,               # relay doesn't need a value
        ids.OperationID.RELAY.value.CLOSE,    # operation: CLOSE (turn off)
        ()                                    # empty payload
    )

    #Frame for opening the main fuel valve
    main_fuel_valve_open_frame = Frame(
        ids.BoardID.ROCKET,                   
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 
        ids.BoardID.SOFTWARE,                 
        ids.DeviceID.SERVO,                   
        2,                                    
        ids.DataTypeID.INT16,                 # POSITION is INT16
        ids.OperationID.SERVO.value.POSITION, # operacja: set position
        (0,)                                  # 0 = open
    )

    #Frame for opening the main oxidizer valve
    main_oxidizer_valve_open_frame = Frame(
        ids.BoardID.ROCKET,                   
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 
        ids.BoardID.SOFTWARE,                 
        ids.DeviceID.SERVO,                   
        3,                                    
        ids.DataTypeID.INT16,                 # POSITION is INT16
        ids.OperationID.SERVO.value.POSITION, # operacja: set position
        (0,)                                  # 0 = open
    )

    #Frame for turning on the igniter
    igniter_on_frame = Frame(
        ids.BoardID.ROCKET,                     
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 
        ids.BoardID.SOFTWARE,                 
        ids.DeviceID.RELAY,                   
        1,                                    
        ids.DataTypeID.NO_DATA,               # relay doesn't need a value
        ids.OperationID.RELAY.value.OPEN,     # operation: OPEN (turn on)
        ()                                    # empty payload
    )

    #Frame for opening the parachute
    parachute_open_frame = Frame(
        ids.BoardID.ROCKET,                   
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 
        ids.BoardID.SOFTWARE,                 
        ids.DeviceID.RELAY,                   
        2,                                    
        ids.DataTypeID.NO_DATA,               # relay doesn't need a value
        ids.OperationID.RELAY.value.OPEN,     # operation: OPEN (turn on)
        ()                                    # empty payload
    )
    
    while True:
        try:
            frame = cm.receive() # We can handle frames using callbacks or by getting frame right from receive() call
            #Collecting data
            altitude_list.append(altitude_value)
            oxidizer_pressure_list.append(oxidizer_pressure_value)
            oxidizer_level_list.append(oxidizer_level_value)
            fuel_level_list.append(fuel_level_value)
            angle_list.append(angle_value)
            
            #Launch preparation sequence
            if oxidizer_level_value == 100 and fuel_level_value == 0:
                cm.push(oxidizer_intake_close_frame)
                cm.send()
                cm.push(fuel_valve_frame)
                cm.send()
            if fuel_level_value == 100 and oxidizer_pressure_value == 30:
                cm.push(fuel_valve_close_frame)
                cm.send()
            if oxidizer_pressure_value == 30:
                cm.push(oxidizer_heater_on_frame)
                cm.send()
            if oxidizer_pressure_value is not None and oxidizer_pressure_value > 60:
                cm.push(oxidizer_heater_off_frame)
                cm.send()
                break
            
        except TransportTimeoutError:
            pass
        except UnregisteredCallbackError as e:
            print(f"unregistered frame received: {e.frame}")
    
    #Ignition sequence
    #Closing valves for safety:
    cm.push(fuel_valve_close_frame)
    cm.send()
    cm.push(oxidizer_intake_close_frame)
    cm.send()
    cm.push(main_fuel_valve_open_frame)
    cm.send()
    
    cm.push(main_oxidizer_valve_open_frame)
    cm.send()
    
    cm.push(igniter_on_frame)
    cm.send()

    while True:
        try:
            frame = cm.receive() # We can handle frames using callbacks or by getting frame right from receive() call
            altitude_list.append(altitude_value)
            oxidizer_pressure_list.append(oxidizer_pressure_value)
            oxidizer_level_list.append(oxidizer_level_value)
            fuel_level_list.append(fuel_level_value)
            angle_list.append(angle_value)

            if len(altitude_list) > 1:
                if altitude_list[-1] < altitude_list[-2]:
                    cm.push(parachute_open_frame)
                    cm.send()
                    if altitude_list[-1] == 0:
                        break

        except TransportTimeoutError:
            pass
        except UnregisteredCallbackError as e:
            print(f"unregistered frame received: {e.frame}")

if __name__ == "__main__":
    if plt is not None:
        def _filtered_xy(series):
            xs = []
            ys = []
            for i, v in enumerate(series):
                if v is not None:
                    xs.append(i)
                    ys.append(v)
            return xs, ys
        def _velocity_xy(series):
            xs = []
            vs = []
            prev_i = None
            prev_v = None
            for i, v in enumerate(series):
                if v is None:
                    continue
                if prev_i is not None and prev_v is not None:
                    dt = i - prev_i
                    if dt > 0:
                        vs.append((v - prev_v) / dt)
                        xs.append(i)
                prev_i = i
                prev_v = v
            return xs, vs
        fig, axes = plt.subplots(3, 2, figsize=(12, 10))
        ax1 = axes[0][0]
        x, y = _filtered_xy(altitude_list)
        ax1.plot(x, y, label="Altitude")
        ax1.set_title("Altitude")
        ax1.set_xlabel("Sample")
        ax1.set_ylabel("m")
        ax1.grid(True)
        ax2 = axes[0][1]
        x, y = _filtered_xy(oxidizer_pressure_list)
        ax2.plot(x, y, color="tab:orange", label="Oxidizer Pressure")
        ax2.set_title("Oxidizer Pressure")
        ax2.set_xlabel("Sample")
        ax2.set_ylabel("bar")
        ax2.grid(True)
        ax3 = axes[1][0]
        x, y = _filtered_xy(oxidizer_level_list)
        ax3.plot(x, y, color="tab:green", label="Oxidizer Level")
        ax3.set_title("Oxidizer Level")
        ax3.set_xlabel("Sample")
        ax3.set_ylabel("%")
        ax3.grid(True)
        ax4 = axes[1][1]
        x, y = _filtered_xy(fuel_level_list)
        ax4.plot(x, y, color="tab:red", label="Fuel Level")
        ax4.set_title("Fuel Level")
        ax4.set_xlabel("Sample")
        ax4.set_ylabel("%")
        ax4.grid(True)
        ax5 = axes[2][0]
        x, y = _filtered_xy(angle_list)
        ax5.plot(x, y, color="tab:purple", label="Angle")
        ax5.set_title("Angle")
        ax5.set_xlabel("Sample")
        ax5.set_ylabel("deg")
        ax5.grid(True)
        ax6 = axes[2][1]
        vx, vy = _velocity_xy(altitude_list)
        ax6.plot(vx, vy, color="tab:brown", label="Velocity")
        ax6.set_title("Vertical Velocity (from Altitude)")
        ax6.set_xlabel("Sample")
        ax6.set_ylabel("m/s")
        ax6.grid(True)
        fig.tight_layout()
        plt.show()
    else:
        print("matplotlib is not available; skipping plots")