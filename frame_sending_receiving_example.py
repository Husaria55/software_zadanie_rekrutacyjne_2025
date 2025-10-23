from communication_library.communication_manager import CommunicationManager, TransportType
from communication_library.tcp_transport import TcpSettings
from communication_library.frame import Frame
from communication_library import ids
from communication_library.exceptions import TransportTimeoutError, TransportError, UnregisteredCallbackError

def on_altitude(frame: Frame):
    global altitude_value
    altitude_value = frame.data
    print(f"altitude FEED: {altitude_value} m")

if __name__ == "__main__":
    cm = CommunicationManager() # Class responsible for communication handling
    cm.change_transport_type(TransportType.TCP)
    # We must create a frame that will serve as a pattern indicating what kind of frames we want to receive
    # During frame equality comparison the following fields are excluded: priority, data_type, payload
    # You can find more information in communication_library/frame.py
    # Zmienna na wysokość (float)
    altitude_value = None


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
    
    #Krok 1 Otwieranie zaworu tankowania utleniacza

    oxidizer_valve_frame = Frame(ids.BoardID.SOFTWARE, 
                           ids.PriorityID.LOW, 
                           ids.ActionID.SERVICE, 
                           ids.BoardID.ROCKET, 
                           ids.DeviceID.SERVO, 
                           1, # oxidizer valve
                           ids.DataTypeID.INT16,
                           ids.OperationID.SERVO.value.POSITION,
                           (0,))

    cm.push(oxidizer_valve_frame) # We need to push the frame onto the send queue
    cm.send() # Send queue first in the send queue
    

    # Zamknięcie zaworu tankowania utleniacza (SERVO id=1) -> pozycja 100
    oxidizer_intake_close_frame = Frame(
        ids.BoardID.ROCKET,                   # destination: ROCKET (wysyłamy do rakiety)
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 # SERVICE: wykonaj operację
        ids.BoardID.SOFTWARE,                 # source: SOFTWARE (my)
        ids.DeviceID.SERVO,                   # urządzenie: SERVO
        1,                                    # device_id=1 -> oxidizer_intake
        ids.DataTypeID.INT16,                 # POSITION oczekuje INT16
        ids.OperationID.SERVO.value.POSITION, # operacja: ustaw pozycję
        (100,)                                # 100 = zamknięty
    )
    


    # Zmienna na poziom utleniacza (float)
    oxidizer_level_value = None

    def on_oxidizer_level(frame: Frame):
        # frame.data zwraca pierwszy element payload przy data_type=FLOAT
        # alternatywnie: value = frame.payload[0]
        global oxidizer_level_value
        oxidizer_level_value = frame.data
        print(f"oxidizer_level FEED: {oxidizer_level_value}%")

    # ... w __main__ po utworzeniu cm i PRZED pętlą receive():
    oxidizer_level_feed_pattern = Frame(
        ids.BoardID.SOFTWARE,               # chcemy otrzymywać do SOFTWARE
        ids.PriorityID.LOW,
        ids.ActionID.FEED,                  # FEED z sensora
        ids.BoardID.ROCKET,                 # źródło: ROCKET
        ids.DeviceID.SENSOR,                # typ: SENSOR
        1,                                  # device_id=1 -> oxidizer_level
        ids.DataTypeID.FLOAT,               # poziom to float
        ids.OperationID.SENSOR.value.READ   # READ
    )

    cm.register_callback(on_oxidizer_level, oxidizer_level_feed_pattern)
    
    #Fuel valve frame
    fuel_valve_frame = Frame(
        ids.BoardID.ROCKET,                   # destination: ROCKET (wysyłamy do rakiety)
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 # SERVICE: wykonaj operację
        ids.BoardID.SOFTWARE,                 # source: SOFTWARE (my)
        ids.DeviceID.SERVO,                   # urządzenie: SERVO
        0,                                    # device_id=0 -> fuel_valve (not main)
        ids.DataTypeID.INT16,                 # POSITION oczekuje INT16
        ids.OperationID.SERVO.value.POSITION, # operacja: ustaw pozycję
        (0,)                                # 0 = otwarte
    )

    # Zmienna na poziom paliwa (float)
    fuel_level_value = None

    def on_fuel_level(frame: Frame):
        # frame.data zwraca pierwszy element payload przy data_type=FLOAT
        # alternatywnie: value = frame.payload[0]
        global fuel_level_value
        fuel_level_value = frame.data
        print(f"fuel_level FEED: {fuel_level_value}%")

    # ... w __main__ po utworzeniu cm i PRZED pętlą receive():
    fuel_level_feed_pattern = Frame(
        ids.BoardID.SOFTWARE,               # chcemy otrzymywać do SOFTWARE
        ids.PriorityID.LOW,
        ids.ActionID.FEED,                  # FEED z sensora
        ids.BoardID.ROCKET,                 # źródło: ROCKET
        ids.DeviceID.SENSOR,                # typ: SENSOR
        0,                                  # device_id=1 -> oxidizer_level
        ids.DataTypeID.FLOAT,               # poziom to float
        ids.OperationID.SENSOR.value.READ   # READ
    )

    cm.register_callback(on_fuel_level, fuel_level_feed_pattern)
    
    # Zmienna na ciśnienie oxidizera (float)
    oxidizer_pressure_value = None

    def on_oxidizer_pressure(frame: Frame):
        # frame.data zwraca pierwszy element payload przy data_type=FLOAT
        # alternatywnie: value = frame.payload[0]
        global oxidizer_pressure_value
        oxidizer_pressure_value = frame.data
        print(f"oxidizer_pressure FEED: {oxidizer_pressure_value} bar")

    # ... w __main__ po utworzeniu cm i PRZED pętlą receive():
    oxidizer_pressure_feed_pattern = Frame(
        ids.BoardID.SOFTWARE,               # chcemy otrzymywać do SOFTWARE
        ids.PriorityID.LOW,
        ids.ActionID.FEED,                  # FEED z sensora
        ids.BoardID.ROCKET,                 # źródło: ROCKET
        ids.DeviceID.SENSOR,                # typ: SENSOR
        3,                                  # device_id=3 -> oxidizer_pressure
        ids.DataTypeID.FLOAT,               # ciśnienie to float
        ids.OperationID.SENSOR.value.READ   # READ
    )

    cm.register_callback(on_oxidizer_pressure, oxidizer_pressure_feed_pattern)


    # Fuel valve close frame
    fuel_valve_close_frame = Frame(
        ids.BoardID.ROCKET,                   # destination: ROCKET (wysyłamy do rakiety)
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 # SERVICE: wykonaj operację
        ids.BoardID.SOFTWARE,                 # source: SOFTWARE (my)
        ids.DeviceID.SERVO,                   # urządzenie: SERVO
        0,                                    # device_id=0 -> fuel_valve (not main)
        ids.DataTypeID.INT16,                 # POSITION oczekuje INT16
        ids.OperationID.SERVO.value.POSITION, # operacja: ustaw pozycję
        (100,)                                # 100 = zamknięty
    )

    # Oxidizer heater ON (relay OPEN, no payload)
    oxidizer_heater_on_frame = Frame(
        ids.BoardID.ROCKET,                   # destination: ROCKET
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 # command
        ids.BoardID.SOFTWARE,                 # source: SOFTWARE
        ids.DeviceID.RELAY,                   # device family: RELAY
        0,                                    # device_id=0 -> oxidizer_heater (see simulator_config.yaml)
        ids.DataTypeID.NO_DATA,               # relay doesn't need a value
        ids.OperationID.RELAY.value.OPEN,     # operation: OPEN (turn on)
        ()                                    # empty payload
    )
    
    # Oxidizer heater OFF (relay CLOSE, no payload)
    oxidizer_heater_off_frame = Frame(
        ids.BoardID.ROCKET,                   # destination: ROCKET
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 # command
        ids.BoardID.SOFTWARE,                 # source: SOFTWARE
        ids.DeviceID.RELAY,                   # device family: RELAY
        0,                                    # device_id=0 -> oxidizer_heater (see simulator_config.yaml)
        ids.DataTypeID.NO_DATA,               # relay doesn't need a value
        ids.OperationID.RELAY.value.CLOSE,    # operation: CLOSE (turn off)
        ()                                    # empty payload
    )
    
    while True:
        try:
            frame = cm.receive() # We can handle frames using callbacks or by getting frame right from receive() call
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
    
    #Sekwencja zapłonu
    #zamknięcie zaworów dla upewnienia:
    cm.push(fuel_valve_close_frame)
    cm.send()
    cm.push(oxidizer_intake_close_frame)
    cm.send()
    #Otwieranie zaworu głównego paliwa

    main_fuel_valve_open_frame = Frame(
        ids.BoardID.ROCKET,                   # destination: ROCKET
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 # command
        ids.BoardID.SOFTWARE,                 # source: SOFTWARE
        ids.DeviceID.SERVO,                   # device family: SERVO
        2,                                    # device_id=2 -> main_fuel_valve (see simulator_config.yaml)
        ids.DataTypeID.INT16,                 # POSITION oczekuje INT16
        ids.OperationID.SERVO.value.POSITION, # operacja: ustaw pozycję
        (0,)                                  # 0 = otwarte
    )

    cm.push(main_fuel_valve_open_frame)
    cm.send()
    
    #Otwieranie zaworu głównego utleniacza
    main_oxidizer_valve_open_frame = Frame(
        ids.BoardID.ROCKET,                   # destination: ROCKET
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 # command
        ids.BoardID.SOFTWARE,                 # source: SOFTWARE
        ids.DeviceID.SERVO,                   # device family: SERVO
        3,                                    # device_id=3 -> main_oxidizer_valve (see simulator_config.yaml)
        ids.DataTypeID.INT16,                 # POSITION oczekuje INT16
        ids.OperationID.SERVO.value.POSITION, # operacja: ustaw pozycję
        (0,)                                  # 0 = otwarte
    )
    
    cm.push(main_oxidizer_valve_open_frame)
    cm.send()
    
    #Włączenie ignitera
    igniter_on_frame = Frame(
        ids.BoardID.ROCKET,                   # destination: ROCKET
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 # command
        ids.BoardID.SOFTWARE,                 # source: SOFTWARE
        ids.DeviceID.RELAY,                   # device family: RELAY
        1,                                    # device_id=1 -> igniter (see simulator_config.yaml)
        ids.DataTypeID.NO_DATA,               # relay doesn't need a value
        ids.OperationID.RELAY.value.OPEN,     # operation: OPEN (turn on)
        ()                                    # empty payload
    )

    cm.push(igniter_on_frame)
    cm.send()

    #Otwieranie spadochronu
    parachute_open_frame = Frame(
        ids.BoardID.ROCKET,                   # destination: ROCKET
        ids.PriorityID.LOW,
        ids.ActionID.SERVICE,                 # command
        ids.BoardID.SOFTWARE,                 # source: SOFTWARE
        ids.DeviceID.RELAY,                   # device family: RELAY
        2,                                    # device_id=2 -> parachute (see simulator_config.yaml)
        ids.DataTypeID.NO_DATA,               # relay doesn't need a value
        ids.OperationID.RELAY.value.OPEN,     # operation: OPEN (turn on)
        ()                                    # empty payload
    )
    
    flight_path = []
    while True:
        try:
            frame = cm.receive() # We can handle frames using callbacks or by getting frame right from receive() call
            flight_path.append(altitude_value)
            if len(flight_path) > 1:
                if flight_path[-1] < flight_path[-2]:
                    cm.push(parachute_open_frame)
                    cm.send()
                    break

            
        except TransportTimeoutError:
            pass
        except UnregisteredCallbackError as e:
            print(f"unregistered frame received: {e.frame}")
    
        
        