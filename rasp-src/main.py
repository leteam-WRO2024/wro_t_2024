from Color_Detection import ColorDetectionThread, ColorsCoordinations
from time import sleep
import serial
from global_vals import FRAME_CENTER, FRAME_WIDTH

def calculate_pillar_error(coords, sign: int = 1):
    error = 0
    object_center_x, _, _, _, w, h = coords

    if sign == 1:
        distance_from_center = object_center_x - FRAME_CENTER
    else:
        distance_from_center = object_center_x
    
    distance_from_center /= FRAME_CENTER
    error = w * h * distance_from_center

    return error

def hanlde_blue(coords, timestamp):
    global blue_counter, blue_time, stop_flag
    blue_time = timestamp
    blue_counter += 1
    
    if blue_counter >= 11:
        stop_flag = True
        serial_message += f"s:{str(stop_flag).lower()}\n"

def handle_orange(coords, timestamp):
    global orange_counter, orange_time, stop_flag
    orange_time = timestamp
    orange_counter += 1
    
    if orange_counter >= 11:
        stop_flag = True
        serial_message += f"s:{str(stop_flag).lower()}\n"


def handle_green(coords, timestamp):
    global green_time, serial_message
    green_time = timestamp
    error = calculate_pillar_error(coords, -1)
    serial_message += f"g:{int(error)}\n"

def handle_red(coords, timestamp):
    global red_time, serial_message
    red_time = timestamp
    error = calculate_pillar_error(coords, 1)
    serial_message += f"r:{int(error)}\n"

def handle_colors(color, values):
    coords, timestamp = values

    if coords == (-1, -1, -1, -1, -1, -1):
        return

    handler = color_handles.get(color)
    handler(coords, timestamp)

def main():
    while True:
        frame = frame_provider.read()
        if frame is None:
            continue

        for thread in threads:
            if hasattr(thread.local_storage, "result"):
                results.update(thread.color, thread.local_storage.result)
            
        for color, values in results.items():
            handle_colors(color, values)
        
        if serial_message:
            ser.write(serial_message.encode("utf-8"))
            serial_message = ""

if __name__ == "__main__":
    colors             = ["red", "green", "orange", "blue"]
    thread_stop_flags  = {color: False for color in colors}
    threads            = []
    results            = {}
    color_handles = {
        "red": handle_red,
        "blue": hanlde_blue,
        "green": handle_green,
        "orange": handle_orange
    }

    # Serial for communicating with arduino
    ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1, stopbits=serial.STOPBITS_ONE)
    ser.flush()
    ser.reset_input_buffer()

    if not ser.is_open():
        ser.open()
        sleep(1)

    # Starting frame capturing thread to get images from the camera
    frame_provider = ColorsCoordinations().start()
    sleep(2)

    ## FLAGS, TIMES, MAIN VARS
    blue_time   = 0
    orange_time = 0
    red_time    = 0
    green_time  = 0

    orange_counter = 0
    blue_counter   = 0

    stop_flag = False

    # Last message to sen to arduino
    serial_message = ""

    for color in colors:
        cthread = ColorDetectionThread(color, frame_provider, results, thread_stop_flags)
        cthread.daemon = True
        cthread.start()
        threads.append(cthread)

    main()