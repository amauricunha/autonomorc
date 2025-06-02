import can
import csv
import time

CAN_CHANNEL = 'can0'
CAN_BUSTYPE = 'socketcan'
NAV_DATA_ID = 0x200  # ID dos dados de navegação enviados pelo Arduino
OBS_DATA_ID = 0x300  # ID da mensagem de obstáculo

def send_can_message(esc_pwm, servo_angle):
    bus = can.interface.Bus(channel=CAN_CHANNEL, bustype=CAN_BUSTYPE)
    msg = can.Message(arbitration_id=0x100, data=[esc_pwm, servo_angle], is_extended_id=False)
    try:
        bus.send(msg)
        print(f"Sent CAN: ESC={esc_pwm}, SERVO={servo_angle}")
    except can.CanError:
        print("CAN send failed")

def receive_can_nav_data(timeout=1.0):
    bus = can.interface.Bus(channel=CAN_CHANNEL, bustype=CAN_BUSTYPE)
    msg = bus.recv(timeout=timeout)
    if msg and msg.arbitration_id == NAV_DATA_ID and len(msg.data) >= 5:
        wheel_speed = ((msg.data[0] << 8) | msg.data[1]) / 100.0
        pulse_count = (msg.data[2] << 8) | msg.data[3]
        slip = msg.data[4] / 100.0
        return {
            "timestamp": time.time(),
            "wheel_speed": wheel_speed,
            "pulse_count": pulse_count,
            "slip": slip
        }
    return None

def receive_obstacle_status(timeout=0.1):
    bus = can.interface.Bus(channel=CAN_CHANNEL, bustype=CAN_BUSTYPE)
    msg = bus.recv(timeout=timeout)
    if msg and msg.arbitration_id == OBS_DATA_ID and len(msg.data) >= 1:
        return bool(msg.data[0])
    return None

def log_nav_data(csv_filename="nav_data_log.csv", stop_event=None):
    with open(csv_filename, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'wheel_speed', 'pulse_count', 'slip'])
        print("Iniciando recepção e log dos dados CAN de navegação...")
        while not (stop_event and stop_event.is_set()):
            nav = receive_can_nav_data()
            if nav:
                writer.writerow([nav['timestamp'], nav['wheel_speed'], nav['pulse_count'], nav['slip']])
                print(f"CAN NAV: wheel_speed={nav['wheel_speed']:.2f} m/s, pulse_count={nav['pulse_count']}, slip={nav['slip']:.2f}")

if __name__ == "__main__":
    # Apenas exemplo de logging
    import threading
    stop_event = threading.Event()
    try:
        log_nav_data("nav_data_log.csv", stop_event)
    except KeyboardInterrupt:
        stop_event.set()
