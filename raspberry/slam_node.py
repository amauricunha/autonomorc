import threading
import time
from can_interface import receive_can_nav_data, send_can_message, receive_obstacle_status
import cv2
import glob

def slam_process(frame, nav_data, obstacle):
    # Placeholder para integração com ORB-SLAM2/RTAB-Map
    # Estratégia de desvio: se obstáculo, vira o servo para a direita/esquerda
    if obstacle:
        print("Obstáculo detectado pelo Arduino! Tentando desviar...")
        # Exemplo: virar servo para 150 (direita) por alguns frames
        return {"esc_pwm": 100, "servo_angle": 150}
    else:
        # Caminho normal (exemplo: servo centralizado, PWM padrão)
        return {"esc_pwm": 120, "servo_angle": 90}

def wait_for_autonomous_mode():
    print("Aguardando ativação do modo autônomo via CAN (CH4)...")
    while True:
        nav = receive_can_nav_data(timeout=0.5)
        # O Arduino só envia dados CAN no modo manual, então espera parar de receber para assumir modo autônomo
        if nav is None:
            print("Modo autônomo ativado (CAN de navegação parou)!")
            break
        time.sleep(0.5)

def run_slam(frame_dir="frames"):
    frame_files = sorted(glob.glob(f"{frame_dir}/frame_*.jpg"))
    frame_idx = 0
    nav_data_log = []
    print("Aguardando modo: gravando navegação (manual) ou controlando (autônomo)...")
    autonomous_mode = False

    while frame_idx < len(frame_files):
        frame = cv2.imread(frame_files[frame_idx])
        nav_data = receive_can_nav_data(timeout=0.1)
        obstacle = receive_obstacle_status(timeout=0.01)

        # Detecta transição de modo: se parar de receber nav_data, assume modo autônomo
        if not autonomous_mode and nav_data is None:
            print("Modo autônomo ativado! SLAM passa a controlar o carro.")
            autonomous_mode = True

        if not autonomous_mode:
            # Modo manual: grava dados de navegação para mapeamento
            if nav_data:
                nav_data_log.append(nav_data)
                print(f"Gravando navegação: {nav_data}")
        else:
            # Modo autônomo: processa SLAM e envia comandos CAN
            ctrl = slam_process(frame, nav_data, obstacle)
            send_can_message(ctrl["esc_pwm"], ctrl["servo_angle"])

        frame_idx += 1
        time.sleep(0.03)  # ~30 FPS

    # Salva log de navegação ao final do mapeamento
    if nav_data_log:
        import csv
        with open("nav_data_log.csv", "w", newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=["timestamp", "wheel_speed", "pulse_count", "slip"])
            writer.writeheader()
            writer.writerows(nav_data_log)
        print("Log de navegação salvo para SLAM.")

    print("Processamento SLAM finalizado.")

if __name__ == "__main__":
    run_slam()
