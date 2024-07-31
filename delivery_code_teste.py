from pymavlink import mavutil
import time

#NESTE CÓDIGO ESTÁ SENDO USADO O PINO 7 COMO O PINO EM QUE O SERVO MOTOR ESTÁ LIGADO
# OBS: ELE PRECISA DE ALIMENTAÇÃO EXTERNA DA PIXHAWK PARA FUNCIONAR

def control_servo_and_get_flight_mode(device_path='/dev/ttyACM0', baud_rate=115200, servo_channel=7, pwm_values=[2000, 700]):
    """
    Conecta à Pixhawk, obtém o modo de voo atual e controla um servo motor no canal especificado.

    device_path: O caminho do dispositivo serial para a Pixhawk.
    baud_rate: A taxa de baud para a comunicação serial.
    servo_channel: O canal ao qual o servo está conectado (normalmente de 1 a 8).
    pwm_values: Lista de valores PWM para ajustar o servo motor.
    """
    # Conecta ao veículo na porta serial especificada
    master = mavutil.mavlink_connection(device_path, baud=baud_rate)

    # Espera a conexão com o veículo
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

    # Função para obter o modo de voo
    def get_flight_mode():
        # Solicita uma mensagem HEARTBEAT
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if not msg:
            print("No heartbeat message received")
            return None

        # Mapeamento dos modos de voo
        mode_mapping = {
            0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED', 5: 'LOITER', 6: 'RTL', 7: 'CIRCLE',
            8: 'LAND', 9: 'OF_LOITER', 10: 'DRIFT', 11: 'SPORT', 12: 'FLIP', 13: 'AUTOTUNE', 14: 'POSHOLD', 15: 'BRAKE',
            16: 'THROW', 17: 'AVOID_ADSB', 18: 'GUIDED_NOGPS', 19: 'SMART_RTL', 20: 'FLOWHOLD', 21: 'FOLLOW', 22: 'ZIGZAG',
            23: 'SYSTEMID', 24: 'AUTOROTATE', 25: 'AUTO_RTL'
        }

        # Obtém o modo de voo atual
        mode_id = msg.custom_mode
        mode_name = mode_mapping.get(mode_id, "Unknown")

        return mode_name

    current_mode = get_flight_mode()
    print(current_mode)
    master.set_mode('GUIDED')

    current_mode = get_flight_mode()
    print(current_mode)

    # Função para ajustar o valor PWM do servo
    def set_servo(channel, pwm_value):
        print(f"Setting channel {channel} to PWM value {pwm_value}")
        master.mav.command_long_send(
            master.target_system,  # system ID
            master.target_component,  # component ID
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
            0,  # confirmation
            channel,  # param1 (servo number)
            pwm_value,  # param2 (PWM value)
            0, 0, 0, 0, 0  # param3 ~ param7 (not used)
        )
        # Aguarda a resposta
        time.sleep(1)   

    # Move o servo nos valores PWM especificados
    for pwm in pwm_values:
        set_servo(servo_channel, pwm)
        time.sleep(2)  # Espera 2 segundos

    # Fecha a conexão
    master.close()

#Chamando a função
control_servo_and_get_flight_mode()
