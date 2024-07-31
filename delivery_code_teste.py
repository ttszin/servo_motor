from pymavlink import mavutil
import time

# Defina o caminho do dispositivo serial para a Pixhawk
device_path = '/dev/ttyACM0'  # Ajuste conforme necessário (ou '/dev/ttyACM0')


# Conecta ao veículo na porta serial especificada
master = mavutil.mavlink_connection(device_path, baud=115200)



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
        0: 'STABILIZE',
        1: 'ACRO',
        2: 'ALT_HOLD',
        3: 'AUTO',
        4: 'GUIDED',
        5: 'LOITER',
        6: 'RTL',
        7: 'CIRCLE',
        8: 'LAND',
        9: 'OF_LOITER',
        10: 'DRIFT',
        11: 'SPORT',
        12: 'FLIP',
        13: 'AUTOTUNE',
        14: 'POSHOLD',
        15: 'BRAKE',
        16: 'THROW',
        17: 'AVOID_ADSB',
        18: 'GUIDED_NOGPS',
        19: 'SMART_RTL',
        20: 'FLOWHOLD',
        21: 'FOLLOW',
        22: 'ZIGZAG',
        23: 'SYSTEMID',
        24: 'AUTOROTATE',
        25: 'AUTO_RTL'
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

def set_servo(channel, pwm_value):
    """
    Ajusta o valor PWM do canal especificado.
    
    channel: O canal ao qual o servo está conectado (normalmente de 1 a 8)
    pwm_value: O valor PWM a ser definido (normalmente de 1000 a 2000 microsegundos)
    """
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

# Move o servo no canal 8 para 1500 microsegundos de PWM
set_servo(7, 2000)
time.sleep(2)  # Espera 2 segundos

# Move o servo para o valor PWM mínimo (1000 microsegundos)
set_servo(7,700)
# time.sleep(2)  # Espera 2 segundos

# # Move o servo de volta ao valor PWM médio (1500 microsegundos)
# set_servo(7, 2000)
# time.sleep(2)  # Espera 2 segundos

# Fecha a conexão
master.close()
