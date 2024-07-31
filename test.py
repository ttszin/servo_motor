from pymavlink import mavutil
import time

# Defina o caminho do dispositivo serial para a Pixhawk
device_path = '/dev/ttyACM0'  # Ajuste conforme necessário (ou '/dev/ttyACM0')

# Conecta ao veículo na porta serial especificada
master = mavutil.mavlink_connection(device_path, baud=57600)

# Espera a conexão com o veículo
master.wait_heartbeat()

master.set_mode('GUIDED')

print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

# Função para obter parâmetros
def get_param(param_name):
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode('ascii'),
        -1  # -1 para ler todos os parâmetros
    )
    while True:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True)
        if msg.param_id == param_name.encode('ascii'):
            return msg.param_value

# Função para definir parâmetros
def set_param(param_name, value):
    master.mav.param_set_send(
        param_name.encode('ascii'),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(1)  # Aguarda a aplicação do parâmetro

# Verificar e ajustar o parâmetro SERVO8_FUNCTION
param_name = 'SERVO8_FUNCTION'
current_value = get_param(param_name)
print(f"Current value of {param_name}: {current_value}")

if current_value != 0:  # Supondo que 0 é a função desejada
    print(f"Setting {param_name} to 0")
    set_param(param_name, 0)
else:
    print(f"{param_name} is already set to 0")

# Fecha a conexão
master.close()
