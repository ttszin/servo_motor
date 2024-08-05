import rospy
from mavros_msgs.srv import CommandLong, SetMode, CommandBool
from mavros_msgs.msg import OverrideRCIn, State
from sensor_msgs.msg import BatteryState

# Inicializa o nó ROS
rospy.init_node('mavros_control_node')

# Conectando-se ao MAVROS e aguardando o heartbeat
rospy.wait_for_service('/mavros/cmd/arming')
rospy.wait_for_service('/mavros/set_mode')
rospy.wait_for_service('/mavros/cmd/command')

def get_flight_mode():
    state = rospy.wait_for_message('/mavros/state', State)
    return state.mode

def set_flight_mode(mode):
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = set_mode_service(0, mode)
        if response.mode_sent:
            print(f"Modo de voo alterado para {mode} com sucesso!")
        else:
            print(f"Falha ao alterar o modo de voo para {mode}.")
    except rospy.ServiceException as e:
        print(f"Falha ao chamar o serviço set_mode: {e}")

def set_servo(channel, pwm_value):
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    rc_override = OverrideRCIn()
    rc_override.channels = [0] * 8  # Inicializa todos os canais com 0 (sem override)
    rc_override.channels[channel - 1] = pwm_value  # Ajusta o PWM no canal especificado
    pub.publish(rc_override)
    rospy.sleep(1)  # Aguarda 1 segundo

# Exemplo de uso:

# Obtenha e imprima o modo de voo atual
current_mode = get_flight_mode()
print("Modo de voo atual:", current_mode)

# Alterar o modo de voo para 'GUIDED'
set_flight_mode('GUIDED')

# Obtenha e imprima novamente o modo de voo atual
current_mode = get_flight_mode()
print("Modo de voo atual após mudança:", current_mode)

# Ajustar o valor PWM do canal 7 para 2000
set_servo(7, 2000)
rospy.sleep(2)  # Espera 2 segundos

# Ajustar o valor PWM do canal 7 para 700
set_servo(7, 700)
# rospy.sleep(2)  # Espera 2 segundos

# Fechar o nó ROS ao final
rospy.signal_shutdown('Finalizando a conexão')
