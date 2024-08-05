import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String

def state_cb(state):
    rospy.loginfo(f"Current state: {state.mode}")

def connect_mavros():
    rospy.init_node('mavros_connection_node', anonymous=True)

    # Assinatura do tópico de estado
    state_sub = rospy.Subscriber('/mavros/state', State, state_cb)

    # Serviço para armar/desarmar o veículo
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

    # Serviço para definir o modo de voo
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    rate = rospy.Rate(20) # 20 Hz

    while not rospy.is_shutdown():
        # Aqui você pode adicionar a lógica para enviar comandos
        # Exemplo de armamento:
        arming_client.call(True)
        # Exemplo de mudança de modo para GUIDED
        set_mode_client.call(0, "GUIDED")

        rate.sleep()

if __name__ == '__main__':
    try:
        connect_mavros()
    except rospy.ROSInterruptException:
        pass
