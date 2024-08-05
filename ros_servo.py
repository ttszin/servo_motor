import rospy
from mavros_msgs.srv import SetMode, CommandLong
from mavros_msgs.msg import State

current_state = State()

def state_callback(state):
    global current_state
    current_state = state

def set_custom_mode(custom_mode: str = "GUIDED") -> bool:
    rospy.wait_for_service("/mavros/set_mode", timeout=60)
    try:
        set_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        response = set_mode_service(0, custom_mode)
        return response.mode_sent
    except rospy.ServiceException as e:
        rospy.logerr(f"Falha na chamada do serviço: {e}")
        return False

def activate_servo(servo_number, pwm_value):
    if current_state.mode != "GUIDED":
        if not set_custom_mode("GUIDED"):
            rospy.logerr("Falha ao mudar para o modo GUIDED.")
            return False
    
    rospy.wait_for_service('/mavros/cmd/command')
    try:
        command_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        response = command_service(
            False,        # broadcast
            183,          # MAV_CMD_DO_SET_SERVO
            0,            # confirmation
            servo_number, # param1: Número do servo (1, 2, 3, etc.)
            pwm_value,    # param2: Valor PWM para ativar o servo (e.g., 1500)
            0, 0, 0, 0, 0 # param3 - param7: Não usados
        )
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Falha na chamada do serviço: %s" % e)
        return False

if __name__ == "__main__":
    rospy.init_node('servo_control_node')
    rospy.Subscriber("/mavros/state", State, state_callback)
    
    # Exemplo: Ativa o servo no canal 7 com um valor PWM de 1500
    if activate_servo(7, 1500):
        rospy.loginfo("Servo ativado com sucesso!")
    else:
        rospy.logerr("Falha ao ativar o servo.")
