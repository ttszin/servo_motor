import rospy
from mavros_msgs.srv import CommandLong

# Inicializa o nó ROS
rospy.init_node('reboot_autopilot_node')

# Aguarda pelo serviço '/mavros/cmd/command' estar disponível
rospy.wait_for_service('/mavros/cmd/command')

try:
    # Conecta-se ao serviço CommandLong
    command_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
    
    # Envia o comando de reinicialização do piloto automático
    response = command_service(
        broadcast=False,  # Não transmitir para todos os sistemas
        command=246,      # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246)
        confirmation=0,   # Sem necessidade de confirmação
        param1=1,         # Reboot the autopilot
        param2=0,         # Os demais parâmetros não são utilizados
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0
    )
    
    if response.success:
        print("Reinicialização do piloto automático bem-sucedida!")
    else:
        print("Falha na reinicialização do piloto automático.")
except rospy.ServiceException as e:
    print("Erro ao chamar o serviço de reinicialização: %s" % e)
