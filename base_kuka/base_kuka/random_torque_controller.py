from std_msgs.msg import Float64MultiArray
@nrp.MapRobotPublisher('joints_command', Topic('/iiwa/iiwa_effort_controller/command', Float64MultiArray))
@nrp.Neuron2Robot()
def simple_move_robot(t, joints_command):
    from numpy.random import random
    from std_msgs.msg import Float64MultiArray, MultiArrayDimension
    ##################################
    ### Description of std_msgs/Float64MultiArray (printed with rostopic)
    # this is the type of message accepted by effort_controllers/JointGroupEffortController
    ###
    #std_msgs/MultiArrayLayout layout
    #   std_msgs/MultiArrayDimension[] dim
    #       string label
    #       uint32 size
    #       uint32 stride
    #   uint32 data_offset
    #float64[] data
    ###
    ##################################
    
    # Message generation
    cmd = Float64MultiArray()
    cmd.layout.dim.append(MultiArrayDimension(label='torques',size=7,stride=1))
    # assign torques values (that will be applied to the joints)
    cmd.data = list(random(size=7)-0.5)
    
    joints_command.send_message(cmd)
