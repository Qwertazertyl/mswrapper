#!/usr/bin/python

import rospy
import ethernetcommunicator
import serialcommunicator
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String
from mswrapper.srv import *
"""
Wrapper node for MMC/...

Publishes the value of input1 to the mswrapper/input_1 topic
Subscribes to the mswrapper/set_speed topic and jogs the motor the published value
Subscribes to the mswrapper/send_command topic to send the published ascii command to the controller, without a response.

Serves the service mswrapper/send_command_service to send any ascii command to the controller and respond with the result.
"""

#Create an instance of the Ethernetcommunicator class as cm
cm = ethernetcommunicator.Ethernetcommunicator()

#Create an instance of the Serialcommunicator class as cm
#cm = serialcommunicator.Serialcommunicator()

def send_command_callback(data):
    return cm.send_command(data.data)

def set_speed_callback(data):
    cm.motor_jog(data.data)

def handle_send_command(req):
    # Converts the argument to a string, extracts the command part from the string,
    # sends it to the controller, converts the result to a std_msgs/String (Different
    # from python string), formats it as a valid response and sends it.
    controller_response = cm.send_command(str(req.command)[6:])
    if not controller_response:
        controller_response = ""
    return SendCommandResponse(String(controller_response))
#Create publishers
# (topic name, message type (must be imported),
# queue_size (Incomming message queue, if the queue is full,
# the oldest message will be thrown out)) 
input_1_pub = rospy.Publisher('mswrapper/input_1', Bool, queue_size=10)

#Create subscribers for each topic, with callback functions
# (topic name, message type (must be imported), callback function)
rospy.Subscriber('mswrapper/set_speed', Float64, set_speed_callback)
rospy.Subscriber('mswrapper/send_command', String, send_command_callback)

#Initialize and name node 
# In ROS, nodes are uniquely named. If two nodes with the same          
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.
rospy.init_node('asciicom', anonymous=False)

#Create a service to send a command and recieve the response
# Services make more sense for sending commands because they 
# can handle sending direct responses.
send_command_service = rospy.Service('mswrapper/send_command', SendCommand, handle_send_command)


def loop():
    

    # In hz; Makes best effort at maintaining rate for a loop.
    # Set really low to prevent overlapping with other commands.
    # The only thing that happens in the loop is publishing input1's value.
    rate = rospy.Rate(1)

    #rospy.logdebug("This is an example of asciicom logging debug information!")
    #rospy.loginfo("This is an example of asciicom logging information!")
    #rospy.logwarn("This is an example of asciicom logging a warning!")
    #rospy.logerr("This is an example of asciicom logging an error!")
    #rospy.logfatal("This is an example of asciicom logging a fatal error!")

    while not rospy.is_shutdown():

        #Publisher actions
        input_1_pub.publish(cm.get_input_1())

        #Waits for an amount of time to run the loop at the correct speed, set in rospy.Rate()
        rate.sleep()

#True if this program is directly executed. (Not imported)
if __name__ == '__main__':
    try:
        loop()
    except rospy.ROSInterruptException:
        pass
