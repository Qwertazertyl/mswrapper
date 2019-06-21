#!/usr/bin/python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64

set_speed_pub = None

def input_1_callback(data):
    if data.data == True:
        set_speed_pub.publish(4)
    else:
        set_speed_pub.publish(2)

def controller():
    global set_speed_pub
    #Create publishers (topic name, message type (must be imported), queue_size (Incomming message queue, if the queue is full, the oldest message will be thrown out)) 
    set_speed_pub = rospy.Publisher('mswrapper/set_speed', Float64, queue_size=10)

    #Create subscriber for each topic, with callback function (topic name, message type (must be imported), callback function)
    rospy.Subscriber('mswrapper/input_1', Bool, input_1_callback)

    #Initialize and name node 
    # In ROS, nodes are uniquely named. If two nodes with the same          
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('controller', anonymous=False)

    # 10hz; Makes best effort at maintaining rate for a loop. rate.sleep() attempts to wait the correct ammount of time for the loop.
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        #Publisher
        #rospy.loginfo("This is asciicom logging information!")

        #Subscribers are handled in the callback

        #General
        rate.sleep() #Waits for an amount of time to run the loop at the correct speed, set in rospy.Rate()


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
