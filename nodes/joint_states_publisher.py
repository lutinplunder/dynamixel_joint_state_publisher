#!/usr/bin/env python
import rospy
from dynamixel_msgs.msg import JointState as DynamixelJointState
from std_msgs.msg import *
from sensor_msgs.msg import *

class DynamixelJointStatesPublisher:
    def __init__(self):
        self.joint_states_publisher = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self.goal_states_publisher = rospy.Publisher("/joint_goal_states", JointState, queue_size=1)
        self.join_states = {}
        self.goal_states = {}
        while not rospy.has_param("joint_names"):
            rospy.sleep(0.01)
        self.joint_names = rospy.get_param("joint_names")
        self.js = JointState(header=Header(stamp=rospy.Time.now()),
                             name=self.joint_names,
                             position=[0.0 for i in xrange(len(self.joint_names))],
                             velocity=[0.0 for i in xrange(len(self.joint_names))],
                             effort=[0.0 for i in xrange(len(self.joint_names))])
        self.current_stamp = self.js.header.stamp

        self.gs = JointState(header=Header(stamp=rospy.Time.now()),
                             name=self.joint_names,
                             position=[0.0 for i in xrange(len(self.joint_names))],
                             velocity=[0.0 for i in xrange(len(self.joint_names))],
                             effort=[0.0 for i in xrange(len(self.joint_names))])

        """ Setup handles """
        for name in self.joint_names:
            rospy.Subscriber("/syropod/" + name + "/state", DynamixelJointState, self.handle_joint_state)
        
        self.received = [0 for i in xrange(len(self.joint_names))]
#         rospy.Timer(rospy.Duration(0.1), self.broadcast, False)
        rospy.spin()
            
    def handle_joint_state(self, msg):
        if msg.header.stamp.to_sec() > self.current_stamp.to_sec():
            self.current_stamp = msg.header.stamp        
        names = self.join_states.keys()        
        idx = self.joint_names.index(msg.name)    
#         if idx == 1: print '------------------------\n'
#         if idx == 1: print idx,self.js.position[idx]    
        self.js.position[idx] = msg.current_pos
        self.js.velocity[idx] = msg.velocity
        self.js.effort[idx] = msg.load
        self.js.header = Header(stamp=self.current_stamp)
#         if idx == 1: print idx,self.js.position[idx]    
        self.gs.position[idx] = msg.goal_pos
        self.gs.velocity[idx] = 0
        self.gs.effort[idx] = 0
        self.gs.header = self.js.header
#         if idx == 1: print idx,self.js.position[idx]    
        self.received[idx] = 1
        if sum(self.received) == 18:
            self.joint_states_publisher.publish(self.js)
            self.goal_states_publisher.publish(self.gs) 
            self.received = [0*i for i in self.received]
#         if idx == 1: print idx,self.js.position[idx]    
        
    def broadcast(self, event):
        self.joint_states_publisher.publish(self.js)
        self.goal_states_publisher.publish(self.gs) 
           
if __name__ == "__main__":
    rospy.init_node("joint_states_publisher")
    DynamixelJointStatesPublisher()
