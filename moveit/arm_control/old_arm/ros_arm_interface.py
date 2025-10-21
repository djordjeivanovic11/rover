import numpy as np
from rclpy.node import Node
import rclpy

from arm import *

from interfaces.msg import Position
from interfaces.srv import IK

from std_msgs.msg import Int32MultiArray, Float32MultiArray

from scipy.spatial.transform import Rotation

#https://docs.ros.org/en/crystal/Tutorials/Custom-ROS2-Interfaces.html#create-a-new-package
#https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html#fields

L1 = 44.7
L2 = 31.5
L3 = 8.8
L4 = 13.7
#Joint([0,0,1],[0,0,0]),
TICKS_PER_REV = 1600
gear_ratios = [50,100,100,50,50*22/9,1] #last one's probably wrong but I'll fix later

class ArmService(Node):
    def __init__(self):
        super().__init__('arm_service')
        self.failure_reason = 0
        self.joints = [
            Joint([1,0,0],[0,0,0]),
            Joint([-1,0,0],[0,L1,0]),
            Joint([0,1,0],[0,0,0]),
            Joint([-1,0,0],[0,L1+L2+L3,0])
        ]
        
        self.M = np.array([[1,0,0,0],
                [0,1,0,L1+L2+L3+L4],
                [0,0,1,0],
                [0,0,0,1]])
        
        self._arm = Arm(self.M,
                        *self.joints)
        
        self.publisher_ = self.create_service(IK, 'arm_position', self.arm_callback)
        self.get_pos_ = self.create_subscription(Int32MultiArray,'get_arm_position',self.get_pos_callback,10)
        self.fk_pos_publisher = self.create_publisher(Float32MultiArray, 'current_cart',10)
        self.fk_rot_publisher = self.create_publisher(Float32MultiArray, 'current_rot',10)
        timer_period = 0.01  # seconds
        self.fk_timer = self.create_timer(timer_period, self.get_cartesian)
        self.current_arm_pos = [0,0,0,0,0,0]

    def get_cartesian(self):
        arm_angles = np.array(self.current_arm_pos)
        T = self._arm.FK(arm_angles[1:5])
        pos = T[0:3,3]
        rot = T[0:3,0:3]
        rot = Rotation.from_matrix(rot)
        rot = rot.as_euler('xyz',degrees=False)
        self.fk_pos_publisher.publish(Float32MultiArray(data=[pos[0],pos[1],pos[2]]))
        self.fk_rot_publisher.publish(Float32MultiArray(data=[float(rot[0]),float(rot[1]),float(rot[2])]))
        
        
        
    def get_pos_callback(self, msg):
        inp = np.array(list(msg.data))
        angles = list(inp*2*np.pi/TICKS_PER_REV/gear_ratios)
        angles[4] *= -1
        self.current_arm_pos = angles
        
    def failure(self,response):
        response.valid = self.failure_reason
        return response
    
    def arm_callback(self, request, response):
        pos = [request.x,request.y,request.z]
        rotation = [request.rx,request.ry,request.rz]
        
        self._arm.setAngles(self.current_arm_pos[1:5])
        ik_out = self._arm.IK(pos,*rotation)
        
        response.valid = (ik_out is None)
        print(response.valid,ik_out)
        if ik_out is not None:
            print("input",pos)
            print("fk backwards",self._arm.FK(ik_out)[0:3,3])
            self.failure_reason = 0
            fail = False
            ik_out = [0.0,*ik_out]
            #no bounds on first angle, just no excess rotations
            while ik_out[0] > np.pi:
                ik_out[0] -= 2*np.pi
            while ik_out[0] < -np.pi:
                ik_out[0] += 2*np.pi
            #bounds for second angle
            while ik_out[1] > np.pi:
                ik_out[1] -= 2*np.pi
            while ik_out[1] < 0:
                ik_out[1] += 2*np.pi
            if ik_out[1] > np.pi:
                self.failure_reason = -2
                fail = True
            #bounds for third angle
            while ik_out[2] > np.pi/2:
                ik_out[2] -= 2*np.pi
            while ik_out[2] < -np.pi/2:
                ik_out[2] += 2*np.pi
            if ik_out[2] > np.pi/2:
                self.failure_reason = -3
                fail = True
            
            #no bounds on fourth angle, just no excess rotations
            while ik_out[3] > np.pi:
                ik_out[3] -= 2*np.pi
            while ik_out[3] < -np.pi:
                ik_out[3] += 2*np.pi
            #no failure condition
            while ik_out[4] > np.pi/2:
                ik_out[4] -= 2*np.pi
            while ik_out[4] < -np.pi/2:
                ik_out[4] += 2*np.pi
            if ik_out[4] > np.pi/2:
                self.failure_reason = -5
                fail = True
            
            response.out.joint1 = ik_out[0]
            response.out.joint2 = ik_out[1]
            response.out.joint3 = ik_out[2]
            response.out.joint4 = ik_out[3]
            response.out.joint5 = ik_out[4]
            
            if (fail):
                return self.failure(response)
            #response.out.joint5 = ik_out[4]
            #response.out.joint6 = ik_out[5]
            print("failure,",response.valid)
            return response
        else:
            self.failure_reason = 1
            return self.failure(response)
        
    
def main(args=None):
    rclpy.init(args=args)
    arm_service = ArmService()
    rclpy.spin(arm_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()