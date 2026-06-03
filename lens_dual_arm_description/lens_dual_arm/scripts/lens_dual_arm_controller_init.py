import numpy as np
import rclpy
from rclpy.node import Node
import time

from sensor_msgs.msg import JointState
from lens_msgs.srv import MotorEnableSrv
from controller_manager_msgs.srv import (
    LoadController,
    ConfigureController,
    SwitchController,
    ListControllers
)

left_arm_joints = [
    "Left_Shoulder_Pitch_Joint", 
    "Left_Shoulder_Roll_Joint",
    "Left_Shoulder_Yaw_Joint",
    "Left_Elbow_Pitch_Joint",
    "Left_Wrist_Yaw_Joint",
    "Left_Wrist_Roll_Joint",
    "Left_Wrist_Pitch_Joint"
]
right_arm_joints = [
    "Right_Shoulder_Pitch_Joint",
    "Right_Shoulder_Roll_Joint",
    "Right_Shoulder_Yaw_Joint",
    "Right_Elbow_Pitch_Joint",
    "Right_Wrist_Yaw_Joint",
    "Right_Wrist_Roll_Joint",
    "Right_Wrist_Pitch_Joint"
]

#left_arm_joints
dual_arm_joints = left_arm_joints + right_arm_joints +["Left_Gripper_Joint","Right_Gripper_Joint"]

init_position = [0.0]*7

#ros2 controllers 启动
class Ros2ControllersInitilizeNode(Node):
    def __init__(self):
        super().__init__('Ros2_Controllers_Initialized_Node')
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.jointStatesCallback, 10)
        #self.joint_command_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.current_joint_state = JointState()
        self.updated_flag = False
        self.controller_names = ["left_arm_controller", "right_arm_controller", "left_gripper_controller", "right_gripper_controller"]

        self.setMotorClient = self.create_client(MotorEnableSrv, '/MotorEnableSrv')
        while not self.setMotorClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service SetMotorEnable not available, waiting again...')
        print("finish init. Waiting for joint state ... ") 

        #  # 创建服务客户端
        # self.load_controller_client = self.create_client(LoadController, '/controller_manager/load_controller')
        # self.configure_controller_client = self.create_client(ConfigureController, '/controller_manager/configure_controller')
        # self.switch_controller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        # self.list_controller_client =  self.create_client(ListControllers, '/controller_manager/list_controllers')
        
        # while not self.load_controller_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().warn(f'Service /controller_manager/LoadController not available, waiting...')
        # while not self.configure_controller_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().warn(f'Service /controller_manager/ConfigureController not available, waiting...')
        # while not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().warn(f'Service /controller_manager/SwitchController not available, waiting...')
        # while not self.list_controller_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().warn(f'Service /controller_manager/ListControllers not available, waiting...')
                
        # self.get_logger().info('Controller manager node initialized')
    
    def sendMotorEnableRequest(self, enable):
        self.get_logger().info(f'电机开始使能...')
        request = MotorEnableSrv.Request()
        request.client_enable = enable
        future = self.setMotorClient.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                return future.result().service_enabled 
            except Exception as e:
                self.get_logger().error(f'服务调用失败: {e}')
                return
        else:
            self.get_logger().error('服务请求超时')
            return

    def callService(self, service_type, request):
        client = self.clients[service_type]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def loadControllers(self):
        for name in self.controller_names:
            request = LoadController.Request()
            request.name = name
            future = self.load_controller_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response and response.ok:
                self.get_logger().info(f'Successfully loaded controller: {name}')
            else:
                self.get_logger().error(f'Failed to load controller: {name}')
                return False
        return True

    def configureControllers(self):
        for name in self.controller_names:
            request = ConfigureController.Request()
            request.name = name
            future = self.configure_controller_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            if response and response.ok:
                self.get_logger().info(f'Successfully configured controller: {name}')
            else:
                self.get_logger().error(f'Failed to configure controller: {name}')
                return False
        return True

    def switchControllers(self):
        request = SwitchController.Request()
        request.activate_controllers = self.controller_names
        request.deactivate_controllers = []
        request.strictness = True
        
        future = self.switch_controller_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response and response.ok:
            self.get_logger().info(f'Successfully switched controllers.')
            return True
        else:
            self.get_logger().error(f'Failed to switch controllers.')
            return False

    def isControllerActive(self):        
        request = ListControllers.Request()
        future = self.list_controller_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if not response:
            self.get_logger().warn(f'ListControllers service call failed.')
            return False
                
        for controller in self.controller_names:
            found = False
            for controller_info in response.controller:
                if controller_info.name == controller:
                    if controller_info.state == 'active':
                        self.get_logger().info(f'Controller {controller} is active.')
                        found = True
                        break
            if not found:
                self.get_logger().warn(f'Controller {controller} is not active.')
                return False

        self.get_logger().info('All controllers are active.')
        return True        
        
    def jointStatesCallback(self, msg:JointState):
        indices = []
        self.current_joint_state = msg
        self.updated_flag = True

    def moveToInit(self, is_start=True):
        duration = 300

    '''To ensure safe mode transition, the current position must be published before switching to position control mode.'''
    def publishCurrentState(self):
        print("*** publish current joint state...")
        duration = 10
        
        for i in range(0, duration):
            #self.joint_command_pub.publish(self.current_joint_state)
            time.sleep(0.02)
    
    def main(self):
        print("main in")
        if self.sendMotorEnableRequest(True) != True:
            print("电机使能失败")
        else:
            print("电机使能成功")
        # while 1:
        #     if(self.updated_flag):
        #         # if self.loadControllers() and self.configureControllers() and self.switchControllers():
        #         #     print("Controllers loaded and configured successfully.")
        #         # else:
        #         #     print("Failed to load or configure controllers.")
        #         #     break
        #         # while not self.isControllerActive():
        #         #     print("Waiting for controllers to be active...")
        #         #     time.sleep(1)
        #         # # self.publishCurrentState()

        #         if self.sendMotorEnableRequest(True) != True:
        #             print("电机使能失败")
        #             break
        #         else:
        #             print("电机使能成功")
        #         break
        #     else:
        #         time.sleep(0.02)
        #         rclpy.spin_once(self)
        # print("Has moved to init. End of process")

if __name__ == '__main__':
    rclpy.init()
    init_process= Ros2ControllersInitilizeNode()
    init_process.main()
    init_process.destroy_node()
    rclpy.shutdown()
    
    