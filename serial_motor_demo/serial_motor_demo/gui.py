import rclpy
from rclpy.node import Node
import math
from tkinter import *
from serial_motor_demo_msgs.msg import MotorCommand
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class MotorGui(Node):

    def __init__(self):
        super().__init__('motor_gui')

        # Publisher für den physischen Roboter
        self.motor_command_pub = self.create_publisher(MotorCommand, 'motor_command', 10)
        # JointState Publisher für das Modell in RViz
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # GUI-Setup
        self.tk = Tk()
        self.tk.title("Serial Motor GUI")
        root = Frame(self.tk)
        root.pack(fill=BOTH, expand=True)

        Label(root, text="Serial Motor GUI").pack()

        # Mode Buttons
        mode_frame = Frame(root)
        mode_frame.pack(fill=X)
        self.mode_lbl = Label(mode_frame, text="PWM Mode")
        self.mode_lbl.pack(side=LEFT)
        self.mode_btn = Button(mode_frame, text="Switch to Feedback Mode", command=self.switch_mode)
        self.mode_btn.pack(expand=True)

        # Motor 1
        m1_frame = Frame(root)
        m1_frame.pack(fill=X)
        Label(m1_frame, text="Motor 1").pack(side=LEFT)
        self.m1 = Scale(m1_frame, from_=-255, to=255, orient=HORIZONTAL)
        self.m1.pack(side=LEFT, fill=X, expand=True)

        # Motor 2
        m2_frame = Frame(root)
        m2_frame.pack(fill=X)
        Label(m2_frame, text="Motor 2").pack(side=LEFT)
        self.m2 = Scale(m2_frame, from_=-255, to=255, orient=HORIZONTAL)
        self.m2.pack(side=LEFT, fill=X, expand=True)

        # Motor Buttons
        motor_btns_frame = Frame(root)
        motor_btns_frame.pack()
        Button(motor_btns_frame, text='Send Once', command=self.send_motor_once).pack(side=LEFT)
        Button(motor_btns_frame, text='Stop Motors', command=self.stop_motors).pack(side=LEFT)

        # Initial Mode
        self.pwm_mode = True

    def switch_mode(self):
        self.pwm_mode = not self.pwm_mode
        if self.pwm_mode:
            self.mode_lbl.config(text="PWM Mode")
            self.mode_btn.config(text="Switch to Feedback Mode")
        else:
            self.mode_lbl.config(text="Feedback Mode")
            self.mode_btn.config(text="Switch to PWM Mode")

    def send_motor_once(self):
        # Nachricht für den physischen Roboter
        motor_msg = MotorCommand()
        motor_msg.is_pwm = self.pwm_mode
        if self.pwm_mode:
            motor_msg.mot_1_req_rad_sec = float(self.m1.get())
            motor_msg.mot_2_req_rad_sec = float(self.m2.get())
        else:
            motor_msg.mot_1_req_rad_sec = float(self.m1.get() * 2 * math.pi)
            motor_msg.mot_2_req_rad_sec = float(self.m2.get() * 2 * math.pi)

        self.motor_command_pub.publish(motor_msg)

        # Nachricht für das Diff-Drive-Plugin in Gazebo
        scaling_factor = 0.5
        twist_msg = Twist()
        twist_msg.linear.x = scaling_factor*float(self.m1.get()) / 255.0  # Skaliert PWM auf lineare Geschwindigkeit
        twist_msg.angular.z = scaling_factor*float(self.m2.get()) / 255.0  # Skaliert PWM auf Winkelgeschwindigkeit
        self.cmd_vel_pub.publish(twist_msg)
        # Debugging
        self.get_logger().info(f"Sent cmd_vel: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}")

        # Nachricht für das Modell in RViz
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['base_to_left_wheel', 'base_to_right_wheel']
        joint_state_msg.position = [
            float(self.m1.get() * 2 * math.pi / 255),  # Skaliert von PWM auf Radianten
            float(self.m2.get() * 2 * math.pi / 255)   # Skaliert von PWM auf Radianten
        ]
        self.joint_state_pub.publish(joint_state_msg)

    def stop_motors(self):
        # Motoren stoppen
        motor_msg = MotorCommand()
        motor_msg.is_pwm = self.pwm_mode
        motor_msg.mot_1_req_rad_sec = 0.0
        motor_msg.mot_2_req_rad_sec = 0.0
        self.motor_command_pub.publish(motor_msg)

        # Diff-Drive stoppen
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

        # Modell in RViz anhalten
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['base_to_left_wheel', 'base_to_right_wheel']
        joint_state_msg.position = [0.0, 0.0]
        self.joint_state_pub.publish(joint_state_msg)

    def update(self):
        self.tk.update()

def main(args=None):
    rclpy.init(args=args)

    motor_gui = MotorGui()

    rate = motor_gui.create_rate(20)
    while rclpy.ok():
        rclpy.spin_once(motor_gui)
        motor_gui.update()

    motor_gui.destroy_node()
    rclpy.shutdown()
