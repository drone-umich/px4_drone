#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
import time
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String


class Control_Keyboard(Node): 
    
    def __init__(self):
        super().__init__("control_keyboard") 

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.setup_publishers(qos_profile)

        pygame.init()
        self.screen = pygame.display.set_mode((320, 240))
        pygame.display.set_caption("Keyboard Control")
        self.timer = self.create_timer(0.1, self.update)  # 10 FPS

        # Estados de los drones
        self.drone1_active = False
        self.drone2_active = False

        # Timers para enviar los mensajes de parada
        self.stop_msg_timer1 = None
        self.stop_msg_timer2 = None

    def setup_publishers(self, qos_profile):
        #self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        #self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        #self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)        
        
        self.pub_control1 = self.create_publisher(String, 'drone1/keyboard', 1)

        self.pub_control2 = self.create_publisher(String, 'drone2/keyboard', 1)

        

    def send_stop_msg1(self):
        msg = 'stop'
        self.pub_control1.publish(msg)

    def send_stop_msg2(self):
        msg = 'stop'
        self.pub_control2.publish(msg)


    def update(self):
        # RC Speed
        manual_speed = float(30)
        msg = String()

        # Comprobar eventos
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
            elif event.type == pygame.KEYDOWN:  # Evento al presionar una tecla
                if event.key == pygame.K_1:
                    self.drone1_active = not self.drone1_active  # Cambiar el estado de drone1 al presionar '1'
                    if self.drone1_active == True:
                        self.get_logger().info("Drone 1 Activado")
                        if self.stop_msg_timer1:
                            self.stop_msg_timer1.cancel()  # Cancelar el timer si está activo
                            self.stop_msg_timer1 = None
                    elif self.drone1_active == False:
                        self.get_logger().info("Drone 1 Desactivado")
                        if not self.stop_msg_timer1:
                            self.stop_msg_timer1 = self.create_timer(5, self.send_stop_msg1)  # Crear el timer si no está activo
                        
                elif event.key == pygame.K_2:
                    self.drone2_active = not self.drone2_active  # Cambiar el estado de drone2 al presionar '2'
                    if self.drone2_active == True:
                        self.get_logger().info("Drone 2 Activado")
                        if self.stop_msg_timer2:
                            self.stop_msg_timer2.cancel()  # Cancelar el timer si está activo
                            self.stop_msg_timer2 = None
                    elif self.drone2_active == False:
                        self.get_logger().info("Drone 2 Desactivado")
                        if not self.stop_msg_timer2:
                            self.stop_msg_timer2 = self.create_timer(5, self.send_stop_msg2)  # Crear el timer si no está activo
                
                if self.drone1_active or self.drone2_active:  # Solo verificar flechas si al menos uno de los drones está activo
                    if event.key == pygame.K_y:
                        if self.drone1_active:
                            msg.data = "arm"
                            self.pub_control1.publish(msg)
                            self.get_logger().info('Drone 1: Arm')
                        if self.drone2_active:
                            msg.data = "arm"
                            self.pub_control2.publish(msg)
                            self.get_logger().info('Drone 2: Arm')
                    elif event.key == pygame.K_o:
                        if self.drone1_active:
                            msg.data = "offboard"
                            self.pub_control1.publish(msg)
                            self.get_logger().info('Drone 1: Offboard')
                        if self.drone2_active:
                            msg.data = "offboard"
                            self.pub_control2.publish(msg)
                            self.get_logger().info('Drone 2: Offboard')
                    elif event.key == pygame.K_UP:
                        if self.drone1_active:
                            msg.data = "up"
                            self.pub_control1.publish(msg)
                            self.get_logger().info('Drone 1: Adelante')
                        if self.drone2_active:
                            msg.data = "up"
                            self.pub_control2.publish(msg)
                            self.get_logger().info('Drone 2: Adelante')
                    elif event.key == pygame.K_DOWN:
                        if self.drone1_active:
                            msg.data = "down"
                            self.pub_control1.publish(msg)
                            self.get_logger().info('Drone 1: Atrás')
                        if self.drone2_active:
                            msg.data = "down"
                            self.pub_control2.publish(msg)
                            self.get_logger().info('Drone 2: Atrás')
                    elif event.key == pygame.K_LEFT:
                        if self.drone1_active:
                            msg.data = "left"
                            self.pub_control1.publish(msg)
                            self.get_logger().info('Drone 1: Izquierda')
                        if self.drone2_active:
                            msg.data = "left"
                            self.pub_control2.publish(msg)
                            self.get_logger().info('Drone 2: Izquierda')
                    elif event.key == pygame.K_RIGHT:
                        if self.drone1_active:
                            msg.data = "right"
                            self.pub_control1.publish(msg)
                            self.get_logger().info('Drone 1: Derecha')
                        if self.drone2_active:
                            msg.data = "right"
                            self.pub_control2.publish(msg)
                            self.get_logger().info('Drone 2: Derecha')
                    elif event.key == pygame.K_w:
                        if self.drone1_active:
                            msg.data = "w"
                            self.pub_control1.publish(msg)
                            self.get_logger().info('Drone 1: Arriba')
                        if self.drone2_active:
                            msg.data = "w"
                            self.pub_control2.publish(msg)
                            self.get_logger().info('Drone 2: Arriba')
                    elif event.key == pygame.K_s:
                        if self.drone1_active:
                            msg.data = "s"
                            self.pub_control1.publish(msg)
                            self.get_logger().info('Drone 1: Abajo')
                        if self.drone2_active:
                            msg.data = "s"
                            self.pub_control2.publish(msg)
                            self.get_logger().info('Drone 2: Abajo')
                    elif event.key == pygame.K_d:
                        if self.drone1_active:
                            msg.data = "d"
                            self.pub_control1.publish(msg)
                            self.get_logger().info('Drone 1: Rotar derecha')
                        if self.drone2_active:
                            msg.data = "d"
                            self.pub_control2.publish(msg)
                            self.get_logger().info('Drone 2: Rotar derecha')
                    elif event.key == pygame.K_a:
                        if self.drone1_active:
                            msg.data = "a"
                            self.pub_control1.publish(msg)
                            self.get_logger().info('Drone 1: Rotar izquierda')
                        if self.drone2_active:
                            msg.data = "a"
                            self.pub_control2.publish(msg)
                            self.get_logger().info('Drone 2: Rotar izquierda')
                    elif event.key == pygame.K_t:
                        if self.drone1_active:
                            msg.data = "takeoff"
                            self.pub_control1.publish(msg)
                            self.get_logger().info('Drone 1: Takeoff')
                        if self.drone2_active:
                            msg.data = "takeoff"
                            self.pub_control2.publish(msg)
                            self.get_logger().info('Drone 2: Takeoff')                            
                    elif event.key == pygame.K_l:
                        if self.drone1_active:
                            msg.data = "land"
                            self.pub_control1.publish(msg)
                            self.get_logger().info('Drone 1: Land')
                        if self.drone2_active:
                            msg.data = "land"
                            self.pub_control2.publish(msg)
                            self.get_logger().info('Drone 2: Land')
                    elif event.key == pygame.K_c:
                        if self.drone1_active:
                            msg.data = "c"
                            self.pub_control1.publish(msg)
                            self.get_logger().info('Drone 1: Curve')
                        if self.drone2_active:
                            msg.data = "c"
                            self.pub_control2.publish(msg)
                            self.get_logger().info('Drone 2: Curve')
            elif event.type == pygame.KEYUP:  # Evento al soltar una tecla
                if event.key in [pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT, pygame.K_w, pygame.K_s, pygame.K_d, pygame.K_a, pygame.K_t, pygame.K_l]:
                    self.get_logger().info('stop')
                    msg.data = 'stop'
                    if self.drone1_active and self.drone2_active:
                        self.pub_control1.publish(msg)
                        self.pub_control2.publish(msg)
                    elif self.drone1_active:
                        self.pub_control1.publish(msg)
                    elif self.drone2_active:
                        self.pub_control2.publish(msg)

        # Actualizar la pantalla
        pygame.display.flip()


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = Control_Keyboard()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)