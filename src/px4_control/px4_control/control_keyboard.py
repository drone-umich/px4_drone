#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
import time
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

class Control_Keyboard(Node): 
    
    def __init__(self):
        super().__init__("control_keyboard") 

        self.setup_publishers()
        pygame.init()
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption("Keyboard Control")
        self.timer = self.create_timer(0.1, self.update)  # 10 FPS

    def update(self):
        # RC Speed
        manual_speed = float(30)

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
                    if event.key == pygame.K_UP:
                        if self.drone1_active:
                            
                            self.get_logger().info('Drone 1: Adelante')
                        if self.drone2_active:
                            
                            self.get_logger().info('Drone 2: Adelante')
                    elif event.key == pygame.K_DOWN:
                        if self.drone1_active:
                            
                            self.get_logger().info('Drone 1: Atrás')
                        if self.drone2_active:
                            
                            self.get_logger().info('Drone 2: Atrás')
                    elif event.key == pygame.K_LEFT:
                        if self.drone1_active:
                            
                            self.get_logger().info('Drone 1: Izquierda')
                        if self.drone2_active:
                            
                            self.get_logger().info('Drone 2: Izquierda')
                    elif event.key == pygame.K_RIGHT:
                        if self.drone1_active:
                            
                            self.get_logger().info('Drone 1: Derecha')
                        if self.drone2_active:
                            
                            self.get_logger().info('Drone 2: Derecha')
                    elif event.key == pygame.K_w:
                        if self.drone1_active:
                            
                            self.get_logger().info('Drone 1: Arriba')
                        if self.drone2_active:
                            
                            self.get_logger().info('Drone 2: Arriba')
                    elif event.key == pygame.K_s:
                        if self.drone1_active:
                            
                            self.get_logger().info('Drone 1: Abajo')
                        if self.drone2_active:
                            
                            self.get_logger().info('Drone 2: Abajo')
                    elif event.key == pygame.K_d:
                        if self.drone1_active:
                            
                            self.get_logger().info('Drone 1: Rotar derecha')
                        if self.drone2_active:
                            
                            self.get_logger().info('Drone 2: Rotar derecha')
                    elif event.key == pygame.K_a:
                        if self.drone1_active:
                            
                            self.get_logger().info('Drone 1: Rotar izquierda')
                        if self.drone2_active:
                            
                            self.get_logger().info('Drone 2: Rotar izquierda')
                    elif event.key == pygame.K_t:
                        if self.drone1_active:
                            
                            self.get_logger().info('Drone 1: Takeoff')
                        if self.drone2_active:
                            
                            self.get_logger().info('Drone 2: Takeoff')                            
                    elif event.key == pygame.K_l:
                        if self.drone1_active:
                            
                            self.get_logger().info('Drone 1: Land')
                        if self.drone2_active:
                            
                            self.get_logger().info('Drone 2: Land')
                    elif event.key == pygame.K_c:
                        if self.drone1_active:
                            
                            self.get_logger().info('Drone 1: Curve')
                        if self.drone2_active:
                            
                            self.get_logger().info('Drone 2: Curve')
            elif event.type == pygame.KEYUP:  # Evento al soltar una tecla
                if event.key in [pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT, pygame.K_w, pygame.K_s, pygame.K_d, pygame.K_a]:
                    self.get_logger().info('stop')
                    #msg.linear.x = float(0)
                    #msg.linear.y = float(0)
                    #msg.linear.z = float(0)
                    #msg.angular.z = float(0)
                    #if self.drone1_active and self.drone2_active:
                        #self.pub_control1.publish(msg)
                        #self.pub_control2.publish(msg)
                    #elif self.drone1_active:
                        #self.pub_control1.publish(msg)
                    #elif self.drone2_active:
                        #self.pub_control2.publish(msg)

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