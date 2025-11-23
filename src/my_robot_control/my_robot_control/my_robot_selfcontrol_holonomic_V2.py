
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import random


class RobotSelfControl(Node):

    def __init__(self):
        super().__init__('robot_selfcontrol_node')

        # Configurable parameters
        self.declare_parameter('distance_limit', 0.4)
        self.declare_parameter('critical_distance', 0.2)
        self.declare_parameter('speed_factor', 1.0)
        self.declare_parameter('forward_speed', 0.4)
        self.declare_parameter('rotation_speed', 0.8)
        self.declare_parameter('time_to_stop', 30.0)

        self._distanceLimit = self.get_parameter('distance_limit').value
        self._criticalDistance = self.get_parameter('critical_distance').value
        self._speedFactor = self.get_parameter('speed_factor').value
        self._forwardSpeed = self.get_parameter('forward_speed').value
        self._rotationSpeed = self.get_parameter('rotation_speed').value
        self._time_to_stop = self.get_parameter('time_to_stop').value

        self._msg = Twist()
        self._msg.linear.x = self._forwardSpeed * self._speedFactor
        self._msg.angular.z = 0.0

        # Estats del robot
        self._state = "FORWARD"  # FORWARD, REVERSE, TURN
        self._reverse_start_time = 0
        self._turn_start_time = 0
        self._turn_direction = 0  # -1: dreta, 1: esquerra

        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self._shutting_down = False
        self._last_info_time = self.start_time
        self._last_speed_time = self.start_time

    def timer_callback(self):
        if self._shutting_down:
            return
        
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        elapsed_time = now_sec - self.start_time

        # Control d'estats temporitzats
        if self._state == "REVERSE":
            if now_sec - self._reverse_start_time > 1.0:  # 1 segon marxa enrere
                self._state = "TURN"
                self._turn_start_time = now_sec
                self._turn_direction = random.choice([-1, 1])  # Direcció aleatòria
                self.get_logger().info("Canviant a estat: TURN")

        elif self._state == "TURN":
            if now_sec - self._turn_start_time > 1.5:  # 1.5 segons girant
                self._state = "FORWARD"
                self.get_logger().info("Canviant a estat: FORWARD")

        self._cmdVel.publish(self._msg)

        if now_sec - self._last_speed_time >= 1:
            self.get_logger().info(f"Estat: {self._state} | Vx: {self._msg.linear.x:.2f} m/s | w: {self._msg.angular.z:.2f} rad/s")
            self._last_speed_time = now_sec
            
        if elapsed_time >= self._time_to_stop:
            self.stop()
            self.timer.cancel()
            self.get_logger().info("Robot aturat per temps")
            rclpy.try_shutdown()

    def stop(self):
        """Atura el robot"""
        self._msg.linear.x = 0.0
        self._msg.angular.z = 0.0
        self._cmdVel.publish(self._msg)
        self._shutting_down = True

    def laser_callback(self, scan):
        if self._shutting_down:
            return

        # Convert angles a graus
        angle_min_deg = scan.angle_min * 180.0 / math.pi
        angle_increment_deg = scan.angle_increment * 180.0 / math.pi

        # Filtra lectures vàlides
        valid_readings = []
        front_readings = []
        
        for i, distance in enumerate(scan.ranges):
            angle_deg = angle_min_deg + i * angle_increment_deg
            if angle_deg > 180.0:
                angle_deg -= 360.0
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            
            valid_readings.append((distance, angle_deg))
            if -45 <= angle_deg <= 45:  # Zona frontal ampla
                front_readings.append(distance)

        if not valid_readings:
            return

        # Distància mínima frontal
        min_front_distance = min(front_readings) if front_readings else float('inf')
        
        # Obstacle més proper general
        closest_distance, angle_closest = min(valid_readings, key=lambda x: x[0])

        # Determinar zona
        if -30 <= angle_closest <= 30:
            zone = "FRONT"
        elif 30 < angle_closest <= 90:
            zone = "FRONT_LEFT"
        elif -90 <= angle_closest < -30:
            zone = "FRONT_RIGHT"
        elif 90 < angle_closest <= 150:
            zone = "LEFT"
        elif -150 <= angle_closest < -90:
            zone = "RIGHT"
        else:
            zone = "OUTSIDE"

        now = self.get_clock().now().nanoseconds * 1e-9
        
        if now - self._last_info_time >= 1:
            self.get_logger().info(f"[SENSOR] Dist: {closest_distance:.2f}m | Front: {min_front_distance:.2f}m | Angle: {angle_closest:.0f}° | Zona: {zone}")
            self._last_info_time = now

        # MÀQUINA D'ESTATS PRINCIPAL
        if self._state == "FORWARD":
            self.handle_forward_state(min_front_distance, closest_distance, zone, angle_closest, now)
        elif self._state == "REVERSE":
            self.handle_reverse_state()
        elif self._state == "TURN":
            self.handle_turn_state()

    def handle_forward_state(self, min_front_distance, closest_distance, zone, angle_closest, now):
        """Gestiona l'estat de moviment cap endavant"""
        
        # CRÍTIC: Obstacle molt a prop - marxa enrere immediata
        if min_front_distance < self._criticalDistance:
            self._state = "REVERSE"
            self._reverse_start_time = now
            self._msg.linear.x = -0.3  # Velocitat enrere
            self._msg.angular.z = 0.0
            self.get_logger().warn("⚠️  CRÍTIC! Donant marxa enrere")
            
        # Obstacle a distància normal - maniobra d'evitació
        elif closest_distance < self._distanceLimit:
            if zone == "FRONT":
                # Gir pronunciat
                self._msg.linear.x = 0.1
                self._msg.angular.z = 0.6 * self._rotationSpeed
            elif zone == "FRONT_LEFT":
                self._msg.linear.x = 0.15
                self._msg.angular.z = 0.4 * self._rotationSpeed
            elif zone == "FRONT_RIGHT":
                self._msg.linear.x = 0.15
                self._msg.angular.z = -0.4 * self._rotationSpeed
            elif zone == "LEFT":
                self._msg.linear.x = 0.2
                self._msg.angular.z = 0.2 * self._rotationSpeed
            elif zone == "RIGHT":
                self._msg.linear.x = 0.2
                self._msg.angular.z = -0.2 * self._rotationSpeed
            else:
                self._msg.linear.x = self._forwardSpeed
                self._msg.angular.z = 0.0
        else:
            # Sense obstacles - velocitat normal
            self._msg.linear.x = self._forwardSpeed
            self._msg.angular.z = 0.0

    def handle_reverse_state(self):
        """Gestiona l'estat de marxa enrere"""
        self._msg.linear.x = -0.3  # Manté marxa enrere
        self._msg.angular.z = 0.0  # Sense gir durant la reversa

    def handle_turn_state(self):
        """Gestiona l'estat de gir"""
        self._msg.linear.x = 0.1  # Velocitat molt baixa mentre gira
        self._msg.angular.z = self._turn_direction * self._rotationSpeed  # Gir pronunciat
                      
def main(args=None):
    rclpy.init(args=args)
    robot = RobotSelfControl()
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()