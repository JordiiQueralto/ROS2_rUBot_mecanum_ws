import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # Parameters
        self.declare_parameter('distance_limit', 0.3)    # desired distance to right wall
        self.declare_parameter('forward_speed', 0.20)    # linear speed
        self.declare_parameter('turn_speed', 0.40)       # angular speed
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)        # band around base_distance (RIGHT)

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # Last commanded twist (will be published periodically)
        self.cmd = Twist()

        # ROS 2 entities
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)

        # Periodic cmd_vel publisher at 10 Hz (0.1 s)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            "WallFollower (RIGHT tol, BACK_RIGHT when closest) - differential drive."
        )

    #--------------------------------------------------------------------
    def stop_watchdog(self):
        """Stop the robot after time_to_stop seconds."""
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    #--------------------------------------------------------------------
    def stop(self):
        """Safe stop: set cmd to zero Twist, try to publish once, stop timers."""
        self._shutting_down = True

        # Set last command to zero
        self.cmd = Twist()

        # Try a final publish (publisher may still be valid even if shutdown started)
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            # Context/publisher may already be invalid -> ignore
            pass

        # Cancel timers safely
        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    #--------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        """Periodic publisher: send the latest cmd_vel at 10 Hz."""
        if self._shutting_down:
            return

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            # If the context or publisher is invalid, ignore
            pass

    #--------------------------------------------------------------------
    def laser_callback(self, scan):
        """Compute control action from LIDAR and update self.cmd."""
        if self._shutting_down:
            return

        min_distance = float('inf')
        angle_closest_distance = 0.0 

        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance < scan.range_min or distance > scan.range_max:
                continue

            if distance < min_distance:
                min_distance = distance 
                angle_closest_distance = math.degrees(scan.angle_min + i * scan.angle_increment)

        #Normalitza l'angle entre -180 i 180
        angle_closest_distance = (angle_closest_distance + 180) % 360 - 180
        
        twist = Twist()
        action = ""

        #Determina on està la paret més propera
        if -45 <= angle_closest_distance <= 45:
            zone = "FRONT"
        elif 45 < angle_closest_distance <= 135:
            zone = "LEFT"
        elif -135 <= angle_closest_distance < -45:
            zone = "RIGHT"
        elif (-180 <= angle_closest_distance < -135) or (135 < angle_closest_distance <= 180):
            zone = "BACK"
        else:
            zone = "UNKNOWN"

         #Lògica de seguiment de qualsevol paret 
        if min_distance == float('inf'):
            # No detecta cap paret - busca girant una mica
            twist.linear.x = self.v_lin * 0.4
            twist.linear.y = 0.0
            twist.angular.z = self.v_ang * 0.3  # Gira una mica per buscar
            action = "No walls → SEARCH"

        elif zone == "FRONT" and min_distance < self.base_distance:
            # Paret frontal - mou's lateralment segons la paret més propera
            if angle_closest_distance >= 0:
                # Paret més a l'esquerra - mou's dreta
                twist.linear.x = 0.0
                twist.linear.y = -self.v_lin
                action = f"FRONT wall {min_distance:.2f}m → MOVE RIGHT"   

            else:
                # Paret més a la dreta - mou's esquerra
                twist.linear.x = 0.0
                twist.linear.y = self.v_lin
                action = f"FRONT wall {min_distance:.2f}m → MOVE LEFT"
            
            
        elif zone == "RIGHT" and min_distance < self.base_distance * 1.5:
            # Paret dreta - segueix-la mantenint distància

            error = min_distance - self.base_distance
            if abs(error) <= self.tol:
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                action = f"RIGHT wall OK {min_distance:.2f}m → FORWARD"
        
            elif error < 0:  # Massa aprop
                twist.linear.x = self.v_lin * 0.6
                twist.linear.y = self.v_lin * 0.4  # Allunya't cap a l'esquerra
                action = f"RIGHT wall CLOSE {min_distance:.2f}m → LEFT-FORWARD"
        
            else:  # Massa lluny
                twist.linear.x = self.v_lin * 0.6
                twist.linear.y = -self.v_lin * 0.4  # Apropa't cap a la dreta
                action = f"RIGHT wall FAR {min_distance:.2f}m → RIGHT-FORWARD"     

        elif zone == "LEFT" and min_distance < self.base_distance * 1.5:
            # Paret esquerra - segueix-la mantenint distància
            error = min_distance - self.base_distance

            if abs(error) <= self.tol:
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                action = f"LEFT wall OK {min_distance:.2f}m → FORWARD"
        
            elif error < 0:  # Massa aprop
                twist.linear.x = self.v_lin * 0.6
                twist.linear.y = -self.v_lin * 0.4  # Allunya't cap a la dreta
                action = f"LEFT wall CLOSE {min_distance:.2f}m → RIGHT-FORWARD"
        
            else:  # Massa lluny
                twist.linear.x = self.v_lin * 0.6
                twist.linear.y = self.v_lin * 0.4  # Apropa't cap a l'esquerra
                action = f"LEFT wall FAR {min_distance:.2f}m → LEFT-FORWARD"
       
        
        elif zone == "BACK" and min_distance < self.base_distance:
            # Paret posterior - segueix endavant (és una paret que estàs deixant enrere)
            twist.linear.x = self.v_lin
            twist.linear.y = 0.0
            action = f"BACK wall {min_distance:.2f}m → FORWARD"
    
        else:
            # Situació normal - segueix endavant
            twist.linear.x = self.v_lin
            twist.linear.y = 0.0
            action = f"Following {zone} wall {min_distance:.2f}m → FORWARD"

        twist.angular.z = 0.0  # Manté orientació

        
        # Update last commanded twist
        self.cmd = twist

        #Logging
        if action != self._last_action_logged:
            self.get_logger().info(f"{action} [Angle: {angle_closest_distance:.1f}°]")
            self._last_action_logged = action

        self._state_action = action
            

    #--------------------------------------------------------------------
    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
