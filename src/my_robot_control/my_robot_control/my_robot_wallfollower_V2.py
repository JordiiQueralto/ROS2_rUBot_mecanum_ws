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
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        FRONT = []
        FR_RIGHT = []
        RIGHT = []
        BR = []
        BACK = []

        # CLASSIFICACIÓ DELS SECTORS
        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue

            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            if -30 <= ang <= 30:
                FRONT.append(d)
            elif -80 <= ang < -30:
                FR_RIGHT.append(d)
            elif -140 <= ang < -80:
                RIGHT.append(d)
            elif -170 <= ang < -140:
                BR.append(d)
            elif 150 <= ang <= 180 or -180 <= ang <= -170:
                BACK.append(d)

        # Distàncies mínimes
        min_f = min(FRONT) if FRONT else float('inf')
        min_fr = min(FR_RIGHT) if FR_RIGHT else float('inf')
        min_r = min(RIGHT) if RIGHT else float('inf')
        min_br = min(BR) if BR else float('inf')
        min_back = min(BACK) if BACK else float('inf')

        twist = Twist()
        action = ""


        # =================================================
        #                 REGLES HOLONÒMIQUES
        # =================================================

        # 1. FRONT → moure cap a l'esquerra
        if min_f < self.base_distance:
            twist.linear.x = 0.0
            twist.linear.y = self.v_lin         # esquerra
            twist.angular.z = 0.0
            action = f"FRONT {min_f:.2f} m → move LEFT (vy>0)"

        # 2. FRONT-RIGHT → cap al front-esquerra
        elif min_fr < self.base_distance:
            twist.linear.x = self.v_lin * 0.6   # endavant
            twist.linear.y = self.v_lin * 0.6   # esquerra
            twist.angular.z = 0.0
            action = f"FRONT-RIGHT {min_fr:.2f} m → move FRONT-LEFT"

        # 3. RIGHT → avança mantenint distància
        elif math.isfinite(min_r):
            # control de la distància al mur
            error = min_r - self.base_distance

            twist.linear.x = self.v_lin         # endavant
            twist.linear.y = -error * 0.5        # ajust lateral proporcional
            twist.angular.z = 0.0

            action = (
                f"RIGHT {min_r:.2f} m → FORWARD + adjust "
                f"vy={twist.linear.y:.2f}"
            )

        # 4. BACK-RIGHT → cap al front-dreta
        elif math.isfinite(min_br):
            twist.linear.x = self.v_lin * 0.6    # endavant
            twist.linear.y = -self.v_lin * 0.6   # dreta
            twist.angular.z = 0.0
            action = f"BACK-RIGHT {min_br:.2f} m → move FRONT-RIGHT"

        # 5. BACK → cap a la dreta
        elif math.isfinite(min_back):
            twist.linear.x = 0.0
            twist.linear.y = -self.v_lin         # dreta
            twist.angular.z = 0.0
            action = f"BACK {min_back:.2f} m → move RIGHT"

        else:
            twist = Twist()
            action = "No wall detected → STOP"


        # Guarda i publica
        self.cmd = twist

        if action != self._last_action_logged:
            self.get_logger().info(action)
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
