import rospy
import tf
import os
import math

from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from tf.transformations import quaternion_from_euler

# Roboter
WHEEL_RADIUS     = 0.0335  # Radradius in Metern
WHEELBASE        = 0.1     # Spurbreite (Abstand Rad zu Rad) in Metern
TICKS_RESOLUTION = 144     # Encoder-Ticks pro volle Radumdrehung

# Toleranzen 
DIST_TOLERANCE    = 0.04   
HEADING_TOLERANCE = 0.05   

# Geschwindigkeitsgrenzen 
MAX_SPEED = 0.6
MIN_SPEED = 0.2

class PIDController:
    """
    Kp : Proportionalanteil — reagiert auf aktuelle Abweichung
    Ki : Integralanteil     — gleicht bleibende Regelabweichung aus
    Kd : Differentialanteil —  dämpft Ueberschwingen
    """

    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self._integral   = 0.0
        self._prev_error = 0.0

    def reset(self):
        """
        Setzt den internen Zustand zwischen zwei Punkten zurück
        """
        self._integral   = 0.0
        self._prev_error = 0.0

    def compute(self, error, dt):
        """
        Berechnet u für den aktuellen Fehler.

        Args:
            error : aktuelle Regelabweichung e(t)
            dt    : Zeitschritt in Sekunden seit dem letzten Aufruf

        Returns:
            u : Stellgroesse
        """
        
        p = self.Kp * error

        self._integral += error * dt
        i = self.Ki * self._integral

        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        d = self.Kd * derivative

        self._prev_error = error
        return p + i + d

class DriveToGoalRegulated:

    def __init__(self, robot_name):
        self.robot_name = robot_name
        rospy.init_node('drive_to_goal_PID', anonymous=True)

        # Publisher 
        cmd_topic = '/' + robot_name + '/wheels_driver_node/wheels_cmd'
        self.pub = rospy.Publisher(cmd_topic, WheelsCmdStamped, queue_size=10)

        # Subscriber
        rospy.Subscriber('/' + robot_name + '/left_wheel_encoder_driver_node/tick',
                         WheelEncoderStamped, self.cb_left)
        rospy.Subscriber('/' + robot_name + '/right_wheel_encoder_driver_node/tick',
                         WheelEncoderStamped, self.cb_right)

        # TF-Broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Odometrie-Pose
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # Encoder
        self.left_ticks  = None
        self.right_ticks = None
        self.prev_left   = None
        self.prev_right  = None

        # Zwei PID-Regler
        # pid_dist: Regelgroesse = Abstand zum Ziel -> v
        # pid_heading: Regelgroesse = Winkel zum Ziel -> omega
        # Parameter experimentell anpassen
        self.pid_dist    = PIDController(Kp=0.5,  Ki=0.01, Kd=0.05)
        self.pid_heading = PIDController(Kp=2.0,  Ki=0.01, Kd=0.1)

        rospy.on_shutdown(self.stop)
        rospy.sleep(2.0)

    # Encoder-Callbacks

    def cb_left(self, msg):
        self.left_ticks = msg.data

    def cb_right(self, msg):
        self.right_ticks = msg.data

    # Hilfsfunktionen

    def set_wheels(self, vel_left, vel_right):
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left  = float(vel_left)
        cmd.vel_right = float(vel_right)
        self.pub.publish(cmd)

    def stop(self):
        self.set_wheels(0.0, 0.0)

    def wait_for_encoders(self):
        while self.left_ticks is None or self.right_ticks is None:
            rospy.sleep(0.05)

    def v_omega_to_wheels(self, v, omega):
        """Rechnet v und omega in Radgeschwindigkeiten um.
        """
        vel_left  = (v - omega * WHEELBASE / 2) / WHEEL_RADIUS
        vel_right = (v + omega * WHEELBASE / 2) / WHEEL_RADIUS

        # Skalieren falls MAX_SPEED ueberschritten
        max_val = max(abs(vel_left), abs(vel_right))
        if max_val > MAX_SPEED:
            scale     = MAX_SPEED / max_val
            vel_left  *= scale
            vel_right *= scale

        return vel_left, vel_right

    # Odometrie

    def update_odometry(self):
        """
        Aktualisiert die Pose (x, y, theta) aus den Encoder-Tick-Differenzen.
        """
        if self.prev_left is None:
            self.prev_left  = self.left_ticks
            self.prev_right = self.right_ticks
            return

        delta_left  = self.left_ticks  - self.prev_left
        delta_right = self.right_ticks - self.prev_right
        self.prev_left  = self.left_ticks
        self.prev_right = self.right_ticks

        if delta_left == 0 and delta_right == 0:
            return

        d_left = (delta_left  / TICKS_RESOLUTION) * 2 * math.pi * WHEEL_RADIUS
        d_right = (delta_right / TICKS_RESOLUTION) * 2 * math.pi * WHEEL_RADIUS

        v = (d_right + d_left) / 2
        omega = (d_right - d_left) / WHEELBASE

        self.x += v * math.cos(self.theta)
        self.y += v * math.sin(self.theta)
        self.theta = math.atan2(
            math.sin(self.theta + omega),
            math.cos(self.theta + omega)
        )

    def publish_transform(self):
        q = quaternion_from_euler(0, 0, self.theta)
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0), q,
            rospy.Time.now(),
            self.robot_name + "/base", "map"
        )

    # Pose-Regler (Solange Abstand > Toleranz)

    def drive_to_pose(self, goal_x, goal_y, goal_theta=None):
        """Fährt zur Zielposition mit kontinuierlichem PID-Pose-Regler.

        Solange (Abstand zum Ziel > Toleranz):
            1. Odometrie aktualisieren  (wo ist der Roboter jetzt)
            2. Abstandsfehler und Richtungsfehler berechnen
            3. PID-Regler berechnet v und omega
            4. Radgeschwindigkeiten setzen

          Stoppe den Roboter.

          Korrigiere Endorientierung auf goal_theta
        """
        self.pid_dist.reset()
        self.pid_heading.reset()
        prev_time = rospy.Time.now()
        rate = rospy.Rate(20)

        # Fahre zum ziel
        while not rospy.is_shutdown():
            self.update_odometry()
            self.publish_transform()

            dx             = goal_x - self.x
            dy             = goal_y - self.y
            dist_error     = math.sqrt(dx**2 + dy**2)

            # Abbruch wenn Ziel erreicht
            if dist_error < DIST_TOLERANCE:
                break

            # Winkelfehler: Differenz zwischen Zielrichtung und aktueller Ausrichtung
            angle_to_goal  = math.atan2(dy, dx)
            heading_error  = math.atan2(
                math.sin(angle_to_goal - self.theta),
                math.cos(angle_to_goal - self.theta)
            )

            # Zeitschritt für I- und D-Anteil
            now       = rospy.Time.now()
            dt        = (now - prev_time).to_sec()
            prev_time = now

            # PID auf Abstand -> v
            # cos(heading_error) dämpft v wenn der Roboter schief steht:
            # bei 0 Grad => volle Geschwindigkeit, bei 90 Grad => v = 0
            v     = self.pid_dist.compute(dist_error, dt) * max(0.0, math.cos(heading_error))

            # PID auf Winkel -> omega
            omega = self.pid_heading.compute(heading_error, dt)

            vel_left, vel_right = self.v_omega_to_wheels(v, omega)

            rospy.loginfo(
                f"Pose: ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.theta):.0f} Grad) | "
                f"dist={dist_error:.3f}m hdg={math.degrees(heading_error):.1f} Grad | "
                f"v={v:.3f} w={omega:.3f} | vL={vel_left:.3f} vR={vel_right:.3f}"
            )
            self.set_wheels(vel_left, vel_right)
            rate.sleep()

        self.stop()
        rospy.sleep(0.3)

        # Endorientierung korrigieren
        if goal_theta is not None:
            self.correct_heading(goal_theta)

    def correct_heading(self, goal_theta):
        """
        Korrigiert die Endorientierung auf goal_theta mit PID-Regler.
        """
        pid_final = PIDController(Kp=1.5, Ki=0.005, Kd=0.05)
        prev_time = rospy.Time.now()
        rate      = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.update_odometry()
            self.publish_transform()

            heading_error = math.atan2(
                math.sin(goal_theta - self.theta),
                math.cos(goal_theta - self.theta)
            )

            # Solange Winkelabweichung > Toleranz
            if abs(heading_error) < HEADING_TOLERANCE:
                break

            now = rospy.Time.now()
            dt = (now - prev_time).to_sec()
            prev_time = now

            omega = pid_final.compute(heading_error, dt)
            vel_left, vel_right = self.v_omega_to_wheels(0.0, omega)

            self.set_wheels(vel_left, vel_right)
            rate.sleep()

        self.stop()
        rospy.sleep(0.3)



    def run(self, goals):
        """
        Fährt eine Liste von Zielpositionen nacheinander an.

        goals: Liste von (x, y) oder (x, y, theta) Tupeln.
               Koordinaten sind absolut vom Startpunkt (0,0) aus.
               theta ist optional und gibt die gewünschte Endorientierung an.

        Teilaufgabe a: ein Wegpunkt  -> goals = [(0.5, 0.0)]
        Teilaufgabe b: mehrere -> goals = [(0.5, 0.0), (0.5, 0.3), ...]
        """
        self.wait_for_encoders()
        self.prev_left  = self.left_ticks
        self.prev_right = self.right_ticks

        for i, goal in enumerate(goals):
            goal_x = goal[0]
            goal_y = goal[1]
            goal_theta = goal[2] if len(goal) > 2 else None

            rospy.loginfo(
                f"Wegpunkt {i+1}/{len(goals)}: "
                f"Ziel=({goal_x:.2f}, {goal_y:.2f}) | "
                f"Aktuelle Pose: ({self.x:.2f}, {self.y:.2f}, "
                f"{math.degrees(self.theta):.1f} Grad)"
            )

            self.drive_to_pose(goal_x, goal_y, goal_theta)

            # Restfehler ausgeben
            err = math.sqrt((goal_x - self.x)**2 + (goal_y - self.y)**2)
            rospy.loginfo(
                f"Wegpunkt {i+1} abgeschlossen. "
                f"Ist=({self.x:.3f}, {self.y:.3f}) | "
                f"Soll=({goal_x:.2f}, {goal_y:.2f}) | "
                f"Fehler={err*100:.1f} cm"
            )

        rospy.loginfo("Alle Wegpunkte abgefahren.")


if __name__ == '__main__':
    robot = DriveToGoalRegulated(os.getenv('VEHICLE_NAME', 'pi'))


    robot.run(goals=[
        (0.5, 0.0),            # Wegpunkt 1
        # (0.5, 0.3),          # Wegpunkt 2
        # (0.0, 0.0, 0.0),     # Wegpunkt 3
    ])
