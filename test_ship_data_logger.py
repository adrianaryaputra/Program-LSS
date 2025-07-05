import unittest
from unittest.mock import Mock, MagicMock, patch, mock_open
import math
import io # For StringIO

# Import the script to be tested
import ship_data_logger

# Import ROS message types (actual types for constructing realistic mocks)
from geometry_msgs.msg import Twist, TwistStamped, Vector3, Point, Quaternion as GeoQuaternion # Renamed to avoid clash
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64

class TestShipDataLogger(unittest.TestCase):

    def setUp(self):
        # Reset latest_data before each test
        # Make a deep copy of the original structure if it's complex,
        # but for dict of Nones, direct re-assignment is fine.
        ship_data_logger.latest_data = {
            'timestamp': None, 'rudder_angle': None, 'thruster_%': None,
            'pos_NED_x': None, 'pos_NED_y': None,
            'pos_GLB_lat': None, 'pos_GLB_long': None, 'pos_GLB_alt': None,
            'acc_BDY_x': None, 'acc_BDY_y': None, 'acc_BDY_z': None,
            'vel_BDY_x': None, 'vel_BDY_y': None, 'vel_BDY_z': None,
            'vel_NED_x': None, 'vel_NED_y': None, 'vel_NED_z': None,
            'vel_GLB_ground_speed': None, 'vel_GLB_course_over_ground': None, 'vel_GLB_vertical_speed': None,
            'att_NED_roll': None, 'att_NED_pitch': None, 'att_NED_yaw': None,
            'attvel_BDY_rollrate': None, 'attvel_BDY_pitchrate': None, 'attvel_BDY_yawrate': None,
        }
        # Ensure tf_trans and np are available for most tests, can be patched to None for specific tests
        if ship_data_logger.tf_trans is None or ship_data_logger.np is None:
            # Mock them if they failed to import in the main script, so tests can proceed
            # This assumes the main script's try-except for tf_trans/np handles the warning
            ship_data_logger.tf_trans = Mock()
            ship_data_logger.tf_trans.quaternion_from_euler = Mock(return_value=(0,0,0,1))
            ship_data_logger.tf_trans.quaternion_matrix = Mock(return_value=ship_data_logger.np.identity(4) if ship_data_logger.np else Mock()) # type: ignore
            ship_data_logger.np = Mock()
            ship_data_logger.np.array = lambda x: x # simple pass-through
            ship_data_logger.np.dot = lambda a,b: b # simple pass-through for vector
            ship_data_logger.np.identity = lambda x: [[1,0,0],[0,1,0],[0,0,1]] # simple 3x3 identity


    def test_command_callback(self):
        mock_msg = Twist()
        mock_msg.linear.x = 1.5
        mock_msg.angular.z = -0.5
        ship_data_logger.command_callback(mock_msg)
        self.assertEqual(ship_data_logger.latest_data['thruster_%'], 1.5)
        self.assertEqual(ship_data_logger.latest_data['rudder_angle'], -0.5)

    def test_odom_callback(self):
        mock_msg = Odometry()
        mock_msg.pose.pose.position.x = 10.0
        mock_msg.pose.pose.position.y = -20.0
        # pos_NED_z is not set by callback as it's removed
        ship_data_logger.odom_callback(mock_msg)
        self.assertEqual(ship_data_logger.latest_data['pos_NED_x'], 10.0)
        self.assertEqual(ship_data_logger.latest_data['pos_NED_y'], -20.0)

    def test_global_pos_callback(self):
        mock_msg = NavSatFix()
        mock_msg.latitude = 34.0522
        mock_msg.longitude = -118.2437
        mock_msg.altitude = 100.0
        ship_data_logger.global_pos_callback(mock_msg)
        self.assertEqual(ship_data_logger.latest_data['pos_GLB_lat'], 34.0522)
        self.assertEqual(ship_data_logger.latest_data['pos_GLB_long'], -118.2437)
        self.assertEqual(ship_data_logger.latest_data['pos_GLB_alt'], 100.0)

    def test_yaw_ned_callback(self):
        mock_msg = Float64()
        mock_msg.data = 1.57
        ship_data_logger.yaw_ned_callback(mock_msg)
        self.assertEqual(ship_data_logger.latest_data['att_NED_yaw'], 1.57)

    def test_imu_callback_basic(self):
        # Mock tf_trans for consistent testing if not fully mocked in setUp
        original_tf_trans = ship_data_logger.tf_trans
        ship_data_logger.tf_trans = Mock()
        # Simulate quaternion_from_euler if needed, or ensure euler_from_quaternion is mocked
        ship_data_logger.tf_trans.euler_from_quaternion = Mock(return_value=(0.1, 0.2, 0.3)) # r, p, y

        mock_msg = Imu()
        mock_msg.orientation = GeoQuaternion(x=0, y=0, z=0, w=1) # Identity
        mock_msg.angular_velocity = Vector3(x=0.1, y=0.2, z=0.3)
        mock_msg.linear_acceleration = Vector3(x=0.4, y=0.5, z=0.6)

        ship_data_logger.imu_callback(mock_msg)

        self.assertAlmostEqual(ship_data_logger.latest_data['att_NED_roll'], 0.1)
        self.assertAlmostEqual(ship_data_logger.latest_data['att_NED_pitch'], 0.2)
        # Yaw is not set by imu_callback by design
        self.assertEqual(ship_data_logger.latest_data['attvel_BDY_rollrate'], 0.1)
        self.assertEqual(ship_data_logger.latest_data['attvel_BDY_pitchrate'], 0.2)
        self.assertEqual(ship_data_logger.latest_data['attvel_BDY_yawrate'], 0.3)
        self.assertEqual(ship_data_logger.latest_data['acc_BDY_x'], 0.4)
        self.assertEqual(ship_data_logger.latest_data['acc_BDY_y'], 0.5)
        self.assertEqual(ship_data_logger.latest_data['acc_BDY_z'], 0.6)
        ship_data_logger.tf_trans = original_tf_trans # Restore

    def test_imu_callback_no_tf(self):
        original_tf_trans = ship_data_logger.tf_trans
        ship_data_logger.tf_trans = None # Simulate tf not being available
        mock_msg = Imu()
        mock_msg.orientation = GeoQuaternion(x=0,y=0,z=0,w=1)
        mock_msg.angular_velocity = Vector3(x=0,y=0,z=0)
        mock_msg.linear_acceleration = Vector3(x=0,y=0,z=0)
        ship_data_logger.imu_callback(mock_msg)
        self.assertIsNone(ship_data_logger.latest_data['att_NED_roll'])
        self.assertIsNone(ship_data_logger.latest_data['att_NED_pitch'])
        ship_data_logger.tf_trans = original_tf_trans # Restore

    @patch('ship_data_logger.np')
    @patch('ship_data_logger.tf_trans')
    def test_body_vel_callback_no_rotation(self, mock_tf_trans, mock_np):
        # Setup mocks for numpy and tf_trans to behave as expected for this test
        mock_np.array = lambda x: list(x) # simple list conversion for this test
        mock_np.dot = lambda M, v: list(v) # Identity transform for no rotation

        # Identity quaternion for no rotation
        mock_tf_trans.quaternion_from_euler.return_value = (0,0,0,1)
        # Identity rotation matrix
        identity_matrix_4x4 = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        mock_tf_trans.quaternion_matrix.return_value = identity_matrix_4x4

        ship_data_logger.latest_data['att_NED_roll'] = 0.0
        ship_data_logger.latest_data['att_NED_pitch'] = 0.0
        ship_data_logger.latest_data['att_NED_yaw'] = 0.0

        mock_msg = TwistStamped()
        mock_msg.twist.linear.x = 1.0
        mock_msg.twist.linear.y = 2.0
        mock_msg.twist.linear.z = 3.0

        ship_data_logger.body_vel_callback(mock_msg)

        self.assertEqual(ship_data_logger.latest_data['vel_BDY_x'], 1.0)
        self.assertEqual(ship_data_logger.latest_data['vel_BDY_y'], 2.0)
        self.assertEqual(ship_data_logger.latest_data['vel_BDY_z'], 3.0)
        self.assertAlmostEqual(ship_data_logger.latest_data['vel_NED_x'], 1.0)
        self.assertAlmostEqual(ship_data_logger.latest_data['vel_NED_y'], 2.0)
        self.assertAlmostEqual(ship_data_logger.latest_data['vel_NED_z'], 3.0)
        self.assertAlmostEqual(ship_data_logger.latest_data['vel_GLB_ground_speed'], math.sqrt(1.0**2 + 2.0**2))
        self.assertAlmostEqual(ship_data_logger.latest_data['vel_GLB_course_over_ground'], math.atan2(2.0, 1.0))
        self.assertAlmostEqual(ship_data_logger.latest_data['vel_GLB_vertical_speed'], -3.0)

    @patch('ship_data_logger.np')
    @patch('ship_data_logger.tf_trans')
    def test_body_vel_callback_90_deg_yaw(self, mock_tf_trans, mock_np):
        # Body X of 1.0 should become NED Y of 1.0 with 90 deg positive yaw
        mock_np.array = lambda x: x

        # Rotation matrix for 90 deg yaw around Z: [[0,-1,0],[1,0,0],[0,0,1]]
        # quaternion_matrix by default returns 4x4, so we mock its [:3,:3] part
        # q_90_yaw = (0,0,math.sin(math.pi/4), math.cos(math.pi/4)) # x,y,z,w for pi/2 around Z
        # For yaw=pi/2, roll=0, pitch=0 using 'sxyz' for euler_from_quaternion
        # Roll=0 (X), Pitch=0 (Y'), Yaw=pi/2 (Z'')
        # Corresponds to rotation matrix: [[0, -1, 0], [1, 0, 0], [0, 0, 1]] (for standard sxyz interpretation)
        # This means X_body -> Y_ned, Y_body -> -X_ned

        # Let's define the rotation matrix directly for clarity in test
        R_90_yaw = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
        mock_tf_trans.quaternion_matrix.return_value = [[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0,0,0,1]] # Mock 4x4
        mock_tf_trans.quaternion_from_euler.return_value = (0,0,math.sin(math.pi/4), math.cos(math.pi/4)) # x,y,z,w

        # Mock numpy dot product: R_body_to_ned.dot(vel_body_vector)
        def mock_dot_product(R, v_body):
            # R is effectively R_body_to_ned[:3,:3]
            # v_body is [vx, vy, vz]
            # For R_90_yaw:
            # ned_x = R[0][0]*v_body[0] + R[0][1]*v_body[1] + R[0][2]*v_body[2] = 0*vx -1*vy + 0*vz = -vy
            # ned_y = R[1][0]*v_body[0] + R[1][1]*v_body[1] + R[1][2]*v_body[2] = 1*vx + 0*vy + 0*vz = vx
            # ned_z = R[2][0]*v_body[0] + R[2][1]*v_body[1] + R[2][2]*v_body[2] = 0*vx + 0*vy + 1*vz = vz
            if R[0][1] == -1: # Check if it's the 90 deg yaw matrix
                 return [-v_body[1], v_body[0], v_body[2]]
            return v_body # fallback to identity if matrix not recognized

        mock_np.dot = mock_dot_product


        ship_data_logger.latest_data['att_NED_roll'] = 0.0
        ship_data_logger.latest_data['att_NED_pitch'] = 0.0
        ship_data_logger.latest_data['att_NED_yaw'] = math.pi / 2.0 # 90 degrees

        mock_msg = TwistStamped()
        mock_msg.twist.linear.x = 1.0 # Moving along body X
        mock_msg.twist.linear.y = 0.0
        mock_msg.twist.linear.z = 0.0

        ship_data_logger.body_vel_callback(mock_msg)

        self.assertEqual(ship_data_logger.latest_data['vel_BDY_x'], 1.0)
        self.assertAlmostEqual(ship_data_logger.latest_data['vel_NED_x'], 0.0) # Was -vy_body, vy_body = 0
        self.assertAlmostEqual(ship_data_logger.latest_data['vel_NED_y'], 1.0) # Was vx_body
        self.assertAlmostEqual(ship_data_logger.latest_data['vel_NED_z'], 0.0)
        self.assertAlmostEqual(ship_data_logger.latest_data['vel_GLB_ground_speed'], 1.0)
        self.assertAlmostEqual(ship_data_logger.latest_data['vel_GLB_course_over_ground'], math.pi / 2.0)
        self.assertAlmostEqual(ship_data_logger.latest_data['vel_GLB_vertical_speed'], 0.0)

    @patch('ship_data_logger.rospy')
    @patch('ship_data_logger.open', new_callable=mock_open) # Mock open for CSV
    def test_data_logger_main_loop(self, mock_file_open, mock_rospy):
        # Configure rospy mocks
        mock_rospy.get_param.side_effect = lambda param, default: default # Return default for get_param
        mock_rospy.is_shutdown.side_effect = [False, False, True] # Run loop twice

        # Mock StringIO to capture CSV output
        mock_csv_io = io.StringIO()
        mock_file_open.return_value.__enter__.return_value = mock_csv_io

        # Pre-populate some data to be written
        ship_data_logger.latest_data['thruster_%'] = 0.5
        ship_data_logger.latest_data['pos_NED_x'] = 1.1
        ship_data_logger.latest_data['pos_GLB_lat'] = 12.34
        ship_data_logger.latest_data['att_NED_yaw'] = 0.1
        # ... add more for all fields defined in fieldnames if desired for full row test

        ship_data_logger.data_logger()

        # Check if subscribers were called (optional, basic check here)
        self.assertTrue(mock_rospy.Subscriber.called)

        # Check CSV output
        captured_csv = mock_csv_io.getvalue()
        # print(f"Captured CSV:\n{captured_csv}") # For debugging

        # Check header (ensure all fieldnames are present)
        header = ",".join(ship_data_logger.fieldnames) + "\r\n" # \r\n for windows-like newline from csv
        self.assertTrue(captured_csv.startswith(header))

        # Check number of data rows (should be 2 based on is_shutdown side_effect)
        lines = captured_csv.strip().split('\n')
        self.assertEqual(len(lines), 3) # 1 header + 2 data rows

        # Check content of one row (e.g., the first data row)
        # This requires knowing the exact order in fieldnames
        # Example: first data row, check a few values
        first_data_row_values = lines[1].split(',')
        # timestamp,rudder_angle,thruster_%,pos_NED_x,...
        # Find index of 'thruster_%' in fieldnames
        try:
            idx_thruster = ship_data_logger.fieldnames.index('thruster_%')
            self.assertEqual(float(first_data_row_values[idx_thruster]), 0.5)
            idx_pos_ned_x = ship_data_logger.fieldnames.index('pos_NED_x')
            self.assertEqual(float(first_data_row_values[idx_pos_ned_x]), 1.1)
        except ValueError:
            self.fail("Fieldname not found, check fieldnames list in main script and test.")


if __name__ == '__main__':
    unittest.main()
