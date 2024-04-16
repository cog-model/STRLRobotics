import unittest

import tf
import rospy
if __name__ == '__main__':
    rospy.init_node('test_primitives', log_level=rospy.INFO)

from skill_bot.primitives.categories import *
from skill_bot.primitives.entity import *
from skill_bot.primitives.entity_fabric import * 



class TestPrimitives(unittest.TestCase):
    def test_base_class(self):
        tf_bd = tf.TransformBroadcaster(queue_size=5)
        entt = Entity()
        self.assertRaises(AssertionError, entt.TransformToFrame, rospy.Time.now(), 'test_fame_1')
        
        send_tf = lambda x :  tf_bd.sendTransform([0, 0, 0], [0, 0, 0, 1], rospy.Time.now(), 'test_frame_1', 'test_frame_2')
        timer = rospy.Timer(rospy.Duration(0, 100), send_tf)
        entt = Entity(frame = 'test_frame_1')
        entt_cp = copy.deepcopy(entt)
        entt = entt.TransformToFrame(rospy.Time.now(), 'test_frame_2')
        self.assertEqual(entt.entity_type, entt_cp.entity_type)
        self.assertEqual(entt.entity_type_id, entt_cp.entity_type_id)
        self.assertEqual(entt.track_id, entt_cp.track_id)
        
        entt = Entity(frame = 'test_frame_1')
        entt.TransformToFrame(rospy.Time.now(), 'test_frame_2', True)
        self.assertNotEqual(entt_cp.frame, entt.frame)
        timer.shutdown()
    
    def test_fabric(self):
        from std_msgs.msg import Header
        from geometry_msgs.msg import PoseStamped, Pose,  Quaternion
        ps = PoseStamped(header = Header(frame_id = 'test_frame'), pose= Pose(orientation = Quaternion(0, 0, 0, 1)))
        a : Entity = default_fabric.create_box(ps)[0]    
        b : Entity = default_fabric.create_box(ps, 0, 0)[0]
        self.assertEqual(a.frame, b.frame)
        self.assertEqual(a.frame, 'test_frame')
        self.assertTrue(np.all(a.basis - b.basis) < 10e-6)
        
    def test_basis_correction(self):
        test_basis = [
            [0,  0, -1],
            [0,  1,  0],
            [1,  0,  0]
        ]
        from scipy.spatial.transform import Rotation
        r = Rotation.from_euler('xyz', [0, 0, 0]).as_matrix()
        test_basis = test_basis @ r
        a = Entity(basis=test_basis)
        a.correct_basis(True)
        print()
        print(np.round(test_basis,2))
        print(np.round(a.basis,2))
        

        

if __name__ == '__main__':
    unittest.main(verbosity=2)