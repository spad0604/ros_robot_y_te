"""Validate action wire messages offline; no ROS master or velocity publisher."""
import importlib.util
import math
from pathlib import Path
import sys
import time
import types
import unittest
from unittest.mock import Mock, patch

N = types.SimpleNamespace
class Stamp:
    def __init__(self, secs=0): self.secs = secs
    @staticmethod
    def now(): return Stamp(123)
class GoalID:
    def __init__(self, stamp=None, id=''): self.stamp, self.id = stamp, id
class ActionGoal:
    def __init__(self):
        self.header = N(stamp=None)
        self.goal_id = N(id='',stamp=None)
        self.goal = N(mode=None,timeout=None,target_pose=N(header=N(frame_id='',stamp=None),
            pose=N(position=N(x=0.,y=0.),orientation=N(z=0.,w=0.))))

fake_ros = N(get_param=lambda k, default=None: default, Time=Stamp,
             Publisher=Mock(), Subscriber=Mock())
mods = {'rospy':fake_ros, 'tf2_ros':N(), 'actionlib_msgs':N(),
    'actionlib_msgs.msg':N(GoalID=GoalID,GoalStatusArray=object),
    'nav_msgs':N(), 'nav_msgs.msg':N(OccupancyGrid=object),
    'std_msgs':N(),'std_msgs.msg':N(String=object), 'moving_server_msgs':N(),
    'moving_server_msgs.msg':N(MoveCommandActionGoal=ActionGoal,MoveCommandActionFeedback=object,
        MoveCommandActionResult=object,MovingServerState=object,MoveCommandGoal=N(MODE_MOVE_TO_POINT=0))}
spec = importlib.util.spec_from_file_location('delivery_wire_under_test', Path(__file__).parents[1]/'delivery_executor.py')
module = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, mods): spec.loader.exec_module(module)

class WireChecks(unittest.TestCase):
    def setUp(self):
        fake_ros.Publisher.reset_mock()
        fake_ros.Publisher.side_effect = lambda *a,**k: Mock(get_num_connections=lambda:1)
        fake_ros.Subscriber.side_effect = lambda *a,**k: Mock(get_num_connections=lambda:1)
        self.motion = module.Motion()
        self.motion.state = N(active=False,state_name='IDLE')
        self.motion.status_at = time.monotonic()
    def test_only_gateway_goal_and_cancel_are_published(self):
        self.assertEqual([c.args[0] for c in fake_ros.Publisher.call_args_list],
                         ['/moving_server/move_command/goal','/moving_server/move_command/cancel'])
        self.motion.send('delivery:unique',dict(dropoffX=1.25,dropoffY=-2.5,dropoffYaw=math.pi/2),300)
        msg = self.motion.goal_pub.publish.call_args.args[0]
        self.assertEqual(msg.goal_id.id,'delivery:unique')
        self.assertEqual(msg.goal.mode,0)
        self.assertEqual(msg.goal.target_pose.header.frame_id,'map')
        self.assertEqual((msg.goal.target_pose.pose.position.x,msg.goal.target_pose.pose.position.y),(1.25,-2.5))
        self.assertAlmostEqual(msg.goal.target_pose.pose.orientation.w,math.sqrt(.5))
    def test_cancel_has_specific_id_and_zero_stamp(self):
        self.motion.cancel('delivery:unique')
        msg = self.motion.cancel_pub.publish.call_args.args[0]
        self.assertEqual(msg.id,'delivery:unique')
        self.assertEqual(msg.stamp.secs,0)
    def test_busy_gateway_is_not_preempted_by_new_delivery(self):
        self.motion.state.active = True
        with self.assertRaises(RuntimeError): self.motion.send('new',{},300)
        self.motion.goal_pub.publish.assert_not_called()
    def test_rotated_map_origin_and_occupied_cells(self):
        env = module.Environment.__new__(module.Environment)
        env.grid = N(info=N(resolution=1.,width=2,height=2,
            origin=N(position=N(x=10.,y=20.),orientation=N(x=0.,y=0.,z=math.sqrt(.5),w=math.sqrt(.5)))),data=[0,0,0,0])
        self.assertIsNone(env.validate_target(dict(dropoffX=9.5,dropoffY=20.5)))
        env.grid.data[0] = 100
        self.assertEqual(env.validate_target(dict(dropoffX=9.5,dropoffY=20.5)), 'DESTINATION_UNKNOWN_OR_OCCUPIED')
        self.assertEqual(env.validate_target(dict(dropoffX=1000,dropoffY=2000)), 'DESTINATION_OUTSIDE_MAP')

if __name__ == '__main__': unittest.main()
