#!/usr/bin/env python3
"""Run on ROS Noetic: isolated action protocol test, no HTTP or motor publishers."""
import os
import sys
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
import rospy
import actionlib
from actionlib_msgs.msg import GoalID
from moving_server_msgs.msg import MoveCommandAction, MoveCommandResult, MoveCommandFeedback, MovingServerState
from delivery_executor import Motion
from delivery_engine import DeliveryEngine

class Store:
    data = None
    def load(self): return None
    def save(self, data): self.data = data
class Api:
    def __init__(self, number):
        self.task = dict(id='test-'+str(number),robotCode='TEST-ROBOT',mapId='test-map',type='DELIVERY',
                         status='CREATED',dispatchEnabled=True,dropoffX=1.25,dropoffY=-2.5,
                         dropoffYaw=0,cancellationRequested=False)
    def active(self): return []
    def selection(self): return dict(status='APPLIED',mapId='test-map')
    def pending(self): return [dict(self.task)]
    def claim(self, task, execution): self.task.update(status='RUNNING',executionId=execution); return dict(self.task)
    def heartbeat(self, task, body): return dict(self.task)
    def result(self, task, body):
        self.task['status'] = 'CANCELLED' if self.task['cancellationRequested'] else {'SUCCEEDED':'COMPLETED','FAILED':'FAILED','CANCELLED':'CANCELLED'}[body['outcome']]
        return dict(self.task)
class Environment:
    def __init__(self, motion): self.motion = motion
    def ready(self): return self.motion.idle(), 'WAITING_FOR_MOCK_GATEWAY'
    def signature(self): return 'mock-map'
    def active_ok(self, signature): return True, ''
    def validate_target(self, task): return None

def main():
    rospy.init_node('delivery_wire_integration_test')
    action, state_topic = '/delivery_wire_test/move_command', '/delivery_wire_test/state'
    rospy.set_param('~action_name',action)
    rospy.set_param('~moving_state_topic',state_topic)
    pub = rospy.Publisher(state_topic,MovingServerState,queue_size=1,latch=True)
    cancelled, goals = [], []
    rospy.Subscriber(action+'/cancel',GoalID,lambda msg:cancelled.append(msg.id))
    def state(active):
        msg=MovingServerState();msg.active=active;msg.state_name='MOVE_BASE' if active else 'IDLE';pub.publish(msg)
    server = None
    def execute(goal):
        assert goal.mode == 0 and goal.target_pose.header.frame_id == 'map'
        assert goal.target_pose.pose.position.x == 1.25 and goal.target_pose.pose.position.y == -2.5
        goals.append(goal);state(True)
        result=MoveCommandResult()
        for _ in range(20):
            if server.is_preempt_requested():
                result.success=False;result.result_code=2;result.message='Mock cancelled'
                state(False);server.set_preempted(result);return
            feedback=MoveCommandFeedback();feedback.state_name='MOVE_BASE';feedback.distance_remaining=1.;feedback.progress=.5
            server.publish_feedback(feedback);time.sleep(.05)
        result.success=True;result.result_code=0;result.message='Mock arrived'
        result.final_pose=goal.target_pose
        state(False);server.set_succeeded(result)
    server=actionlib.SimpleActionServer(action,MoveCommandAction,execute_cb=execute,auto_start=False)
    server.start();state(False)
    motion=Motion()
    deadline=time.monotonic()+8
    while not motion.idle() and time.monotonic()<deadline:time.sleep(.1)
    assert motion.idle(),'Mock action gateway not ready'
    for number, expected in ((1,'COMPLETED'),(2,'CANCELLED')):
        api=Api(number);engine=DeliveryEngine('TEST-ROBOT',api,motion,Environment(motion),Store())
        motion.engine=engine
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            engine.step()
            if number==2 and len(goals)==2:api.task['cancellationRequested']=True
            if api.task['status'] in ('COMPLETED','FAILED','CANCELLED') and engine.job is None:break
            time.sleep(.1)
        assert api.task['status']==expected,(api.task,expected)
        if number==2:assert cancelled and cancelled[-1].startswith('delivery:TEST-ROBOT:test-2:')
        print('ROS_ACTION_E2E',expected,'PASS',flush=True)
    assert len(goals)==2
    print('Two isolated action goals; no velocity publisher or physical robot goal used.',flush=True)
    rospy.signal_shutdown('Integration checks complete')

if __name__=='__main__':main()
