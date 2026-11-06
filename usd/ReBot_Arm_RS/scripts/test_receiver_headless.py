import json, os, socket, sys, time
from pathlib import Path
import numpy as np
os.environ['REBOT_ISAAC_HEADLESS'] = '1'
sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'reBotArm_Isaacsim'))
from isaacsim_joint_receiver import IsaacJointMirror
m=IsaacJointMirror(port=15005)
sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
try:
 m.setup_isaac_sim()
 wire=np.array([-0.1,-0.3,-0.4,0,0,0])
 p={'sequence':1,'timestamp':time.time(),'joint_positions':wire.tolist(),'gripper_position':0.02}
 sender.sendto(json.dumps(p).encode(),('127.0.0.1',15005))
 time.sleep(0.05)
 packet=m._recv_latest_packet(); assert packet is not None
 q,seq,grip=packet; print('wire',wire,'asset target',q,flush=True)
 np.testing.assert_allclose(q,m.wire_to_asset_sign * wire); assert (seq,grip)==(1,0.02)
 m.latest_q=q
 m.articulation.set_joint_positions(q,joint_indices=m.arm_joint_indices)
 m._apply_gripper_target(grip)
 for _ in range(10): m.world.step(render=True)
 observed=m.articulation.get_joint_positions(joint_indices=m.arm_joint_indices)
 fingers=m.articulation.get_joint_positions(joint_indices=m.gripper_joint_indices)
 print('asset observed',observed,'fingers',fingers,flush=True)
 np.testing.assert_allclose(observed,q,atol=0.03)
 np.testing.assert_allclose(fingers,[0.02,0.02],atol=0.005)
 # A legacy oversized request must clamp both fingers to 50 mm.
 m._apply_gripper_target(0.0715)
 for _ in range(10): m.world.step(render=True)
 limited=m.articulation.get_joint_positions(joint_indices=m.gripper_joint_indices)
 print('limited fingers',limited,flush=True)
 np.testing.assert_allclose(limited,[0.05,0.05],atol=0.005)
 print('RECEIVER_INTEGRATION_PASS',flush=True)
finally:
 sender.close(); m.shutdown()
