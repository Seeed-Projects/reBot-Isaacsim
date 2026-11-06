"""Experimental isolated postprocessing: restore URDF inertias and drives."""
from pathlib import Path
import importlib.util
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, Usd, UsdPhysics
root=Path(__file__).resolve().parents[3]
urdf=root/'urdf/RS/urdf/ReBot_Arm_RS.urdf'
new=root/'usd/ReBot_Arm_RS/ReBot_Arm_RS.usda'
old=root/'usd/RS-rebot-dev-arm/RS-rebot-dev-arm.usda'
script=root/'usd/RS-rebot-dev-arm/scripts/validate_physics_fidelity.py'
spec=importlib.util.spec_from_file_location('fidelity',script); mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
errs=[]; links,joints,_=mod.parse_urdf(urdf,errs)
if errs: raise RuntimeError(errs)
s=Usd.Stage.Open(str(new)); s.SetEditTarget(s.GetRootLayer()); source=Usd.Stage.Open(str(old))
oldj={p.GetName():p for p in source.Traverse() if p.IsA(UsdPhysics.Joint)}
for p in s.Traverse():
 name=p.GetName()
 # Match the validated existing asset: a hull fills the jaw opening and
 # produces false finger contact, so decompose the three gripper colliders.
 if p.HasAPI(UsdPhysics.CollisionAPI) and p.GetParent().GetName() in {"gripper_end", "gripper_left", "gripper_right"}:
  collision=UsdPhysics.MeshCollisionAPI(p)
  if collision: collision.CreateApproximationAttr().Set("convexDecomposition")
 if p.HasAPI(UsdPhysics.MassAPI) and name in links:
  t=links[name]['tensor']; values,vectors=np.linalg.eigh(t)
  if np.any(values<=0): raise RuntimeError(f'{name}: nonpositive inertia eigenvalues')
  if np.linalg.det(vectors)<0: vectors[:,0]*=-1
  x,y,z,w=Rotation.from_matrix(vectors).as_quat(); m=UsdPhysics.MassAPI(p)
  m.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*map(float,values)))
  m.CreatePrincipalAxesAttr().Set(Gf.Quatf(float(w),Gf.Vec3f(float(x),float(y),float(z))))
 if name in joints and p.IsA(UsdPhysics.Joint):
  ref=joints[name]; kind='angular' if ref['type']=='revolute' else 'linear'; oldname={'gripper_joint1':'joint_left','gripper_joint2':'joint_right'}.get(name,name)
  od=UsdPhysics.DriveAPI(oldj[oldname],kind); d=UsdPhysics.DriveAPI.Apply(p,kind)
  d.CreateTypeAttr().Set('force'); d.CreateStiffnessAttr().Set(od.GetStiffnessAttr().Get()); d.CreateDampingAttr().Set(od.GetDampingAttr().Get()); d.CreateMaxForceAttr().Set(ref['effort']); d.CreateTargetPositionAttr().Set(0.0)
s.GetRootLayer().Save(); print('Prepared experimental USD:',new)
