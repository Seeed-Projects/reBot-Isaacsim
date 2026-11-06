from collections import defaultdict
from pathlib import Path
import importlib.util
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdUtils
root=Path(__file__).resolve().parents[3]
urdf=root/'urdf/RS/urdf/ReBot_Arm_RS.urdf'
usd=root/'usd/ReBot_Arm_RS/ReBot_Arm_RS.usda'
script=root/'usd/RS-rebot-dev-arm/scripts/validate_physics_fidelity.py'
spec=importlib.util.spec_from_file_location('fidelity',script); mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
errors=[]; warnings=[]; metrics=defaultdict(float)
links,joints,frames=mod.parse_urdf(urdf,errors); stage=Usd.Stage.Open(str(usd))
if not stage: raise RuntimeError('USD could not be opened')
prims=list(stage.Traverse()); bodies={p.GetName():p for p in prims if p.HasAPI(UsdPhysics.MassAPI)}
joint_prims={p.GetName():p for p in prims if p.IsA(UsdPhysics.Joint)}
mod.check_usd_pose(stage,frames,bodies,errors,metrics,'generated')
for name,ref in links.items():
 p=bodies.get(name)
 if not p: errors.append(f'missing body {name}'); continue
 m=UsdPhysics.MassAPI(p); tensor=mod.usd_tensor(p,errors,name)
 me=abs(float(m.GetMassAttr().Get())-ref['mass']); ce=float(np.max(np.abs(np.asarray(m.GetCenterOfMassAttr().Get())-ref['com']))); ie=float(np.max(np.abs(tensor-ref['tensor']))) if tensor is not None else float('inf')
 metrics['max_mass_error_kg']=max(metrics['max_mass_error_kg'],me); metrics['max_com_error_m']=max(metrics['max_com_error_m'],ce); metrics['max_inertia_error_kgm2']=max(metrics['max_inertia_error_kgm2'],ie)
 if me>mod.MASS_ATOL or ce>mod.COM_ATOL or ie>mod.INERTIA_ATOL: errors.append(f'{name}: inertial mismatch {me} {ce} {ie}')
source={j.get('name'):j for j in ET.parse(urdf).getroot().findall('joint')}
for name,ref in joints.items():
 p=joint_prims.get(name)
 if not p: errors.append(f'missing joint {name}'); continue
 j=UsdPhysics.Joint(p); kind='angular' if ref['type']=='revolute' else 'linear'
 if not p.HasAPI(UsdPhysics.DriveAPI,kind): warnings.append(f'{name}: no {kind} drive')
 factor=180/math.pi if kind=='angular' else 1
 for field in ('lower','upper'):
  a=p.GetAttribute('physics:'+field+'Limit'); value=a.Get() if a else None; expected=ref[field]*factor
  if value is None or abs(float(value)-expected)>0.0002: errors.append(f'{name}: {field} limit {value} != {expected}')
 axis=source[name].find('axis'); expected=np.array([float(x) for x in axis.get('xyz').split()]); ax=p.GetAttribute('physics:axis').Get(); vec={'X':Gf.Vec3f(1,0,0),'Y':Gf.Vec3f(0,1,0),'Z':Gf.Vec3f(0,0,1)}[str(ax)]; actual=np.asarray(j.GetLocalRot1Attr().Get().Transform(vec))
 if np.max(np.abs(actual-expected))>1e-4: errors.append(f'{name}: joint axis {actual.tolist()} != {expected.tolist()}')
_,_,missing=UsdUtils.ComputeAllDependencies(str(usd))
if missing: errors.append('unresolved assets: '+repr(missing))
result={'static_pass':not errors,'drive_attributes_present':not warnings,'link_count':len(links),'joint_count':len(joints),'usd_mesh_count':sum(p.IsA(UsdGeom.Mesh) for p in prims),'metrics':dict(metrics),'errors':errors,'warnings':warnings}
print(json.dumps(result,indent=2))
raise SystemExit(1 if errors else 0)
