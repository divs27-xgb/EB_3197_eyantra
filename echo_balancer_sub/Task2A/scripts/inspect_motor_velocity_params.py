"""Read-only query of CoppeliaSim joint velocity-controller parameters."""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require("sim")
joint = sim.getObject("/reference_frame/spherical_joint/bike_respondable/front_motor")
print("front_motor", joint)
for name in dir(sim):
    lower = name.lower()
    if "jointfloatparam" in lower and any(key in lower for key in ("pid", "velocity", "target", "max", "limit")):
        param = getattr(sim, name)
        if isinstance(param, (int, float)):
            try:
                value = sim.getObjectFloatParam(joint, param)
                print(f"{name} ({param}) = {value}")
            except Exception as exc:
                print(f"{name} ({param}) unavailable: {exc}")
for name in dir(sim):
    if "joint" in name.lower() and any(key in name.lower() for key in ("pid", "ctrl", "velocity")):
        value = getattr(sim, name)
        if isinstance(value, (int, float, str)):
            print("constant", name, "=", value)
