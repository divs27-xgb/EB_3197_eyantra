"""Read-only joint-mode/axis check for the Task 2A wheel assembly.

Run with the Task 2A scene open in CoppeliaSim. It does not start the
simulation or send joint commands.
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


JOINTS = {
    "front_motor": "/reference_frame/spherical_joint/bike_respondable/front_motor",
    "front_roller": "/reference_frame/spherical_joint/bike_respondable/front_motor/omniwheel_sphere/roller",
    "rear_drive_motor": "/reference_frame/spherical_joint/bike_respondable/drive_motor",
}


def vector(values):
    return "[" + ", ".join(f"{float(v): .6f}" for v in values) + "]"


def names_for(sim, value, prefix):
    names = []
    for name in dir(sim):
        if name.startswith(prefix) and getattr(sim, name, None) == value:
            names.append(name)
    return ", ".join(names) if names else "(no matching constant exposed)"


def main():
    client = RemoteAPIClient()
    sim = client.require("sim")
    world = sim.handle_world

    for label, path in JOINTS.items():
        print(f"\n--- {label}: {path} ---")
        try:
            joint = sim.getObject(path)
        except Exception as exc:
            print(f"NOT FOUND: {type(exc).__name__}: {exc}")
            continue

        print("handle:", joint)
        try:
            print("joint type:", sim.getJointType(joint))
        except Exception as exc:
            print("joint type unavailable:", exc)

        try:
            mode_info = sim.getJointMode(joint)
            mode_code = mode_info[0] if isinstance(mode_info, (tuple, list)) else mode_info
            print("operation mode:", mode_info, "=", names_for(sim, mode_code, "jointmode_"))
        except Exception as exc:
            print("operation mode unavailable:", exc)

        try:
            ctrl_param = sim.jointintparam_dynctrlmode
            ctrl = sim.getObjectInt32Param(joint, ctrl_param)
            print("dynamic control mode:", ctrl, "=", names_for(sim, ctrl, "jointdynctrl_"))
        except Exception as exc:
            print("dynamic control mode unavailable:", exc)

        try:
            matrix = sim.getObjectMatrix(joint, world)
            axis = [matrix[2], matrix[6], matrix[10]]
            print("local Z axis in world:", vector(axis))
        except Exception as exc:
            print("world axis unavailable:", exc)

        for label2, getter in (
            ("position rad", sim.getJointPosition),
            ("target velocity rad/s", sim.getJointTargetVelocity),
            ("actual velocity rad/s", sim.getJointVelocity),
            ("target force/torque", sim.getJointTargetForce),
            ("reported force/torque", sim.getJointForce),
        ):
            try:
                print(f"{label2}:", getter(joint))
            except Exception as exc:
                print(f"{label2} unavailable:", exc)


if __name__ == "__main__":
    main()
