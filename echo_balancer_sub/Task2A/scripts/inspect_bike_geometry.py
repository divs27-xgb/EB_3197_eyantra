"""Read-only CoppeliaSim geometry inspector for Echo Balancer Task 2A.

Connects to an already open CoppeliaSim instance through the ZeroMQ Remote
API. It does not start/stop the simulation or change any scene properties.
Run it the same way you ran logger_lqr_v3.py.
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


ROOT = "/reference_frame/spherical_joint/bike_respondable"
OBJECT_PATHS = [
    "/Floor",
    "/Floor/box",
    "/reference_frame",
    "/reference_frame/spherical_joint",
    ROOT,
    ROOT + "/front_motor",
    ROOT + "/front_motor/omniwheel_sphere",
    ROOT + "/front_motor/omniwheel_sphere/roller",
    ROOT + "/front_motor/omniwheel_sphere/roller/roller_sphere",
    "/reference_frame/omniwheel_visual",
    ROOT + "/drive_motor",
    ROOT + "/drive_motor/drivewheel_sphere",
]


def fmt(values, digits=6):
    if isinstance(values, (list, tuple)):
        return "[" + ", ".join(fmt(v, digits) for v in values) + "]"
    try:
        return f"{float(values):.{digits}f}"
    except (TypeError, ValueError):
        return repr(values)


def object_path(sim, handle):
    """Build a path from parent aliases (compatible with older sim APIs)."""
    aliases = []
    current = handle
    while current != sim.handle_scene:
        aliases.append(sim.getObjectAlias(current, 1))
        parent = sim.getObjectParent(current)
        if parent < 0 or parent == current:
            break
        current = parent
    return "/" + "/".join(reversed(aliases))


def try_print(label, fn):
    try:
        value = fn()
        print(f"  {label}: {value}")
        return value
    except Exception as exc:
        print(f"  {label}: unavailable ({type(exc).__name__}: {exc})")
        return None


def main():
    client = RemoteAPIClient()
    sim = client.require("sim")
    world = sim.handle_world

    print("Connected to CoppeliaSim. This script is read-only.")
    print("Object axes are the local coordinate axes expressed in world frame.")
    print("For a revolute joint, local Z is normally the joint axis.")

    for path in OBJECT_PATHS:
        print(f"\n--- {path} ---")
        try:
            handle = sim.getObject(path)
        except Exception as exc:
            print(f"  NOT FOUND ({type(exc).__name__}: {exc})")
            continue

        print("  handle:", handle)
        try_print("alias", lambda: sim.getObjectAlias(handle, 1))
        pos = try_print("world position m", lambda: fmt(sim.getObjectPosition(handle, world)))
        try_print("world orientation Euler rad", lambda: fmt(sim.getObjectOrientation(handle, world)))
        matrix = try_print("world transform 3x4", lambda: fmt(sim.getObjectMatrix(handle, world)))
        if matrix is not None and len(matrix) == 12:
            z_axis = [matrix[2], matrix[6], matrix[10]]
            print("  local Z axis in world:", fmt(z_axis))

        # Shape-only properties. Visual-only meshes may report mass too;
        # static/respondable flags are printed separately to avoid counting them.
        try_print("shape mass kg", lambda: sim.getShapeMass(handle))
        inertia_result = try_print("shape inertia and inertia-frame transform", lambda: sim.getShapeInertia(handle))
        if inertia_result is not None:
            try:
                inertia, inertia_frame = inertia_result
                print("  inertia tensor packed values kg m^2:", fmt(inertia, 9))
                print("  inertia-frame transform:", fmt(inertia_frame, 9))
            except Exception:
                pass
        try_print("bounding box local m (API nested layout)", lambda: fmt(sim.getShapeBB(handle)))

        for key in ("shapeintparam_static", "shapeintparam_respondable"):
            param = getattr(sim, key, None)
            if param is not None:
                try_print(key, lambda p=param: sim.getObjectInt32Param(handle, p))

        try:
            joint_type = sim.getJointType(handle)
            print("  joint type:", joint_type)
            if joint_type == sim.joint_revolute:
                try_print("joint position rad", lambda: sim.getJointPosition(handle))
                try_print("joint target velocity rad/s", lambda: sim.getJointTargetVelocity(handle))
                try_print("joint actual velocity rad/s", lambda: sim.getJointVelocity(handle))
        except Exception:
            pass

    print("\n--- Dynamic shapes in scene tree ---")
    try:
        shapes = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
        for handle in shapes:
            alias = sim.getObjectAlias(handle, 1)
            path = object_path(sim, handle)
            print(f"{alias}: {path}")
    except Exception as exc:
        print(f"Could not list shape tree: {type(exc).__name__}: {exc}")

    print("\nSave/copy all output and send it back. It contains scene geometry only;")
    print("wheel-ground rolling direction still needs a visual/manual confirmation.")


if __name__ == "__main__":
    main()
