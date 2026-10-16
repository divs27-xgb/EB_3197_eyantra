"""Passive Task 2A LQR logger.

Run from the Task2A Python environment while the LQR Tuner is running.
The tuner output u is the front motor target velocity, confirmed manually,
so `u_target_rad_s` is read directly from that joint.
"""

import csv
import os
import time
from datetime import datetime

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def main(sample_period=0.01):
    client = RemoteAPIClient()
    sim = client.require("sim")

    motor_path = "/reference_frame/spherical_joint/bike_respondable/front_motor"
    roller_path = "/reference_frame/spherical_joint/bike_respondable/front_motor/omniwheel_sphere/roller"
    rear_motor_path = "/reference_frame/spherical_joint/bike_respondable/drive_motor"
    body_path = "/reference_frame/spherical_joint/bike_respondable"
    reference_path = "/reference_frame"

    motor = sim.getObject(motor_path)
    roller = sim.getObject(roller_path)
    rear_motor = sim.getObject(rear_motor_path)
    body = sim.getObject(body_path)
    reference = sim.getObject(reference_path)

    print("Front motor:", sim.getObjectAlias(motor, 2), "handle:", motor)
    print("Front roller:", sim.getObjectAlias(roller, 2), "handle:", roller)
    print("Rear drive motor:", sim.getObjectAlias(rear_motor, 2), "handle:", rear_motor)
    print("Body:", sim.getObjectAlias(body, 2), "handle:", body)
    print("Waiting for simulation to run. Press Ctrl+C to stop logging.")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "local_data"))
    os.makedirs(output_dir, exist_ok=True)
    csv_path = os.path.join(output_dir, f"lqr_capture_{timestamp}.csv")

    with open(csv_path, "x", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow([
            "run_id",
            "reset_detected",
            "simulation_time_s",
            "sample_interval_s",
            "u_target_rad_s",
            "motor_actual_rad_s",
            "motor_target_force_Nm",
            "motor_applied_force_Nm",
            "front_roller_position_rad",
            "front_roller_actual_rad_s",
            "rear_motor_target_velocity_rad_s",
            "rear_motor_actual_rad_s",
            "rear_motor_target_force_Nm",
            "rear_motor_applied_force_Nm",
            "body_euler_x_rad",
            "body_euler_y_rad",
            "body_euler_z_rad",
            "body_angular_velocity_x_world_rad_s",
            "body_angular_velocity_y_world_rad_s",
            "body_angular_velocity_z_world_rad_s",
        ])
        file.flush()

        try:
            while sim.getSimulationState() == sim.simulation_stopped:
                time.sleep(0.05)

            previous_time = None
            run_id = 1
            while sim.getSimulationState() != sim.simulation_stopped:
                sim_time = float(sim.getSimulationTime())
                reset_detected = False
                # The tuner can restart the simulation immediately after a fall.
                # Keep one CSV, but mark the first sample of every new episode.
                if previous_time is not None and sim_time < previous_time:
                    run_id += 1
                    reset_detected = True
                    print(f"Simulation reset detected; logging episode {run_id}.")
                if previous_time is not None and sim_time == previous_time:
                    time.sleep(sample_period)
                    continue

                sample_interval = (
                    0.0 if previous_time is None or reset_detected
                    else sim_time - previous_time
                )
                previous_time = sim_time

                euler = sim.getObjectOrientation(body, reference)
                _, angular_velocity = sim.getObjectVelocity(body)
                target_velocity = float(sim.getJointTargetVelocity(motor))
                actual_velocity = float(sim.getJointVelocity(motor))
                target_force = float(sim.getJointTargetForce(motor))
                applied_force = float(sim.getJointForce(motor))
                roller_position = float(sim.getJointPosition(roller))
                roller_velocity = float(sim.getJointVelocity(roller))
                rear_target_velocity = float(sim.getJointTargetVelocity(rear_motor))
                rear_actual_velocity = float(sim.getJointVelocity(rear_motor))
                rear_target_force = float(sim.getJointTargetForce(rear_motor))
                rear_applied_force = float(sim.getJointForce(rear_motor))

                writer.writerow([
                    run_id,
                    int(reset_detected),
                    f"{sim_time:.6f}",
                    f"{sample_interval:.6f}",
                    f"{target_velocity:.9g}",
                    f"{actual_velocity:.9g}",
                    f"{target_force:.9g}",
                    f"{applied_force:.9g}",
                    f"{roller_position:.9g}",
                    f"{roller_velocity:.9g}",
                    f"{rear_target_velocity:.9g}",
                    f"{rear_actual_velocity:.9g}",
                    f"{rear_target_force:.9g}",
                    f"{rear_applied_force:.9g}",
                    *(f"{float(v):.9g}" for v in euler),
                    *(f"{float(v):.9g}" for v in angular_velocity),
                ])
                # Flush frequently so a tuner stop or logger interruption keeps the data.
                file.flush()
                time.sleep(sample_period)
        except KeyboardInterrupt:
            print("Logging interrupted by user.")

    print("CSV saved:", csv_path)


if __name__ == "__main__":
    main()


