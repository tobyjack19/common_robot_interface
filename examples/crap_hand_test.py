from cri.grippers import CRAPHandController
import time


def main():
    # Adjust COM port as needed or pass 'auto' to try autodetect
    gripper = CRAPHandController(port="COM4", baud=115200)
    try:
        print(gripper.info)
        gripper.enable(True)
        # gripper.set_limits(0, 3000)
        # gripper.set_speed(3000)
        # gripper.set_accel(2000)

        # Read persisted position and show it
        current_pos = gripper.current_position()
        print(f"current_position={current_pos if current_pos is not None else 'unknown'}")

        # Move to new position and wait
        new_pos = 0.5
        print(f"grasp_position({new_pos})")
        gripper.set_position(new_pos, wait=True, timeout_s=30.0)

        # Read persisted position and show it
        current_pos = gripper.current_position()
        print(f"current_position={current_pos if current_pos is not None else 'unknown'}")

        print(f"jog({2000})")
        gripper.jog(2000, wait=True, timeout_s=30.0)

        # Read persisted position and show it
        current_pos = gripper.current_position()
        print(f"current_position={current_pos if current_pos is not None else 'unknown'}")

        # Move to new position and wait
        new_pos = 0
        print(f"grasp_position({new_pos})")
        gripper.set_position(new_pos, wait=True, timeout_s=30.0)

        # Read persisted position and show it
        current_pos = gripper.current_position()
        print(f"current_position={current_pos if current_pos is not None else 'unknown'}")

    finally:
        gripper.close()


if __name__ == "__main__":
    main()
