# Copied from PythonStepper_V7/python/run_function_demo.py with CRI import
import time
from cri.grippers.crap_hand_controller import CRAPHandController


def run_sequence(dev: CRAPHandController):
    print("enable(True)")
    dev.enable(True)

    persisted = dev.status().get('current_steps', None)
    print(f"current_steps={persisted}")

    new_pos = 0.5
    print(f"set_position({new_pos}, wait=True)")
    dev.set_position(new_pos, wait=True, timeout_s=20.0)

    # Example jog
    # print("jog(200)")
    # dev.jog(200)
    # time.sleep(1.0)


def main():
    dev = CRAPHandController(port="COM4", baud=115200)
    try:
        run_sequence(dev)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
