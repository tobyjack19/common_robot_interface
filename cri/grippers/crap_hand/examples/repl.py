# Copied from PythonStepper_V7/python/repl.py with import adjusted for CRI integration
import argparse
import sys

from cri.grippers.crap_hand_controller import CRAPHandController


def run_repl(port: str, baud: int):
    dev = CRAPHandController(port=port, baud=baud)
    print(f"Opening {port} @ {baud} (persistent session). Type 'help' for commands.")
    try:
        print("Driver enabled by default on Arduino setup. You can use 'enable on/off'.")
        while True:
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            cmd = line.split()
            c0 = cmd[0].lower()
            try:
                if c0 in ("quit", "exit"):
                    break
                elif c0 == "help":
                    print("Commands: enable on|off, speed <sps>, accel <sps2>, limits <min> <max>, min_pos <min>, max_pos <max>, position <0..1> [--wait], jog <steps>, status")
                elif c0 == "enable" and len(cmd) >= 2:
                    dev.enable(cmd[1].lower() == "on")
                elif c0 == "speed" and len(cmd) >= 2:
                    dev.set_speed(float(cmd[1]))
                elif c0 == "accel" and len(cmd) >= 2:
                    dev.set_accel(float(cmd[1]))
                elif c0 == "limits" and len(cmd) >= 3:
                    dev.set_limits(int(cmd[1]), int(cmd[2]))
                elif c0 == "min_pos" and len(cmd) >= 2:
                    st = dev.status()
                    cur_max = st.get("max_pos") if st else None
                    if cur_max is None:
                        raise ValueError("Cannot read current max_pos; try 'status' first")
                    dev.set_limits(int(cmd[1]), int(cur_max))
                elif c0 == "max_pos" and len(cmd) >= 2:
                    st = dev.status()
                    cur_min = st.get("min_pos") if st else None
                    if cur_min is None:
                        raise ValueError("Cannot read current min_pos; try 'status' first")
                    dev.set_limits(int(cur_min), int(cmd[1]))
                elif c0 == "position" and len(cmd) >= 2:
                    wait = "--wait" in cmd
                    dev.set_position(float(cmd[1]), wait=wait)
                elif c0 == "jog" and len(cmd) >= 2:
                    dev.jog(int(cmd[1]))
                elif c0 == "status":
                    print(dev.status())
                else:
                    print("Unknown or incomplete command. Type 'help'.")
            except Exception as e:
                print(f"Error: {e}")
    finally:
        dev.close()
        print("Closed.")


def main(argv=None):
    ap = argparse.ArgumentParser(description="Persistent REPL for CRAP hand controller (keeps COM port open)")
    ap.add_argument("--port", "-p", default="COM4")
    ap.add_argument("--baud", "-b", type=int, default=115200)
    args = ap.parse_args(argv or sys.argv[1:])
    run_repl(args.port, args.baud)


if __name__ == "__main__":
    main()
