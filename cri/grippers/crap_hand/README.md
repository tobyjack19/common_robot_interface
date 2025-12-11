CRAP Hand Gripper (Arduino + Stepper)

This folder contains peripheral assets for the CRAP hand gripper:
- Firmware sketch for Arduino/ESP32 in firmware/StepperController.ino
- Example Python scripts in examples/ for quick manual control

The integrated CRI driver is `cri.grippers.crap_hand_controller.CRAPHandController`.
Use it in your application to control the gripper alongside a robot controller.

Quick start (Windows PowerShell):

```powershell
# ensure pyserial is installed in your environment
pip install pyserial

# List COM ports
Get-WmiObject Win32_SerialPort | Select-Object DeviceID, Description

# Example standalone usage of examples scripts
python examples\repl.py -p COM4
```
