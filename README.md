# ulixarm_cpp

C++20 library to control the **UlixArm**: a 6-DoF arm driven by Damiao motors, with an optional
gripper, connected to the PC through a Damiao USB-CAN bridge.

It gives you:

- connection, enable/disable and motor configuration checks (MIT mode)
- per-joint impedance control (`kp`, `kd`, feedforward torque) at the bus rate
- gravity compensation from the URDF, computed with Pinocchio
- gripper control by width, ratio or grip strength
- safety checks on every command: joint limits, step size, motor temperatures, stale feedback

## Requirements

| Dependency | Notes |
|---|---|
| CMake ≥ 3.16, C++20 compiler | |
| Eigen ≥ 3.4 | |
| Pinocchio | gravity compensation |
| libserialport | `sudo apt install libserialport-dev` |
| `ulixarm_description` | git submodule, provides the URDF |

## Build

```bash
git clone --recurse-submodules https://github.com/itacarobotics/ulixarm_cpp.git
cd ulixarm_cpp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
sudo cmake --install build   # optional
```

Already cloned without submodules? Run `git submodule update --init`.

To use the installed library from your own CMake project:

```cmake
find_package(ulixarm_cpp REQUIRED)
target_link_libraries(my_app PRIVATE ulixarm_cpp::ulixarm_core)
```

## Usage

```cpp
#include <ulixarm_cpp/ulixarm.hpp>

int main()
{
    ulixarm::UlixArm arm(/*gripper=*/true, /*camera=*/false);

    try
    {
        arm.connect("/dev/ttyACM0");  // opens the port, checks motor config
        arm.enable();                 // the arm must be resting in its rest pose
        arm.hold();                   // hold the current pose

        const ulixarm::Joints kp = ulixarm::Joints::Constant(20.0);
        const ulixarm::Joints kd = ulixarm::Joints::Constant(1.0);
        const ulixarm::Joints zero = ulixarm::Joints::Zero();

        // control loop: each call sends one command and reads the feedback
        for (int i = 0; i < 1000; ++i)
            arm.control(arm.get_q(), zero, zero, kp, kd, /*gravity=*/true);

        arm.open_ratio(1.0);  // fully open the gripper
        arm.close(0.5);       // close at half strength

        arm.disconnect();     // motors stay enabled on the last command
    }
    catch (const std::exception& e)
    {
        arm.log_errors(e);    // prints the full chain of nested errors
        return 1;
    }
}
```

### API at a glance

| Method | What it does |
|---|---|
| `connect(port)` / `disconnect()` | open / close the serial link |
| `enable()` / `disable()` | power the motors on / off |
| `control(q, dq, tau, kp, kd, gravity)` | send one impedance command to the 6 joints |
| `hold()` | hold the current pose |
| `refresh()` | read fresh motor feedback without commanding |
| `get_q()`, `get_dq()`, `get_tau()` | joint position [rad], velocity [rad/s], torque [Nm] |
| `get_tmos()`, `get_trot()` | MOSFET and rotor temperatures [°C] |
| `open()`, `open_width(m)`, `open_ratio(r)` | open the gripper (default 80 %, 0–0.090 m, 0–1) |
| `close()`, `close(f)` | close the gripper (strength `f` in 0–1) |

Errors are reported as exceptions. The success path of `control()` does not allocate, so it can
run in a realtime loop.

### Example

`examples/gravity.cpp` runs pure gravity compensation, so you can move the arm by hand:

```bash
./build/examples_gravity /dev/ttyACM0
```

Press Ctrl+C to hold the current pose and quit.

## Safety

> [!WARNING]
> Read this before powering the arm.

- **Stopping the program does not stop the motors.** They keep running the last command, so the arm
  doesn't fall. If that command was torque-only (`kp = 0`), the arm is not held in place.
- **`enable()` does not hold the arm.** Until the first `control()` or `hold()` the motors apply a
  zero command. Enable only when the arm is resting.
- **`disable()` makes the arm fall.** Move it to a safe pose first.
- **Self collision is not checked.**
- `control()` rejects a command if it:
  - exceeds the joint limits (URDF limits + 0.1 rad margin)
  - moves a joint with `kp > 0` more than **0.2 rad** from its current position (interpolate large motions)
  - finds a motor above **90 °C** (warning from 70 °C)
- The getters (`get_q()`, ...) throw if the motor data is older than 500 ms.

## URDF lookup

The constructor picks the URDF matching the `gripper` and `camera` flags:

| `gripper` | `camera` | URDF |
|-----------|----------|------|
| `false` | `false` | `arm.urdf` |
| `true`  | `false` | `arm_gripper.urdf` |
| `true`  | `true`  | `arm_gripper_camera.urdf` |

`gripper=false, camera=true` throws `std::invalid_argument` (the camera is mounted on the gripper).

The file is loaded from the first location that exists:

1. the `urdf_path` constructor argument
2. `$ULIXARM_DESCRIPTION_PATH/urdf/<file>`
3. `<install prefix>/share/ulixarm_cpp/ulixarm_description/urdf/<file>`
4. the `ulixarm_description` submodule in the source tree

Bindings (e.g. pybind11) that ship their own copy of the URDF should pass `urdf_path`.

## Serial port access (Linux)

Add your user to the `dialout` group, then log out and back in:

```bash
sudo usermod -aG dialout $USER
```

Or add a udev rule, e.g. `/etc/udev/rules.d/99-ulixarm.rules` (find the IDs with `lsusb`):

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="<vid>", ATTRS{idProduct}=="<pid>", MODE="0666", SYMLINK+="ulixarm"
```

and reload with `sudo udevadm control --reload && sudo udevadm trigger`.

## Project layout

```
include/ulixarm_cpp/   public headers (ulixarm.hpp is the main API)
src/                   library implementation and Damiao protocol
examples/              example programs
cmake/                 CMake package config
ulixarm_description/   URDF submodule
```
