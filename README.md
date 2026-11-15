# ulixarm_cpp

Self-contained C++ API for the UlixArm (6 Damiao motors + gripper, over a Damiao USB-CAN bridge).

## Dependencies

- CMake ≥ 3.16, a C++20 compiler
- Eigen ≥ 3.4
- Pinocchio
- libserialport (`sudo apt install libserialport-dev`)
- the `ulixarm_description` submodule (URDF)

## Build and install

```bash
git clone --recurse-submodules <repo-url>
cd ulixarm_cpp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
sudo cmake --install build        # optional, or set -DCMAKE_INSTALL_PREFIX
```

If you cloned without `--recurse-submodules`: `git submodule update --init`.

Use it from another CMake project:

```cmake
find_package(ulixarm_cpp REQUIRED)
target_link_libraries(my_app PRIVATE ulixarm_cpp::ulixarm_core)
```

## Quickstart

```cpp
#include <ulixarm_cpp/ulixarm.hpp>

int main()
{
    ulixarm::UlixArm arm(/*gripper=*/true);

    try
    {
        arm.connect("/dev/ttyACM0");   // also checks every motor is in MIT mode
        arm.enable();
        arm.hold();                    // hold the current pose with gravity compensation

        const ulixarm::Joints kp = ulixarm::Joints::Constant(20.0);
        const ulixarm::Joints kd = ulixarm::Joints::Constant(1.0);

        // call this in your control loop; large motions must be interpolated (max 0.2 rad per call)
        arm.control(arm.get_q(), ulixarm::Joints::Zero(), ulixarm::Joints::Zero(), kp, kd, /*gravity=*/true);

        arm.open_ratio(1.0);
        arm.close(0.5);

        arm.disconnect();
    }
    catch (const std::exception& e)
    {
        ulixarm::UlixArm::log_errors(e);
        return 1;
    }
}
```

### URDF lookup

The constructor looks for `robot.urdf` in this order:

1. the `urdf_path` argument
2. `$ULIXARM_DESCRIPTION_PATH/urdf/robot.urdf`
3. `<install prefix>/share/ulixarm_cpp/ulixarm_description/urdf/robot.urdf`
4. the submodule in the source tree

Bindings (e.g. pybind11) that ship the URDF inside their own package should pass `urdf_path`.

## Safety notes

- **The motors keep running the last command when the program stops.** Neither the destructor nor
  `disconnect()` disables the motors, so the arm doesn't fall. If the last command was torque-only
  (`kp = 0`), the arm is not held in place.
- **`enable()` does not hold the arm.** Until the first `control()` / `hold()` the motors apply a zero
  command. Enable the arm only when it is resting on its rest position.
- **`disable()` makes the arm fall.** Bring it to a safe pose first.
- Self collision is not checked.
- `control()` checks joint limits, a 0.2 rad maximum step for joints with `kp > 0`, and motor
  temperatures (warning from 70 C, error from 90 C).

## Serial port permissions

On Linux, add your user to the `dialout` group (then log out and back in):

```bash
sudo usermod -aG dialout $USER
```

or add a udev rule, for example `/etc/udev/rules.d/99-ulixarm.rules`:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="<vid>", ATTRS{idProduct}=="<pid>", MODE="0666", SYMLINK+="ulixarm"
```

(`lsusb` shows the bridge's vendor and product ID.) Then run `sudo udevadm control --reload && sudo udevadm trigger`.
