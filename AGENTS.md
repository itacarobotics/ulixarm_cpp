# AGENTS.md: using ulixarm_cpp

Notes for coding agents writing programs that drive the UlixArm with this library.
The library talks to **real hardware**. A wrong command moves a physical arm.

## Safety rules for agents

- **Never run code against the arm without the user's explicit go-ahead.** Building and
  compiling is fine; executing a program that calls `connect()` is not, unless asked.
- **Never call `disable()` unless the user asked for it.** The arm falls when disabled.
- **Ending the program does not stop the arm.** The destructor and `disconnect()` leave the
  motors enabled, executing the last command. Make the last command a `hold()` (stiff hold
  at the current pose), not a torque-only command (`kp = 0`).
- **`enable()` does not hold the arm.** Call `hold()` or `control()` right after it.
- Self collision is **not** checked. Keep targets inside a workspace the user confirmed.
- Do not edit the hard-coded limits in the library to make a command pass. Fix the command.

## Build and link

```cmake
find_package(ulixarm_cpp REQUIRED)
target_link_libraries(my_app PRIVATE ulixarm_cpp::ulixarm_core)
```

C++20. Dependencies: Eigen ≥ 3.4, Pinocchio, libserialport. See `README.md` for building
and installing the library itself, and for serial port permissions.

```cpp
#include <ulixarm_cpp/ulixarm.hpp>
```

Everything is in `namespace ulixarm`. `Joints` is `Eigen::Matrix<double, 6, 1>`, one entry
per joint (`joint1` .. `joint6`).

## Lifecycle

```
UlixArm arm(gripper, urdf_path)   // loads the URDF, no hardware access
arm.connect("/dev/ttyACM0")       // opens the port, checks every motor is in MIT mode
arm.enable()                      // enables the motors, then refresh()
arm.hold()                        // hold the current pose with gravity compensation
... control loop with arm.control(...) ...
arm.hold()                        // leave the arm in a stiff hold
arm.disconnect()                  // closes the port, motors stay enabled
```

| Call | Allowed when | Notes |
|------|--------------|-------|
| `connect(port)` | not connected | Throws and closes the port if a motor is misconfigured. |
| `disconnect()` | always | Closes the port. Does not disable the motors. |
| `enable()` / `disable()` | connected | `disable()` drops the arm. |
| `refresh()` | connected, enabled or disabled | Reads the state without commanding. |
| `control(...)` / `hold()` | enabled | Manipulator only. |
| `get_q/dq/tau/tmos/trot()` | after `refresh()` or `control()` | Throws if data is older than 500 ms. |
| `open()`, `open_width(m)`, `open_ratio(r)`, `close()`, `close(f)` | enabled, gripper mounted | Gripper only. |

## `control()`

```cpp
arm.control(q_target, dq_target, tau_ff, kp, kd, /*gravity=*/true);
```

Per joint the motor applies `kp * (q_target - q) + kd * (dq_target - dq) + tau_ff`
(+ gravity torque if `gravity` is true). Every call sends the command and reads back the
feedback, so the getters are fresh after it.

The command is rejected (exception, nothing sent) if any of these fails:

- `q_target` within the URDF position limits
- `|dq_target|` within min(URDF velocity limit, motor limit)
- `|tau_ff + gravity|` within min(URDF effort limit, motor limit)
- `kp` in [0, 500], `kd` in [0, 5]
- for joints with `kp > 0`: `|q_target - q_measured| <= 0.2 rad`
- no motor above 90 C (a warning is printed once above 70 C)
- motor data not older than 500 ms (needed when `gravity` is true or any `kp > 0`)

So:

- **Interpolate large motions.** Never send a far target; step toward it a little each cycle.
- **Keep the loop running.** If more than 500 ms pass between calls, the next call throws.
  Call `refresh()` first after a pause.
- NaN or Inf anywhere in a command is rejected.

### Typical loop

```cpp
#include <ulixarm_cpp/ulixarm.hpp>
#include <chrono>
#include <thread>

using ulixarm::Joints;

void move_to(ulixarm::UlixArm& arm, const Joints& q_goal)
{
    const Joints kp = (Joints() << 50, 100, 60, 30, 20, 10).finished();
    const Joints kd = (Joints() << 1.0, 2.0, 1.5, 0.8, 0.5, 0.2).finished();
    const double max_step = 0.002;                     // rad per cycle, 1 rad/s at 500 Hz
    const auto period = std::chrono::milliseconds(2);

    Joints q_cmd = arm.get_q();
    auto next = std::chrono::steady_clock::now();

    while ((q_goal - q_cmd).cwiseAbs().maxCoeff() > 1e-4)
    {
        q_cmd += (q_goal - q_cmd).cwiseMax(-max_step).cwiseMin(max_step);
        arm.control(q_cmd, Joints::Zero(), Joints::Zero(), kp, kd, /*gravity=*/true);

        next += period;
        std::this_thread::sleep_until(next);
    }
}
```

The step check compares against the **measured** position, so if the arm lags behind
`q_cmd` by more than 0.2 rad (blocked, gains too low), `control()` throws. Treat that as a
fault, not something to work around.

The success path of `control()` does not allocate, so it can run in a realtime thread.

## Gripper

- `open_width(meters)`: finger opening 0.0 to 0.090 m. `open_ratio(r)`: 0.0 to 1.0.
  `open()` opens to 80 %. Out-of-range values throw.
- `close(f)`: torque-controlled close, `f` in [0, 1] (0.2 to 1.2 Nm). It keeps pushing
  until the next gripper command; this is intended.
- Construct with `UlixArm(false)` if no gripper is mounted; gripper calls then throw.

## Errors

Every failure is an exception, usually with nested causes. Print the whole chain:

```cpp
try
{
    arm.control(...);
}
catch (const std::exception& e)
{
    arm.log_errors(e);   // prints the error and every nested cause
    // decide on a safe reaction; the arm keeps the last command that was accepted
}
```

There is no automatic fault reaction yet: after an exception in `control()`, some motors may
have received the new command and others not. Stop commanding motion and let the user decide.
Motor faults (overvoltage, overtemperature, ...) and a motor reporting disabled while the
arm is enabled are raised as exceptions from the call that read the feedback.

## URDF

The constructor looks for `robot.urdf` in this order: the `urdf_path` argument,
`$ULIXARM_DESCRIPTION_PATH/urdf/robot.urdf`, the installed copy, the source-tree submodule.
Pass `urdf_path` explicitly when packaging (e.g. Python bindings).
