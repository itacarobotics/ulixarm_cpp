# ulixarm_cpp: code review questions

Review of commit `bc5c3dc` (all of `src/`, `include/`, `examples/`, `CMakeLists.txt`, and the joint limits in the URDF).

**How to answer:** write under each `Answer:` line. Short answers are fine, for example
`yes`, `no`, `skip`, `A`, `B`, or free text. If you leave an item empty I will **not** touch it.
Where I list options, **(rec)** marks the one I recommend.

Severity tags:
- 🔴 **Safety:** the arm can move or keep moving unexpectedly, or hurt someone.
- 🟠 **Bug:** wrong behaviour, misleading docs, or a crash path.
- 🟡 **Robustness:** works today but is fragile.
- 🔵 **Design / quality:** maintainability, API, build, performance.

> Note on "vulnerabilities": I found no memory-safety problems. All buffers are fixed-size,
> parsing indexes stay in bounds, and NaN/Inf are rejected before integer casts. The real risks
> in this library are **physical safety** issues, so section 1 covers those.

---

## 1. Safety

### S1 🔴 No motor-side watchdog: if the process dies, the motors keep running the last command forever
`~UlixArm()` deliberately leaves the motors enabled (`src/ulixarm.cpp:47-54`). If the program
crashes, gets killed (Ctrl+C / SIGINT), or hangs, each motor keeps executing its **last MIT command**.
In `examples/gravity.cpp` that command is `kp=0` plus a gravity torque computed for the *old* pose.
If the arm is then pushed, that torque is wrong for the new pose and the arm can drift or fall slowly.
Damiao motors have a CAN-timeout register that disables the motor (or makes it go limp) when no
frame arrives within N ms.

Should I:
- **A (rec)** add an API to write/verify the CAN-timeout register at `connect()`/`enable()`, with the value set by you
- B document it only (you configure the timeout with the Damiao tool)
- C leave as is

Answer: What do you mean limp? I prefere in case of connection loss, to keep last commanded position command, otherwise the arm would fall under gravity. I understand the risk if only torque is commanded. I will tackle that later.

**Status:** ⏸ **Open, kept for later.** No change. Safety notes added to the README.

### S2 🔴 A motor that reports "disabled" is treated as healthy
`check_motor_state()` accepts state `0x0` (disabled) as OK (`src/protocol.cpp:249-251`). Suppose the
library thinks the arm is `ENABLED` but one motor rebooted after a brown-out, lost power, or was
disabled externally. `control()` then keeps "commanding" it without error while that joint is limp.
Should a motor that reports `disabled` while `status == ENABLED` raise an error?

Answer: Yes, fix this.

**Status:** ✅ **Done.** `check_motor_state()` takes `expect_enabled`; a motor reporting disabled while the arm is `Enabled` throws. `enable()` refreshes right after, so it also verifies every motor came up.

### S3 🔴 Errors never put the arm in a safe state, and `ERROR` status is never set
When `control()` throws partway through the 6-motor loop (timeout, motor fault, bad frame),
joints 1..k have the new command and joints k+1..6 keep the old one. `status` stays `ENABLED`,
so the caller can keep calling `control()`. `UlixArmStatus::ERROR` and `RUNNING` are never assigned
anywhere. Also, once in `ERROR`, `disable()` and `connect()` refuse to run (`src/ulixarm.cpp:66, 178`),
so you could never shut the arm down from that state.

Proposed policy:
1. On a comms or motor fault inside `control()`/`open()`/`close()`: set `status = ERROR`, try a best-effort
   **safe reaction** (see S4), then rethrow.
2. In `ERROR`, allow only `disable()`, `clear_error()` / `reset()`, and reading state.
3. Remove `RUNNING` or give it a meaning.

Do you agree? What should happen to the arm on a fault?

Answer: I will see this later. Keep it an open point

**Status:** ⏸ **Open, kept for later.** No change (`Running`/`Error` kept in the enum).

### S4 🔴 No "safe stop" / damping mode
There is `hold()` (stiff hold at the current pose) and `disable()` (the arm falls). A common safe
reaction in between is **damping mode**: `kp=0`, a moderate `kd`, and gravity compensation, so the arm
sinks slowly instead of dropping or fighting. Should I add `damp()` / `safe_stop()` and use it as the
automatic fault reaction from S3?

Answer: see S3.

**Status:** ⏸ **Open, kept for later** (with S3).

### S5 🔴 `enable()` leaves the arm limp until the first `control()` call
After the enable frame a Damiao motor applies a zero MIT command, so the arm has no gravity
compensation and no stiffness. `enable()` then calls `refresh()` and returns. Nothing holds the arm
until the user calls `control()`. If the user's code throws or pauses here, the arm falls.
Should `enable()` immediately send a hold (or gravity-only) command right after enabling?

Answer: It is fine. The arm starts anyway from the rest position which is on the table. So it does not fall down under gravity.

**Status:** ✅ **No change** (as decided). Noted in the README safety notes.

### S6 🔴 `disable()` drops the arm without any safeguard
The docs warn about this, but the code doesn't help. Options:
- A require an explicit flag, e.g. `disable(/*i_know_it_will_fall=*/true)`
- **B (rec)** add `park()` / `lower_and_disable()`: damp down to a rest pose (or until velocity ≈ 0), then disable
- C leave as is

Answer: Keep this point for later.

**Status:** ⏸ **Open, kept for later.**

### S7 🔴 Total commanded torque is not bounded, only the feedforward part
`check_manipulator_joint_limits()` checks `|tau_ff + tau_gravity| <= effort limit`, `kp <= 500`, and
`kd <= 5`. The motor actually applies `kp*(q_t - q) + kd*(dq_t - dq) + tau_ff`. With `max_q_step = 0.2 rad`
and `kp = 500`, the spring term alone asks for 100 Nm (the motor saturates at 28 Nm). Should I also check
`|kp*(q_t-q)| + |kd*(dq_t-dq)| + |tau_ff|` against the effort limit (or a configurable fraction of it)?

Answer: No.

**Status:** ✅ **No change** (as decided).

### S8 🔴 No monitoring of measured state
Feedback is decoded but only temperature is checked. Nothing checks for measured velocity above a
threshold, a measured position outside the URDF limits, or a large tracking error. Should `control()`
check the **measured** `q`, `dq`, `tau` after each feedback and trigger S3/S4 on violation?
If yes, give the thresholds (or "use URDF limits").

Answer: no keep as is.

**Status:** ✅ **No change** (as decided).

### S9 🔴 Self-collision is not checked (`check_self_collision` is declared but not implemented)
`include/ulixarm_cpp/ulixarm.hpp:360` declares `check_self_collision()` and has unused
`pin_collision_model` / `pin_collision_data` members. The function has no definition, so calling it
would be a link error. The URDF already has `<collision>` meshes. Options:
- **A (rec)** implement it with Pinocchio + hpp-fcl (load `GeometryModel` from the URDF, `computeCollisions` on `q_target`); it costs some time per cycle, so make it optional
- B remove the declaration and members until later
- C leave as is

Answer: B remove declatations and members.

**Status:** ✅ **Done.** Removed `check_self_collision()`, the collision members and the collision/geometry includes.

### S10 🔴 Motor control mode is never verified
The library assumes every motor runs in **MIT mode**. A motor configured in position-velocity or
velocity mode reads the same 8 bytes differently and may move unexpectedly. Should
`connect()`/`enable()` read the control-mode register of each motor and refuse to enable if it's not MIT?

Answer: Yes. 

**Status:** ✅ **Done.** `connect()` reads `CTRL_MODE` (RID 10) of every motor and throws unless it is MIT (1). On failure the port is closed again.

### S11 🔴 The `PMAX/VMAX/TMAX` scaling is hard-coded and must match each motor's firmware
`MotorLimitParam{.q = 12.5, .dq = ..., .tau = ...}` (`src/ulixarm.cpp:14-23`) must match the values
flashed in each motor. If one doesn't match, every command and every reading for that joint is
mis-scaled, so torques are wrong and position checks are meaningless. Should I read these registers
from the motors at `connect()` and compare (or use them directly)?

Answer: Just compare and throw an error for now. Keep this point open.

**Status:** ⏸ **Partly done, kept open.** `connect()` reads PMAX/VMAX/TMAX (RID 21-23) and throws if they differ from the hard-coded values (0.1 % tolerance). Using the read values directly is still open.

### S12 🔴 The example has no error handling inside the control loop and no signal handling
In `examples/gravity.cpp`:
- any exception from `robot.control()`/`open()`/`close()` in the loop is uncaught, so `std::terminate` runs, the terminal stays in raw/non-blocking mode, and the motors keep the last command (see S1)
- Ctrl+C has the same effect
- the loop is unthrottled (busy loop; the rate is whatever the serial latency allows)

Should I rewrite the example with a try/catch inside the loop, a SIGINT handler that sets `running=false`,
a terminal restore guard (RAII), a fixed loop rate, and a safe reaction on error?

Answer: remove this example completely.

**Status:** ✅ **Done.** `examples/gravity.cpp` and its CMake target removed.

### S13 🟠 `gripper=false` still uses the gripper's mass in gravity compensation
The doc says that when `gripper == false`, gripper "gravity compensation and motors are ignored"
(`ulixarm.hpp:376`), but the same URDF with the gripper is always loaded, so gravity compensation
over-compensates. There is also no way to add a **payload** mass (a grasped object). Options:
- A load a separate URDF without the gripper when `gripper=false`
- **B (rec)** remove the gripper subtree from the Pinocchio model at runtime, and add `set_payload(mass, com)`
- C only fix the doc

Answer: Keep this point for later.

**Status:** ⏸ **Open, kept for later.**

### S14 🟡 Stale gripper feedback is used in the temperature check
`check_motor_temperatures()` includes the gripper motor, but its temperature is only updated on
`open()`/`close()`/`refresh()`. It can be minutes old. The opposite problem also exists: there is no getter
for the gripper position, velocity, torque or temperature. Should I (a) refresh the gripper periodically
or exclude it from the check, and (b) add `get_gripper_state()`?

Answer: Keep this point for later.

**Status:** ⏸ **Open, kept for later.**

### S15 🟡 `close()` applies a constant torque forever
`close(f)` sends `tau = 0.2..1.2 Nm`, `kp = 0` with no end condition. Holding 1.2 Nm continuously can
heat the DM4310, and if nothing is grasped the fingers drive into their hard stop. Should close
lower the torque to a holding value once the gripper stops moving, or is the current behaviour intended?

Answer: It is intended. Nominal torque is 3 Nm, so nothing bad shoud happen.

**Status:** ✅ **No change** (intended).

### S16 🟡 `refresh()` requires `ENABLED`
You can't read the arm's pose while it is only `CONNECTED`/`DISABLED`, for example to check that it's
in a safe pose **before** enabling, or to log state. Damiao motors answer the refresh query while
disabled. Should `refresh()` be allowed in `CONNECTED`/`DISABLED`?

Answer: Yes

**Status:** ✅ **Done.** `refresh()` works in `Connected`, `Disabled` and `Enabled`.

### S17 🟡 Temperature limit and gain/limit constants
`max_motor_temperature = 90 °C`, `max_q_step = 0.2 rad`, `max_motors_update_age = 100 ms`, and the hold
gains in `hold()` are all hard-coded `const`s. Should they be configurable (constructor config struct /
setters)? Are the values right? Notes: 90 °C is quite high for the rotor of a DM4310, and a 100 ms staleness
limit means a control loop slower than 10 Hz fails on every call when gravity compensation is on.

Answer: Ok so keep hardcoded, so the user cannot edit them. Make 100 ms to 500 ms. And temperature throws a warning already at 70 C.

**Status:** ✅ **Done.** Still hard-coded `const` members, so users cannot change them. Staleness 500 ms. A warning is printed once above 70 C (printed again after all motors cool down); error stays at 90 C.

---

## 2. Bugs and inconsistencies

### B1 🟠 Wrong error message in `disable_motors()`
`src/ulixarm.cpp:163` says `"Failed to enable motor: "` inside `disable_motors`. Fix it?

Answer: fix

**Status:** ✅ **Done.** Message fixed in `disable_motors()`.

### B2 🟠 `SerialPort::read()` doc doesn't match the code
The doc (`serial_port.hpp:128-133`) says *"On a tail mismatch, the search restarts from the next header byte
inside the frame"*, but the code just throws. Should I (a) implement the resync (rec, makes reads more robust
to noise) or (b) fix the comment?

Answer: fix the comment

**Status:** ✅ **Done.** Comment fixed (throws on tail mismatch).

### B3 🟠 Designated initializers are C++20, but the project builds as C++17
`MotorLimitParam{.q = 12.5, ...}` compiles as a GCC/Clang extension, warns with `-pedantic`, and fails
on MSVC in C++17 mode. Should I (a) switch to `CMAKE_CXX_STANDARD 20` or (b) use positional init / a constructor?

Answer: a

**Status:** ✅ **Done.** `CMAKE_CXX_STANDARD 20`, extensions off.

### B4 🟠 Gravity compensation assumes Pinocchio joint order == motor order
`compute_tau_gravity()` writes `q_full[i] = q[i]` and reads `tau_full[i]` by position. It only works
because joint1..6 happen to be the first revolute joints in the URDF. The limit check, on the other hand,
looks joints up by name (`getJointId` + `idx_q`). Should I make gravity compensation look up joints by name as well (and cache the indices in
the constructor)?

Answer: yes use joint names

**Status:** ✅ **Done.** Joint `idx_q`/`idx_v` looked up by name and cached in the constructor (throws there if a joint is missing). Gravity and limit checks use them.

### B5 🟠 The URDF error message is misleading
Any failure in `buildModel` (parse error, missing mesh, etc.) is reported as "Urdf file not found".
Change it to "Failed to load URDF '<path>'" and keep the nested cause?

Answer: fix

**Status:** ✅ **Done.** `"Failed to load URDF '<path>'."` with the nested cause.

### B6 🟡 The feedback's motor ID isn't cross-checked
`read_feedback()` matches only on the bridge's `master_id`. Payload byte 0 also has the slave ID in its
low nibble. Should I verify both?

Answer: yes

**Status:** ✅ **Done.** Feedback is matched on master id, and the slave id nibble in payload byte 0 must match too.

### B7 🟡 Off-by-one and units nits
- The head-scan loop allows `max_iterations + 2` bytes, not `max_iterations` (`serial_port.hpp:152-163`)
- `// baudrate [byte/s]` should be bit/s (`ulixarm.cpp:78`)
- `kp_uint`/`kd_uint` ranges `500`/`5` are duplicated in `protocol.cpp` and `ulixarm.cpp`

Fix all of these?

Answer: yes

**Status:** ✅ **Done.** Head scan reads at most `max_scan_factor * len` bytes; baudrate comment says bit/s; kp/kd ranges are `DamiaoProtocol::kp_max`/`kd_max`.

---

## 3. Design and API

### D1 🔵 Global namespace pollution
`#define N_JOINTS 6`, `typedef ... Joints`, unscoped `enum UlixArmStatus { IDLE, ..., ERROR }`
(`ERROR` clashes with `<windows.h>` and some logging libraries), plus generic class names
`Motor`, `SerialPort`, `Payload` in the global namespace. Should I:
- move everything into `namespace ulixarm`
- use `enum class Status`
- use `constexpr std::size_t kNumJoints = 6;` and `using Joints = ...`

Answer: ok

**Status:** ✅ **Done.** Everything in `namespace ulixarm`; `constexpr std::size_t kNumJoints = 6`, `using Joints`, `enum class Status`.

### D2 🔵 Include layout and install
CMake exposes `include/ulixarm_cpp` as the include root, so users write `#include "ulixarm.hpp"` and
`"protocol.hpp"`/`"motor.hpp"`, which are very collision-prone names. Should I expose `include/` so users write
`#include <ulixarm_cpp/ulixarm.hpp>`, and add `install()` + `ulixarm_cppConfig.cmake` so other projects can
`find_package(ulixarm_cpp)`?

Answer: yes

**Status:** ✅ **Done.** Include root is `include/` (`#include <ulixarm_cpp/ulixarm.hpp>`). `install()` + exported `ulixarm_cpp::ulixarm_core` + `ulixarm_cppConfig.cmake`, so `find_package(ulixarm_cpp)` works (tested).

### D3 🔵 The URDF path is baked in at compile time as an absolute source path
`ULIXARM_DESCRIPTION_PATH="${CMAKE_CURRENT_SOURCE_DIR}/..."` breaks as soon as the library is installed,
moved, or packaged. Options: install the description and resolve it relative to the install prefix, use an env var
(`ULIXARM_DESCRIPTION_PATH`), or require `urdf_path`. Preference?

Answer: I don't want it to be a source of problems with usage. Consider also that it will be wrapped to python pybind11. What would be best?

**Status:** ✅ **Done.** Lookup order: `urdf_path` argument → `$ULIXARM_DESCRIPTION_PATH` → installed copy (`share/ulixarm_cpp/ulixarm_description/urdf`, installed by `cmake --install`) → source-tree submodule. For pybind11: ship the URDF in the Python package and pass `urdf_path` (e.g. from `importlib.resources`), so the wheel never depends on build paths. Only `urdf/` is installed (meshes are not loaded).

### D4 🔵 Large headers are pulled into the public API
`ulixarm.hpp` includes 8 Pinocchio headers, `<iostream>`, `<iomanip>`, and `serial_port.hpp` (so
`libserialport.h` too). Every user pays the compile time and gets these transitive dependencies. Should I hide them behind
a pImpl (or at least forward-declare and move includes into the `.cpp`)?

Answer: Fix

**Status:** ↩️ **Reverted on request.** A pImpl was implemented, then dropped to keep the simple class layout of `main`. The header still includes only the Pinocchio model/data headers (algorithms, parser and collision headers moved to the `.cpp`), but Pinocchio and libserialport remain public dependencies.

### D5 🔵 Pass `Joints` by `const&`, and make getters `const`
`control(Joints, Joints, ...)`, `check_*`, and `compute_tau_gravity` take 48-byte Eigen fixed-size vectors by value
(this is also on Eigen's "don't pass fixed-size vectorizable types by value" list). The getters `get_q()` etc. aren't `const`.
The five getters copy-paste the same staleness check. Clean up?

Answer: fix

**Status:** ✅ **Done.** All `Joints` params are `const&`, getters are `const`, the staleness check is shared.

### D6 🔵 Stringly-typed `open(double d, std::string type)`
`"metric"` / `"normal"` typos only fail at runtime. Replace with `open_width(double meters)` and `open_ratio(double r)`
(or an `enum class OpeningUnit`)?

Answer: use the two different function names.

**Status:** ✅ **Done.** `open(d, type)` replaced by `open_width(meters)` and `open_ratio(ratio)`; `open()` kept (80 %).

### D7 🔵 Dead code and leftovers
- `return;` right after `std::throw_with_nested` (`ulixarm.cpp:220, 798, 852`), plus trailing `return;` in void functions
- `#include <iostream>` in `protocol.hpp`, unused `<termios>`-style includes in places, empty ctor/dtor in `DamiaoProtocol`/`Motor`
- `Motor` stores `joint_name` but returns it by value each call

Clean up? 

Answer: yes

**Status:** ✅ **Done.** Dead `return;`s, empty ctors/dtors and `<iostream>` in `protocol.hpp` removed; `get_joint_name()` returns `const&`.

### D8 🔵 Exceptions vs. error codes in the control path
Every control cycle allocates `std::string`s and builds nested exceptions on error, and every `control()` call checks
limits and may throw. That's fine for a non-realtime loop. If you plan to run this in a realtime thread
(e.g. a `ros2_control` hardware interface), the hot path should avoid allocation. Is realtime use a goal?

Answer: yes real time is important. Fix this.

**Status:** ✅ **Done (success path).** `control()` no longer allocates when nothing fails: cached joint indices, preallocated Pinocchio input, no string copies, frames on the stack. Errors are still exceptions (they allocate only when thrown, and map cleanly to pybind11); switching to error codes would be a separate API change.

---

## 4. Performance / communication

### P1 🔵 Sequential request→response per motor limits the loop rate
`control()` does flush → write → blocking read for each of the 6 motors one after another, so each cycle has
6 USB round-trips (≈1 ms each over USB-CDC). That puts the ceiling at roughly 100-150 Hz. Options:
- **A (rec)** write all 6 frames, then read the 6 replies and match them by `master_id`
- B use the bridge's multi-frame/batch command if it supports one
- C fine as is

What loop rate do you need?

Answer: Use A.

**Status:** ✅ **Done.** All frames of a group go out in one write (`send_frames()`), then the replies are read and matched by master id (`read_feedbacks()`). Used by `control()`, `refresh()` and the gripper.

### P2 🔵 Byte-by-byte reads
`wait_for_byte()` makes one `sp_blocking_read` syscall per byte (16 per frame, about 100 per cycle). Reading into a
buffer and scanning it would be cheaper. Do it together with P1?

Answer: I prefer wait_for_byte() for now. Keep as is.

**Status:** ✅ **No change** (as decided).

### P3 🔵 `flush_input()` before every write
This throws away any in-flight frame, including a late reply from the previous motor, which can then
make the next read time out instead of being matched. With P1 (match by ID) you only flush once per cycle.
OK to change?

Answer: Ok.

**Status:** ✅ **Done.** One flush per cycle (inside `send_frames()`).

---

## 5. Testing, tooling, docs

### T1 🔵 No tests
`DamiaoProtocol` is pure and easy to unit-test (encode/decode round-trip, bit packing, saturation, fault codes,
bridge frame parsing). `gripper_ik` and the limit checks can be tested without hardware. Should I add tests
(GoogleTest or Catch2? any preference) and a `ctest` target?

Answer: Leave out for now.

**Status:** ⏸ **Left out for now.**

### T2 🔵 A mock serial port / simulation mode
An abstract transport interface (`ITransport`) with a `SerialPort` implementation and a fake that simulates motors
would let you test `UlixArm` state handling and the fault reactions (S2-S8) without the arm. Worth it?

Answer: Leave out for now.

**Status:** ⏸ **Left out for now.**

### T3 🔵 Compiler warnings, sanitizers, CI
Add `-Wall -Wextra -Wpedantic` (and optionally `-Werror`), an ASan/UBSan build option, and a GitHub Actions
workflow that builds and runs the tests?

Answer: Leave out for now. But add the `-Wall -Wextra -Wpedantic -Werror`

**Status:** ✅ **Done.** `-Wall -Wextra -Wpedantic -Werror` on `ulixarm_core` (builds clean). Sanitizers/CI left out.

### T4 🔵 README
The README is one line. Should I write build instructions (deps: Eigen, Pinocchio, libserialport, the submodule),
a quickstart, the safety notes (S1, S5, S6), and serial-port permissions (`dialout` group / udev rule)?

Answer: yes

**Status:** ✅ **Done.** README: deps, build/install, `find_package`, quickstart, URDF lookup, safety notes, `dialout`/udev.

---

## 6. Anything else?

Priorities, constraints (e.g. "don't change the public API", "must stay C++17", "ROS 2 integration is next"),
or things I missed:

Answer: Add a disconnect method to close serial port.

**Status:** ✅ **Done.** `disconnect()` closes the serial port and returns to `Idle`. It does not disable the motors and is safe to call when not connected.
