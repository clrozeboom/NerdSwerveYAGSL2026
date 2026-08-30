# NerdSwerve 2027 — Custom Swerve

Team 5010's swerve drivetrain, written from scratch against **WPILib 2027.0.0-alpha-6** using the
**Commands v2** framework and **AdvantageKit** for logging and deterministic replay.

This replaces the YAGSL-based 2026 project. Every drivetrain number here — CAN IDs, gear ratios,
encoder offsets, current limits, module positions, feedforward gains — was carried over from that
project's `deploy/swerve/neo` configuration and its `Constants.java`, so this describes the same
physical robot.

---

## Why alpha-6 and not alpha-7

alpha-7 is the newest WPILib 2027 alpha, but alpha-6 is the better place to build right now:

- **Every published 2027 vendordep targets `2027_alpha5`.** PathPlannerLib, Phoenix6, REVLib,
  ReduxLib, ThriftyLib and PhotonLib all declare that. Checking their jars' class references against
  each alpha, the classes they need — `SendableChooser`, `SmartDashboard`, `Sendable`, `Alert`,
  `Pair`, `AprilTagFieldLayout`, `EpilogueBackend` — all still exist in alpha-6. Several are **gone**
  in alpha-7.
- **alpha-7 deleted the whole `Sendable`/`SmartDashboard` world** and replaced it with new
  `org.wpilib.telemetry` and `org.wpilib.tunable` modules. Building on alpha-6 means the dashboard
  code here is ordinary and portable rather than written against modules that did not exist a
  release ago.

The cost is that alpha-6's WPILib jars are **not** on the public frcmaven mirror — only alpha-7 is.
They come from the local WPILib 2027 alpha-6 installer, which `settings.gradle` points at via
`wpilibYear = '2027_alpha5'` (alpha-6 keeps the `alpha5` directory name). Everyone building this
needs that installer; it also supplies the JDK 25 the build requires.

### Migrating to alpha-7 later

The changes this project would need, all mechanical:

| alpha-6 | alpha-7 |
| --- | --- |
| `Rotation2d.kZero`, `Pose2d.kZero`, `Translation2d.kZero` | `.ZERO` (constants renamed to all-caps) |
| `Translation2d.getAngle()` returns `Rotation2d` | returns `Optional<Rotation2d>` |
| `org.wpilib.smartdashboard.SmartDashboard` | removed → `org.wpilib.telemetry.Telemetry` |
| `org.wpilib.smartdashboard.SendableChooser` | removed → `org.wpilib.tunable.Selectable` + `Tunables.publish` |
| `org.wpilib.smartdashboard.Field2d` | removed along with `Sendable` |
| `org.wpilib.driverstation.Alert` | `org.wpilib.util.Alert` |
| `org.wpilib.math.util.Pair` | `org.wpilib.util.Pair` |

`SmartDashboard` is only touched in two places now that AdvantageKit's `Logger` carries the
telemetry: the `Field2d` publish in `Drive` and the auto chooser in `RobotContainer`.

---

## Structure

```
subsystems/drive/
  Drive.java           subsystem: kinematics, odometry, control
  Module.java          per-module logic: units, optimization, telemetry
  ModuleIO.java        hardware interface + ModuleIOInputs struct
  ModuleIOSim.java       physics sim
  ModuleIOSpark.java     SPARK MAX + NEO + Thrifty analog encoder
  GyroIO.java          gyro interface + GyroIOInputs struct
  GyroIONone.java        no gyro (the default — none planned)
  GyroIOOnboard.java     SystemCore's built-in IMU
commands/DriveCommands.java   command factories
```

Every loop reads all hardware into the `*Inputs` structs first, then runs control off that one
snapshot, and `Logger.processInputs` is what makes that snapshot replayable — writing it to the log
when running, and substituting the logged values when replaying.

### Replay

AdvantageKit 27.0.0-alpha-4 is a dependency, and **replay has been verified working end to end** on
this project: a simulation run wrote a 245 KB `.wpilog`, that log was replayed back through the same
control code, and the resulting log carries a `ReplayOutputs` table recomputed from the logged
inputs alongside the original `RealOutputs`.

To replay a log:

1. Copy a `.wpilog` off the robot (or set `LOG_IN_SIM = true` in `Robot.java` to capture one from a
   sim run).
2. Set `REPLAY_LOG` in `Robot.java` to its path.
3. Run the simulator. Every `updateInputs` call is fed from the log instead of from hardware, the
   control code re-executes against exactly the inputs it saw, and a `*_replay.wpilog` lands beside
   the original.
4. Open both in AdvantageScope and compare `RealOutputs` against `ReplayOutputs`.

Two things AdvantageKit needs that the vendordep does not bring on its own, both already wired up
here: the `@AutoLog` annotation processor is a separate artifact
(`org.littletonrobotics.akit:akit-autolog`, added explicitly in `build.gradle` with its version
pinned to the vendordep's), and `Robot` must extend `LoggedRobot` rather than `TimedRobot` so the
loop is driven by the log during replay instead of by the system clock.

---

## No gyro — and why the default is robot-relative

There is no gyro on this robot and none planned, so **robot-relative driving is the default**.

`RobotContainer` constructs `GyroIONone`. `Drive` handles that by integrating module positions to
track heading, which works but drifts, and a drifting heading makes field-relative driving
progressively wrong in a way that is confusing to drive. Robot-relative ignores heading entirely, so
it stays correct indefinitely. Field-relative is still there on the left bumper, and becomes the
sensible default the moment a gyro is fitted.

Heading is still tracked and logged, and the pose estimate still runs — they are just not trusted
for driving.

If a gyro is ever wanted, **`GyroIOOnboard`** is already written and uses SystemCore's built-in IMU
(no NavX needed, which matters because NavX has no 2027 release and SystemCore dropped SPI). Switch
one line in `RobotContainer`, confirm the mount orientation, and swap the two bindings back. Note
that `GyroIOOnboard` reads the raw Z gyro rate, which is only the robot's yaw rate when the
SystemCore is mounted `FLAT`.

---

## Controls

| Control | Action |
| --- | --- |
| Left stick | Translate (robot-relative) |
| Right stick X | Rotate |
| Left bumper (hold) | Field-relative instead (needs a gyro to be useful) |
| East face button (B) | Hold modules in an X |
| Start | Zero heading |

2027 replaced the individual controller classes with a single `Gamepad`, so bindings use
`CommandGamepad` and its face-direction button names (`eastFace()` rather than `b()`).

---

## Values worth a second look

These were carried across verbatim because they describe the existing robot, but they looked unusual
on the way over:

- **`MAX_LINEAR_SPEED` is 1 ft/s (~0.3 m/s).** That is the YAGSL project's `MAX_SPEED` unchanged — a
  deliberately crawling value. A NEO swerve module will do roughly 4–5 m/s. `MAX_ANGULAR_SPEED`
  scales off this, so raising one raises both.
- **Drive gear ratio 1.36 with a 2 in wheel.** Most swerve modules are between 4:1 and 8:1. Worth
  confirming against the physical module before driving at speed.
- **PID gains do not transfer.** `DRIVE_KP` and `TURN_KP` came from the YAGSL config, but YAGSL ran
  its loops in different units than this project does. They are starting points, not a tune.
- **Feedforward gains do transfer**, since they are physical — but they were measured in volts per
  m/s, so `Constants.Module` converts them to volts per wheel-rad/s. The originals are kept as
  `DRIVE_KV_PER_METER_PER_SEC` / `DRIVE_KA_PER_METER_PER_SEC2`.
- **Front-right static friction.** The 2026 project measured ~0.65 V against ~0.31–0.41 V on the
  other three corners. Still worth a physical look. Unlike YAGSL, this structure could hold
  per-module gains if you want to chase it.

Every one of these has a bring-up routine that measures it — see below.

---

## Bring-up and tuning

Several numbers above are inherited rather than measured, so the project ships routines that turn
each of them into a measurement. They appear on the **auto chooser** (the one place a command can be
selected and run without a controller binding) whenever `Constants.TUNING_MODE` is true, and they
all log to the `Tuning/` table — read the result from the plot in AdvantageScope, not from a number
on the dashboard.

| Routine | Measures | Moves the robot? |
| --- | --- | --- |
| Report Encoder Offsets | absolute encoder offsets | no — runs disabled |
| Turn Step Response | turn kP / kD | modules steer, wheels held still |
| Drive Step Response | drive kP | **yes** |
| Measure Wheel Radius | true wheel radius / gear ratio | **yes**, several metres |
| Drive SysId (all four) | drive kS / kV / kA, and a recommended kP | **yes**, both directions |

### Tuning kP without redeploying

Gains are `TunableNumber`s, which publish to the dashboard under `Tuning/Drive/*` and `Tuning/Turn/*`
and push down to the controllers the moment they change. So a tuning session is:

1. Select **Turn Step Response** (or **Drive Step Response**) and enable.
2. Watch `Tuning/TurnSetpointDeg` against `Drive/<module>/TurnPositionDeg` in AdvantageScope.
3. Drag `Tuning/Turn/kP` on the dashboard. Raise it until the module just begins to overshoot, then
   back off slightly or add a little kD.
4. Copy the settled value into `Constants` and set `TUNING_MODE = false` before competition — with
   it off, every `TunableNumber` collapses to its compiled-in value and no dashboard entry is
   created, so nothing depends on a number somebody forgot to set.

Because `TunableNumber` is built on AdvantageKit's `LoggedNetworkNumber`, the gains land in the log
too — a replay re-runs with the gains that were actually in effect, rather than whatever is compiled
in today.

### Suggested order

1. **Report Encoder Offsets** first. The offsets in `Constants` came from YAGSL, which read the
   Thrifty encoders through its own conversion; there is no reason to expect them to transfer. Point
   all four wheels forward by hand, run it disabled, copy the four printed numbers into
   `Constants.ModuleConfig`.
2. **Measure Wheel Radius**, because a wrong radius or gear ratio makes every distance the robot
   believes wrong by the same factor — including odometry.
3. **Drive SysId**, for kS/kV/kA in the correct units. Only meaningful once step 2 is settled.
4. **Step responses**, for the two kP values. Drive kP should only be closing a small gap; if it has
   to be large to reach the setpoint at all, the feedforward is wrong — go back to step 3 rather
   than fighting it with kP.

---

## Not yet carried over

- **PathPlanner / autonomous paths.** The auto chooser currently offers "Do Nothing" and the
  characterization routine. PathPlannerLib has a 2027 alpha-3 build that is compatible with alpha-6.
- **Vision.** PhotonLib has a 2027 alpha-2 build, also alpha-6-compatible.
- **Pose estimation from vision.** `Drive` tracks pose by wheel odometry only. WPILib's
  `SwerveDrivePoseEstimator` exists in alpha-6 and slots in behind the same interface when there is
  a vision measurement to fuse.

---

## Verification status

**This builds and runs.** `./gradlew build` succeeds against the real WPILib 2027 alpha-6 toolchain,
the robot program starts and loops in simulation, and an AdvantageKit replay round-trip has been
exercised end to end.

What has *not* been done: nothing has touched real hardware. `ModuleIOSpark` compiles against REVLib
2027.0.0-alpha-6 and every call resolves, but no SPARK MAX has answered any of them. Expect the
usual first-bringup work — encoder directions, absolute offsets, and the PID gains that did not
transfer from YAGSL.

`./gradlew build`, then `./gradlew simulateJava` to drive it in the simulator.
