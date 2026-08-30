# NerdSwerve 2027 — Custom Swerve

Team 5010's swerve drivetrain, written from scratch against **WPILib 2027.0.0-alpha-6** using the
**Commands v2** framework, structured the way an AdvantageKit swerve project is structured.

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
`wpilibYear = '2027_alpha6'`. Everyone building this needs that installer.

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

`SmartDashboard` is only touched in two places — `util/Telem.java` and the chooser/field wiring in
`Drive` and `RobotContainer` — which is why `Telem` exists at all.

---

## Structure

The AdvantageKit pattern, without the AdvantageKit dependency:

```
subsystems/drive/
  Drive.java           subsystem: kinematics, odometry, control
  Module.java          per-module logic: units, optimization, telemetry
  ModuleIO.java        hardware interface + ModuleIOInputs struct
  ModuleIOSim.java       physics sim
  ModuleIOSpark.java     SPARK MAX + NEO + Thrifty analog encoder
  GyroIO.java          gyro interface + GyroIOInputs struct
  GyroIONone.java        no gyro (current default)
  GyroIOOnboard.java     SystemCore's built-in IMU
commands/DriveCommands.java   command factories
util/Telem.java               telemetry facade
```

Every loop reads all hardware into the `*Inputs` structs first, then runs control off that one
snapshot. That is the property that makes replay possible and that keeps sim and real behaviour
identical.

**AdvantageKit itself is deliberately not a dependency.** The request was for code that works the way
AdvantageKit swerve code works, and the structure above delivers that: hardware swapped by
constructor argument, logic testable without a robot, sim and real sharing one control path. Taking
the actual dependency would add a vendordep that is itself built for alpha-5, plus the `gversion`
build plumbing. The `*Inputs` classes are plain structs of primitives, so if you do want real replay
logging later, annotating them `@AutoLog` and swapping `Telem` for `Logger` is the whole job.

---

## Gyro

There is no gyro wired up. `RobotContainer` constructs `GyroIONone`, and `Drive` handles that by
integrating module positions to track heading — field-relative driving works, it just drifts.

The YAGSL config used a NavX, which has no 2027 release and which SystemCore could not talk to
anyway now that SPI is removed. The replacement is already written: **`GyroIOOnboard`** uses
SystemCore's built-in IMU. To switch, change one line in `RobotContainer` and confirm the mount
orientation — `MountOrientation.FLAT` assumes the SystemCore is mounted horizontally. Note that
`GyroIOOnboard` reads the raw Z gyro rate, which is only the robot's yaw rate when mounted `FLAT`.

Hold **left bumper** to drive robot-relative, which is what you want whenever the heading estimate
is not trustworthy.

---

## Controls

| Control | Action |
| --- | --- |
| Left stick | Translate (field-relative) |
| Right stick X | Rotate |
| Left bumper (hold) | Drive robot-relative instead |
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

Re-run `DriveCommands.feedforwardCharacterization` (available from the auto chooser) after any
gearing or wheel change.

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

This code has **not been compiled.** WPILib 2027 targets Java 25 and its jars are Java 25 class
files; no JDK 25 was available in the environment where this was written, and alpha-6's jars are not
on the public mirror. Instead, every WPILib class and method it calls was checked individually
against the alpha-6 sources at tag `v2027.0.0-alpha-6` and the alpha-7 jars. That caught real
differences — `MathUtil.clamp` and `LinearSystemId` are both gone in 2027, and `AnalogEncoder` has
no `isConnected()` — but it is not a substitute for a build.

`ModuleIOSpark` is the least verified file: `maven.revrobotics.com` was unreachable, so its REVLib
calls follow the 2025/2026 API from memory of that API's shape rather than from the alpha-6 jar. If
anything there fails to resolve, it is contained to that one file — everything else talks to
`ModuleIO`.

**Build it first.** `./gradlew build`, then `./gradlew simulateJava` to drive it in the simulator.
