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
  other three corners — still worth a physical look, since that much extra drag usually means
  something is binding. The feedforward gains are now held **per module** so that spread can be
  carried rather than averaged away; see below.

Every one of these has a bring-up routine that measures it — see below.

---

## Bring-up and tuning

Several numbers above are inherited rather than measured, so the project ships routines that turn
each of them into a measurement. They appear on the **auto chooser** (the one place a command can be
selected and run without a controller binding) whenever `Constants.TUNING_MODE` is true, numbered in
the order of the bring-up sequence below, and they all log to the `Tuning/` table — read the result from the plot in AdvantageScope, not from a number
on the dashboard.

| Routine | Measures | Space needed | On blocks? |
| --- | --- | --- | --- |
| Report Encoder Offsets | absolute encoder offsets | none — runs disabled | **yes** |
| Turn Step Response | turn kP / kD | none — wheels held still | floor |
| Spin Step Response | drive kP | ~1 m around the robot | floor |
| Drive Step Response | drive kP | ~0.5 m per cycle, **open-ended** | floor |
| Measure Wheel Radius | true wheel radius / gear ratio | ~1.0 m, then stops | **no** |
| Feedforward Ramp | drive kS / kV, read off a plot | ~1 m around the robot | floor |
| Spin SysId (all four) | drive kS / kV | ~1 m around the robot | floor |
| Drive SysId (all four) | drive kS / kV / kA | ~2.5 m per run, both ways | floor |

### Spin instead of straight line

Both driving tests have a spin variant that points the modules tangentially so the robot rotates on
the spot. **Prefer these.** They measure the same thing in about a metre of clearance instead of
three metres of runway, and the spin step response never leaves the robot's own footprint however
long you leave it running — unlike the straight-line version, which is open-ended.

kS and kV transfer exactly: both are per-wheel properties, and a wheel does the same work following
a circle as following a line. The only quantity that does not transfer is kA, which in a spin
reflects the robot's rotational inertia rather than its mass — for this robot roughly 0.6–0.7× the
straight-line value, depending on where the mass sits. **That does not matter here:** the drive
feedforward is `kS·sign(v) + kV·v`, with no kA term anywhere in the control path. Run the
straight-line SysId only if you later add something that needs kA.

Each spin SysId run is about 1.75 rotations. `SpinGeometryTest` checks that the tangent angles
really do produce a pure rotation, so a sign error there fails the build rather than quietly
smearing translation into the results.

### What can be done on blocks

Only **Report Encoder Offsets** is genuinely a blocks job — it is also the easiest place to sight the
wheels straight, and it runs while disabled.

Everything else wants weight on the wheels:

- **Turn kP on blocks will be under-tuned.** Steering friction with the robot's weight on the wheels
  is a real load; gains that look crisp with the wheels hanging will be sluggish on the floor.
- **Drive kP and SysId on blocks measure an unloaded drivetrain.** kV survives reasonably, kS comes
  out low because there is no weight-dependent friction, and kA is meaningless with the robot's mass
  absent. Use the spin variants on the floor instead — they cost about as little space as blocks do
  and keep the load real.
- **Measure Wheel Radius cannot be done on blocks at all.** It works by comparing believed distance
  against a tape measure, and on blocks there is no distance.

### How much floor space

Worked from the feedforward (`kS = 0.4234 V`, `kV = 1.0618 V per m/s`), so these are planning
figures, not guarantees — kS and kV are part of what SysId is being run to re-measure. The likely
error is on the safe side: if the drivetrain is geared down more than the config claims, it will be
slower than predicted and use less room.

- **Drive Step Response** — 10 rad/s (0.25 m/s) for a 2 s "on" phase is **~0.5 m per cycle**, and it
  runs until interrupted. Five cycles is ~2.5 m. Watch two or three cycles and disable; don't leave
  it running.
- **Measure Wheel Radius** — 40 wheel radians at 0.2 m/s is **~1.0 m over ~5 s**, then it stops
  itself. But it stops on *believed* distance, so if the wheel is larger than the configured 2 in it
  will travel proportionally further. Give it 3 m the first time.
- **Drive SysId** — **~2.3 m for each quasistatic run and ~2.2 m for each dynamic**, four runs total,
  alternating forward and reverse. Clear roughly 3 m each way from the start point and it will stay
  inside that. The routine prints its own predicted distances to the console before it moves.

A space about **4 m long by 2 m wide** covers everything comfortably. In a smaller space, drop
`Constants.SysId.QUASISTATIC_TIMEOUT_SECS` — distance scales with roughly its square, so 4 s → 3 s
takes the quasistatic run from ~2.3 m to ~1.2 m.

Note the tuning routines command the modules directly and **bypass** `MAX_LINEAR_SPEED`, so the 1
ft/s teleop cap does not apply to them.

### Per-module feedforward

`kS` and `kV` live on each corner in `Constants.ModuleConfig`, not once for the drivetrain. Static
friction genuinely differs between four hand-built modules — bearing preload, seal drag, gear mesh —
and the 2026 measurement saw front-right running roughly 0.65 V against 0.31–0.41 V elsewhere, which
is too large a spread to average away. YAGSL could only hold one feedforward for the whole
drivetrain; this can hold four.

All four currently default to the averaged `Module.DRIVE_KS`, because the old measurement recorded
*which* corner was the outlier but not which value belonged to each of the other three. Nothing is
invented here — run the feedforward ramp and fill in the real numbers.

The **PID** gains stay shared. They describe how hard the controller pushes on an error rather than
a property of the hardware, so four copies would be four things to keep in step for no physical
reason. Dashboard keys reflect the split: `Tuning/Drive/<Module>/kS` per corner, `Tuning/Drive/kP`
once.

`PerModuleGainsTest` asserts the corners really are independent, so a refactor back to a shared
static fails the build.

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

The first three steps need only about a metre of clear floor around the robot, so all of the gain
tuning can happen in a corner of the shop. Only step 4 needs a runway, and it can wait until you
have one.

1. **Report Encoder Offsets.** Point all four wheels forward by hand and run it disabled — blocks are
   the easiest place to sight them straight. The offsets in `Constants` came from YAGSL, which read
   the Thrifty encoders through its own conversion, so there is no reason to expect them to transfer.
   Copy the four printed numbers into `Constants.ModuleConfig` and redeploy. Nothing below means much
   until this is right.
2. **Feedforward Ramp** or **Spin SysId**, for kS and kV in the correct units — the two gains the
   drive feedforward actually uses. Both spin in place and both cover about 1.75 rotations per run.
   The ramp is one run and you read the answer straight off a plot: put
   `Tuning/Feedforward/<Module>/SpeedMetersPerSec` on the x axis against `Tuning/Feedforward/Volts`
   on the y, and each corner's intercept is its kS while the slope is its kV. Four lines lying on top
   of each other means the modules match; one offset upward is the corner with extra static friction. Spin SysId is four runs and needs the log exported
   to the SysId tool, but gives you its statistics and a recommended kP alongside. Start with the
   ramp to see whether the numbers are sane; reach for SysId when you want them properly fitted.
3. **Spin Step Response** and **Turn Step Response**, for the two kP values. Drive kP should only be
   closing a small gap; if it has to be large to reach the setpoint at all, the feedforward is wrong
   — go back to step 2 rather than fighting it with kP. Tune turn kP with weight on the wheels, not
   on blocks: steering friction under load is a real part of what the controller is fighting.
4. **Measure Wheel Radius**, once you have a few metres. A wrong radius or gear ratio makes every
   distance the robot believes wrong by the same factor, so this matters for odometry and for any
   path following added later — but it does not affect steering or velocity control, so it can wait
   for a bigger space. Give it 3 m the first time: it stops on *believed* distance, so a larger wheel
   than configured overshoots.

Optional, and only if something later needs kA: **Drive SysId** (the straight-line version) measures
it properly, at the cost of ~2.5 m each way. The current control path has no kA term, so this is not
part of normal bring-up. Note that WPILib's stock SysId config would need ~43 m on this drivetrain —
the inherited 1.36:1 gearing implies roughly 11 m/s free speed — which is why `Constants.SysId`
overrides it. Revisit those numbers once step 4 has established the real gearing.

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
