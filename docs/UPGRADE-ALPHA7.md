# Upgrading to WPILib 2027.0.0-alpha-7

Status as of 2026-08-30: **not yet feasible.** One hard blocker, one minor one. Our own code is
ready to move on a few hours' notice.

Release: <https://github.com/wpilibsuite/allwpilib/releases/tag/v2027.0.0-alpha-7>

---

## The gate

alpha-7 deletes the entire `Sendable` world — `Sendable`, `SmartDashboard`, `SendableChooser`,
`Field2d`, `NTSendable`, `NTSendableBuilder` — and replaces it with the new `org.wpilib.telemetry`
and `org.wpilib.tunable` modules. It also moves `Pair` and `Alert` out of the packages they were in
at alpha-6. Anything compiled against the old shape breaks.

Measured by extracting each jar and diffing its `org/wpilib/…` constant-pool references against the
full alpha-7 class index:

| Dependency | Version | Classes missing in alpha-7 | Verdict |
| --- | --- | --- | --- |
| **AdvantageKit** | 27.0.0-alpha-4 | `SendableChooser`, `SmartDashboard`, `util.sendable.Sendable`, `networktables.NTSendable`, `NTSendableBuilder` | **Hard blocker** |
| **REVLib** | 2027.0.0-alpha-6 | `math.util.Pair` (moved to `util.Pair`) | Minor — one class |
| WPILib core | 2027.0.0-alpha-7 | — | Installer published |

**AdvantageKit is the blocker.** Five missing classes, all load-bearing: `LoggedDashboardChooser`
is built on `SendableChooser`, and the NT publishing path uses `NTSendable`. There is no way to
work around that from our side — it needs a release from Littleton.

**REVLib is one class.** `math.util.Pair` moved to `util.Pair`. Whether that actually throws depends
on whether the code path using it runs; it would surface as a `NoClassDefFoundError` at the moment
that API is first touched, not at startup. Not something to gamble a build on, but close enough that
a REVLib alpha-7 build is likely to be a quick turnaround.

### Vendor ecosystem state

The vendordep marketplace has **no `2027_alpha6` or `2027_alpha7` bucket** — the newest is
`2027_alpha5`. Every 2027 vendordep still declares `wpilibYear: 2027_alpha5`. We run on alpha-6 today
only because alpha-6 happened to keep every class those alpha-5 builds reference. alpha-7 breaks that
truce.

So the practical signal to watch is not the WPILib release — that is already out — but **the
appearance of a `2027_alpha6`/`2027_alpha7` marketplace bucket**, and specifically an AdvantageKit
release in it.

---

## Our own code: ready, and small

Nothing here is blocked. The whole migration is mechanical and confined to six files:

| Change | Occurrences | Where |
| --- | --- | --- |
| `Rotation2d.kZero` / `Pose2d.kZero` / `Translation2d.kZero` → `.ZERO` | 10 | `Drive`, `Module`, `ModuleIO`, `GyroIO`, `DriveCommands` |
| `Translation2d.getAngle()` now returns `Optional<Rotation2d>` | 8 call sites to audit (only the `Translation2d` ones matter) | `Drive.stopWithX`, `Drive.tangentAngle` |
| `SmartDashboard` → `org.wpilib.telemetry.Telemetry` | 4 | `Drive` (Field2d publish), `RobotContainer` (chooser) |
| `SendableChooser` → `org.wpilib.tunable.Selectable` + `Tunables.publish` | 2 | `RobotContainer` |
| `Field2d` | 2 | `Drive` — deleted with `Sendable`; needs a `Telemetry`-based pose publish instead |

`Pair` and `Alert` are not used by our code at all, so those two moves cost us nothing.

Note that `TunableNumber` is built on AdvantageKit's `LoggedNetworkNumber`, so it moves with
AdvantageKit rather than with WPILib. If AdvantageKit's alpha-7 release changes that class, the
tuning entries follow it.

---

## Plan, in order

1. **Wait for AdvantageKit.** Nothing else can start until there is a build whose jar does not
   reference the deleted Sendable classes. The routine below watches for this.
2. **Confirm REVLib.** Either an alpha-7 build appears, or re-run the reference diff and confirm the
   remaining `Pair` reference sits on a path we never call.
3. **Install the alpha-7 toolchain.** `frcmaven.wpi.edu/artifactory/installer/v2027.0.0-alpha-7/`
   ships the JDK and the maven repo together, the same way alpha-6 did.
4. **Branch** from `claude/swerve-2027-advantagekit` — do not upgrade in place, so there is a working
   robot to fall back to.
5. **Build files:** `settings.gradle` `wpilibYear`, `.wpilib/wpilib_preferences.json` `projectYear`,
   GradleRIO plugin version, vendordep JSONs. Diff against the alpha-7 project template rather than
   hand-editing — that is what caught the shadow-jar and `debugJni` differences last time.
6. **Source migration**, per the table above. Run the constant renames first, then handle
   `Telemetry`/`Selectable`, which is the only part that is a rewrite rather than a rename.
7. **Verify the same way as before:** `./gradlew build`, then run the shadow jar headless and check
   for a clean startup with zero loop overruns, then an AdvantageKit replay round-trip.

### If AdvantageKit stalls

The fallback is dropping back to the plain-WPILib branch (`claude/swerve-2027-alpha6`), which has no
AdvantageKit dependency and only needs the mechanical renames plus the `Telemetry` swap. That branch
loses replay but is not blocked by anyone else's release schedule. Keep it alive as insurance.

---

## Automated watch

A routine runs every 6 hours and reports when the picture changes. It checks:

- whether a `2027_alpha6` or `2027_alpha7` vendordep marketplace bucket has appeared
- the newest AdvantageKit, REVLib, PathPlannerLib, PhotonLib, Phoenix6, ReduxLib and ThriftyLib
  versions, and the `wpilibYear` each declares
- for AdvantageKit and REVLib specifically, whether the published jar still references classes
  alpha-7 deleted — the actual compatibility test, not just the version label

It reports **GO** only when AdvantageKit ships a build that is clean against alpha-7. Anything else
is a status line, and it stays quiet when nothing has changed.
