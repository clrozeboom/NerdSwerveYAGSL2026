# Upgrading to WPILib 2027.0.0-alpha-7

Status as of 2026-09-02: **feasible now.** AdvantageKit 27.0.0-alpha-5 shipped clean against
alpha-7 and cleared the hard blocker. Our own code is ready to move on a few hours' notice.

Release: <https://github.com/wpilibsuite/allwpilib/releases/tag/v2027.0.0-alpha-7>

---

## The gate

alpha-7 deletes the `Sendable` world — `Sendable`, `SmartDashboard`, `SendableChooser`,
`NTSendable`, `NTSendableBuilder` — and replaces it with the new `org.wpilib.telemetry` and
`org.wpilib.tunable` modules. It also moves `Pair` and `Alert` out of the packages they were in at
alpha-6. Anything compiled against the old shape breaks.

`Field2d` survives, contrary to what the release notes read like: `org.wpilib.smartdashboard` still
ships `Field2d`, `FieldObject2d` and the whole `Mechanism2d` family. Only the Sendable plumbing
underneath went away.

Measured by extracting each jar and diffing its `org/wpilib/…` constant-pool references against a
class index built from all 21 alpha-7 `org.wpilib` java artifacts (1063 classes):

| Dependency | Version | Classes missing in alpha-7 | Verdict |
| --- | --- | --- | --- |
| **AdvantageKit** | 27.0.0-alpha-5 | — | **Clear** |
| **REVLib** | 2027.0.0-alpha-6 | `math.util.Pair` (moved to `util.Pair`) | Clear in practice — see below |
| Phoenix6 | 26.50.0-alpha-1 | `epilogue.logging.EpilogueBackend`, `NestedBackend` | Blocker for `BasicRobotLessons` |
| PhotonLib | v2027.0.0-alpha-2 | `Alert`, `math.util.Pair`, `SmartDashboard`, `Sendable`, `AprilTag`, `AprilTagFieldLayout`, `AprilTagFields` | Blocker for `BasicRobotLessons` |
| WPILib core | 2027.0.0-alpha-7 | — | Installer published |

**AdvantageKit cleared on 2026-09-02.** 27.0.0-alpha-4 referenced five load-bearing Sendable
classes and could not run on alpha-7. 27.0.0-alpha-5 references none of them — it publishes through
raw NetworkTables topics instead, and `LoggedDashboardChooser` was replaced by
`LoggedNetworkChooser`, which no longer needs `SendableChooser`. All seven AdvantageKit classes this
project imports (`Logger`, `LoggedRobot`, `AutoLog`, `LoggedNetworkNumber`, `NT4Publisher`,
`WPILOGReader`, `WPILOGWriter`) survive the rewrite unchanged, and we never used the chooser, so the
rename costs us nothing. `akit-autolog` 27.0.0-alpha-5 is published alongside it.

**REVLib's one class cannot reach us.** `math.util.Pair` moved to `util.Pair`, and REVLib's only
reference to it is in `com.revrobotics.sim.MovingAverageFilterSim`, which is reachable solely from
`SparkSim`. This project simulates through its own `ModuleIOSim` and never touches REVLib's sim
classes — none of the eleven REVLib types we import reference `com.revrobotics.sim` at all. A
missing class only throws when the JVM loads the class that references it, so this one never fires
here. It is recorded as a waiver in the readiness check rather than left as a footnote, so the
verdict reflects it and a *different* missing class in a future REVLib build still blocks.

### Vendor ecosystem state

The vendordep marketplace still has **no `2027_alpha6` or `2027_alpha7` bucket** — the newest is
`2027_alpha5`, and every 2027 vendordep still declares `wpilibYear: 2027_alpha5`. AdvantageKit
alpha-5 cleared without that bucket appearing, which is the useful lesson: the bucket is a
convenience, not the compatibility signal. Only the per-jar reference diff answers the question.

---

## Our own code: ready, and small

Nothing here is blocked. The whole migration is mechanical and confined to six files:

| Change | Occurrences | Where |
| --- | --- | --- |
| `Rotation2d.kZero` / `Pose2d.kZero` / `Translation2d.kZero` → `.ZERO` | 10 | `Drive`, `Module`, `ModuleIO`, `GyroIO`, `DriveCommands` |
| `Translation2d.getAngle()` now returns `Optional<Rotation2d>` | 8 call sites to audit (only the `Translation2d` ones matter) | `Drive.stopWithX`, `Drive.tangentAngle` |
| `SmartDashboard` → `org.wpilib.telemetry.Telemetry` | 4 | `Drive` (Field2d publish), `RobotContainer` (chooser) |
| `SendableChooser` → `org.wpilib.tunable.Selectable` + `Tunables.publish` | 2 | `RobotContainer` |
| `Field2d` | 2 | `Drive` — the class survives, but publishing it went through `SmartDashboard`, so the publish call moves to `Telemetry` |

`Pair` and `Alert` are not used by our code at all, so those two moves cost us nothing.

Note that `TunableNumber` is built on AdvantageKit's `LoggedNetworkNumber`, so it moves with
AdvantageKit rather than with WPILib. If AdvantageKit's alpha-7 release changes that class, the
tuning entries follow it.

---

## Plan, in order

1. ~~**Wait for AdvantageKit.**~~ Done — 27.0.0-alpha-5, 2026-09-02. Bump `akitVersion` in
   `build.gradle` from `27.0.0-alpha-4`; the `akit-autolog` annotation processor moves with it.
2. ~~**Confirm REVLib.**~~ Done — the remaining `Pair` reference sits on a path we never call.
   Re-check this if REVLib publishes a new build.
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

### Fallback

`claude/swerve-2027-alpha6` has no AdvantageKit dependency and needs only the mechanical renames
plus the `Telemetry` swap. It loses replay but depends on nobody else's release schedule. Now that
AdvantageKit has cleared this is insurance rather than a plan, but keep it alive until the alpha-7
branch has actually run on hardware.

---

## Automated watch

`tools/check-alpha7-readiness.sh` is the check; a routine runs it every 6 hours and reports when the
picture changes. It covers this repo and `FRC5010/BasicRobotLessons` together, because both are
gated on the same WPILib release and differ only in which vendordeps they pin.

What it does:

1. Downloads all 21 alpha-7 `org.wpilib` java artifacts and builds a class index from them. Nothing
   about which classes were deleted is hardcoded — the index is the answer, so the check stays
   correct when alpha-8 deletes something nobody has written down yet. It refuses to report a
   verdict if any artifact fails to download, rather than calling every vendor blocked because a
   fetch timed out.
2. Downloads each pinned vendordep jar — AdvantageKit and REVLib for this repo, Phoenix6 and
   PhotonLib for the lessons repo — and diffs its `org/wpilib/…` constant-pool references against
   that index. Anything referenced but not in the index is a class that no longer exists.
3. Checks whether a `2027_alpha6` or `2027_alpha7` vendordep marketplace bucket has appeared, and
   lists every 2027 vendordep published so far for context.

It also carries a `WAIVED` table: missing classes analysed once and recorded as unreachable for the
project that pins them, with the reasoning inline. Waivers are keyed on the exact class, so a new
missing class in a later vendor build is never cleared silently — and they are per-vendor, so
PhotonLib is still counted as blocked on the same `math.util.Pair` that is waived for REVLib.

Exit status is 0 when every pin is clean, 1 when at least one is blocked, 2 when the check itself
could not run. It prints a separate verdict per project, so one repo can go green while the other
waits — which is now the case.

`maven.revrobotics.com` is reachable from this environment as of 2026-09-02, but is not always
reachable from a sandbox, and REVLib's marketplace JSON is named with a different version string
than its maven artifact (`2027.0.0-alpha6` vs `2027.0.0-alpha-6`), so the URL cannot be
reconstructed from the listing when it is not. When the fetch fails the check falls back to the copy
in the Gradle cache and labels it as such; with neither, it reports REVLib as unverified rather than
guessing.
