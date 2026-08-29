# Yet Another Generic Swerve Library (YAGSL) Example project — 2026

This is the 2026-season port of [NerdSwerveYAGSL](https://github.com/), rebuilt on the 2026 WPILib base
project. The robot code, YAGSL swerve/PathPlanner configuration under `src/main/deploy`, and overall
structure are unchanged from the 2025 project — only the vendor libraries and the year-specific
GradleRIO/WPILib base were updated.

YAGSL is intended to be an easy implementation of a generic swerve drive that should work for most
square swerve drives. The project is documented [here](https://github.com/Yet-Another-Software-Suite/YAGSL/wiki).

This example is intended to be a starting place on how to use YAGSL. By no means is this intended to
be the base of your robot project. YAGSL provides an easy way to generate a SwerveDrive which can be
used in both TimedRobot and Command-Based Robot templates.

## Vendor library versions

The absolute-latest vendor releases as of build time (August 2026) include early previews for the
*2027* season (WPILib/REVLib/Phoenix 6/PhotonVision alpha builds tied to the new "SystemCore"
hardware) mixed in alongside the last stable 2026 releases. To keep this project on stable,
in-season code, every vendordep below is pinned to the last stable release from **around
April/May 2026** rather than whatever is newest today:

| Library | Version | Notes |
|---|---|---|
| GradleRIO / WPILib | 2026.2.1 | Last stable 2026 WPILib release (Jan 2026); no newer non-alpha release exists |
| YAGSL | 2026.4.1 | Apr 1, 2026 — last release before an Aug 2026 update burst |
| PathPlannerLib | 2026.1.2 | Jan 2026 — still the current stable release |
| CTRE Phoenix 6 | 26.3.0 | May 26, 2026 |
| CTRE Phoenix 5 | 5.36.0 | Current stable for 2026 |
| REVLib | 2026.0.5 | Current stable 2026 channel (REV's `2027.0.0-alpha` builds are on a separate track) |
| ReduxLib | 2026.1.2 | Current stable 2026 channel |
| Studica (NavX) | 2026.0.0 | Current stable 2026 channel |
| ThriftyLib | 2026.1.2 | Current stable 2026 channel |
| PhotonLib | v2026.3.4 | Apr 10, 2026 — last stable release before PhotonVision's 2027 alpha |
| maple-sim | 0.4.0-beta | Jan 17, 2026 — 2026 game-piece rebuild, still the current release |

All versions above were verified to actually resolve (`./gradlew build`) and, for YAGSL, to
successfully parse every config under `src/main/deploy/swerve` (`neo`, `maxSwerve`, `falcon`).

[Javadocs here](https://yet-another-software-suite.github.io/YAGSL/javadocs/)  
[Library here](https://github.com/Yet-Another-Software-Suite/YAGSL/)  
[WIKI](https://github.com/Yet-Another-Software-Suite/YAGSL/wiki)  
[Config Generation](https://docs.yagsl.com/)

## Migrating to a newer library version

If you later want to move off these pinned versions, use the WPILib VS Code extension's
"Manage Vendor Libraries" → "Install new library (online)" for each vendor, then re-run
`./gradlew build` and re-check that your `src/main/deploy/swerve/*` configs still parse — YAGSL's
JSON schema occasionally changes between seasons (see "Migrating Old Configuration Files" below,
kept from the upstream YAGSL-Example README for reference).

### My Robot Spins around uncontrollably during autonomous or when attempting to set the heading!

* Invert the gyroscope.
* Invert the drive motors for every module. (If front and back become reversed when turning)

### Angle motors are erratic.

* Invert the angle motor.

### My robot is heavy.

* Implement momentum velocity limitations in SwerveMath.

### Ensure the IMU is centered on the robot

# Maintainers (upstream YAGSL)
- @thenetworkgrinch
- @Technologyman00
