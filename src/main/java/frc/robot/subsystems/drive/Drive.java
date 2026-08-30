// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;

/**
 * Four-module swerve drivetrain.
 *
 * <p>Structured the way an AdvantageKit swerve project is: this subsystem owns the kinematics, the
 * pose estimate and the control logic, and reaches hardware only through {@link ModuleIO} and
 * {@link GyroIO}. Swapping simulation for real hardware is a constructor argument, not a code
 * change.
 *
 * <p>Heading comes from the gyro when one is connected and from integrated module positions when it
 * is not, so the drivetrain is fully drivable before any gyro is wired up.
 */
public class Drive extends SubsystemBase {
  private static final String[] MODULE_NAMES = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

  /** Matches TimedRobot's default period; used to discretize commanded velocities. */
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Constants.Drivebase.MODULE_TRANSLATIONS);

  private final Field2d field = new Field2d();

  /** Heading used for field-relative driving and odometry. */
  private Rotation2d rawGyroRotation = Rotation2d.kZero;

  /** Module positions as of the previous loop, used to integrate heading when there is no gyro. */
  private SwerveModulePosition[] lastModulePositions = {
    new SwerveModulePosition(), new SwerveModulePosition(),
    new SwerveModulePosition(), new SwerveModulePosition()
  };

  private Pose2d pose = Pose2d.kZero;

  public Drive(
      GyroIO gyroIO,
      ModuleIO frontLeft,
      ModuleIO frontRight,
      ModuleIO backLeft,
      ModuleIO backRight) {
    this.gyroIO = gyroIO;
    // ModuleConfig.ORDERED is in the same front-left, front-right, back-left, back-right order as
    // MODULE_NAMES and MODULE_TRANSLATIONS, so index i lines up across all three.
    modules[0] = new Module(frontLeft, MODULE_NAMES[0], Constants.ModuleConfig.ORDERED[0]);
    modules[1] = new Module(frontRight, MODULE_NAMES[1], Constants.ModuleConfig.ORDERED[1]);
    modules[2] = new Module(backLeft, MODULE_NAMES[2], Constants.ModuleConfig.ORDERED[2]);
    modules[3] = new Module(backRight, MODULE_NAMES[3], Constants.ModuleConfig.ORDERED[3]);

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // Read every input first, so the rest of the loop works from one consistent snapshot. This is
    // the property that makes AdvantageKit-style replay possible, and it is worth keeping even
    // without the library.
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (Module module : modules) {
      module.updateInputs();
    }

    updateOdometry();

    Logger.recordOutput("Drive/GyroConnected", gyroInputs.connected);
    Logger.recordOutput("Drive/HeadingDeg", getRotation().getDegrees());
    Logger.recordOutput("Drive/PoseX", pose.getX());
    Logger.recordOutput("Drive/PoseY", pose.getY());
    ChassisVelocities measured = getChassisVelocities();
    Logger.recordOutput("Drive/MeasuredVx", measured.vx);
    Logger.recordOutput("Drive/MeasuredVy", measured.vy);
    Logger.recordOutput("Drive/MeasuredOmega", measured.omega);
    field.setRobotPose(pose);
  }

  private void updateOdometry() {
    SwerveModulePosition[] positions = getModulePositions();
    // How far the chassis moved and rotated since the last loop, derived from how far each wheel
    // rolled and which way it was pointing.
    Twist2d twist = kinematics.toTwist2d(lastModulePositions, positions);

    Rotation2d newRotation;
    if (gyroInputs.connected) {
      newRotation = gyroInputs.yawPosition;
    } else {
      // No gyro: fall back to the rotation the modules imply. Drifts over time, but keeps
      // field-relative driving usable until a gyro is wired up.
      newRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // Integrate translation using the heading change we actually believe rather than the one the
    // wheels implied, so that with a gyro connected the pose's position and rotation stay
    // consistent with each other. This is what WPILib's own SwerveDriveOdometry does.
    twist.dtheta = newRotation.minus(rawGyroRotation).getRadians();
    pose = pose.plus(twist.exp());
    pose = new Pose2d(pose.getTranslation(), newRotation);

    rawGyroRotation = newRotation;
    lastModulePositions = positions;
  }

  /**
   * Drives the robot at the given chassis velocities, in robot-relative terms.
   *
   * @param velocities desired forward, sideways and rotational velocity
   */
  public void runVelocity(ChassisVelocities velocities) {
    // Compensate for the fact that the robot moves in a straight line between discrete loops while
    // the commanded motion is a curve; without this the robot drifts when translating and rotating
    // at the same time.
    ChassisVelocities discretized = velocities.discretize(LOOP_PERIOD_SECS);
    // 2027 made the kinematics helpers immutable: desaturate returns a new array rather than
    // scaling the one passed in, so the result has to be assigned.
    SwerveModuleVelocity[] setpoints =
        SwerveDriveKinematics.desaturateWheelVelocities(
            kinematics.toSwerveModuleVelocities(discretized),
            Constants.Drivebase.MAX_LINEAR_SPEED);

    for (int i = 0; i < modules.length; i++) {
      modules[i].runSetpoint(setpoints[i]);
    }

    Logger.recordOutput("Drive/SetpointVx", discretized.vx);
    Logger.recordOutput("Drive/SetpointVy", discretized.vy);
    Logger.recordOutput("Drive/SetpointOmega", discretized.omega);
  }

  /** Stops all four modules where they are. */
  public void stop() {
    runVelocity(new ChassisVelocities());
  }

  /**
   * Stops and points the modules into an X, so the robot resists being pushed. The angles come from
   * each module's position relative to robot centre.
   */
  public void stopWithX() {
    for (int i = 0; i < modules.length; i++) {
      Rotation2d angle = Constants.Drivebase.MODULE_TRANSLATIONS[i].getAngle();
      modules[i].runSetpoint(new SwerveModuleVelocity(0.0, angle));
    }
  }

  /** Runs every drive motor open-loop at the same voltage, for feedforward characterization. */
  public void runCharacterization(double volts) {
    for (Module module : modules) {
      module.runCharacterization(volts);
    }
  }

  /**
   * Runs every drive motor open-loop with the modules pointed tangentially, so the robot spins in
   * place instead of driving in a straight line.
   *
   * <p>Same electrical test as {@link #runCharacterization(double)}, but it fits in the robot's own
   * footprint. Each wheel still travels a real distance against the robot's inertia, so the wheel
   * velocity SysId reads is just as valid — the wheels are simply following a circle rather than a
   * line.
   *
   * @param volts voltage to apply to every drive motor
   */
  public void runCharacterizationSpin(double volts) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].runTurnSetpoint(tangentAngle(i));
      modules[i].runCharacterizationDriveOnly(volts);
    }
  }

  /**
   * The heading that points a module along its circle about robot centre — its position angle turned
   * a quarter turn. Driving all four at this angle spins the robot counter-clockwise.
   */
  static Rotation2d tangentAngle(int moduleIndex) {
    return Constants.Drivebase.MODULE_TRANSLATIONS[moduleIndex]
        .getAngle()
        .plus(Rotation2d.kCCW_Pi_2);
  }

  /**
   * Commands one wheel speed to all four modules with the modules pointed tangentially, spinning the
   * robot in place. The spin step response uses this to tune drive kP in a small space.
   *
   * @param velocityRadPerSec wheel speed, in radians per second
   */
  public void runSpinSetpoint(double velocityRadPerSec) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].runTurnSetpoint(tangentAngle(i));
      modules[i].runDriveSetpoint(velocityRadPerSec);
    }
    Logger.recordOutput("Tuning/SpinSetpointRadPerSec", velocityRadPerSec);
    Logger.recordOutput(
        "Tuning/SpinSetpointRobotRadPerSec",
        velocityRadPerSec * Constants.Module.WHEEL_RADIUS / Constants.Drivebase.DRIVE_BASE_RADIUS);
  }

  /**
   * Points all four modules at one angle without moving the wheels. The turn step-response test uses
   * this: step the angle, then watch setpoint against measured to judge turn kP.
   *
   * @param angle the heading to command
   */
  public void runTurnSetpoint(Rotation2d angle) {
    for (Module module : modules) {
      module.runTurnSetpoint(angle);
      module.runDriveSetpoint(0.0);
    }
    Logger.recordOutput("Tuning/TurnSetpointDeg", angle.getDegrees());
  }

  /**
   * Commands one wheel speed to all four modules with the modules held straight. The drive step test
   * uses this to judge drive kP against a square-wave setpoint.
   *
   * @param velocityRadPerSec wheel speed, in radians per second
   */
  public void runDriveSetpoint(double velocityRadPerSec) {
    for (Module module : modules) {
      module.runTurnSetpoint(Rotation2d.kZero);
      module.runDriveSetpoint(velocityRadPerSec);
    }
    Logger.recordOutput("Tuning/DriveSetpointRadPerSec", velocityRadPerSec);
    Logger.recordOutput(
        "Tuning/DriveSetpointMetersPerSec", velocityRadPerSec * Constants.Module.WHEEL_RADIUS);
  }

  /** Average measured wheel speed across the four modules, in rad/s. Pairs with the step test. */
  public double getAverageWheelVelocityRadPerSec() {
    double sum = 0.0;
    for (Module module : modules) {
      sum += module.getVelocityMetersPerSec() / Constants.Module.WHEEL_RADIUS;
    }
    return sum / modules.length;
  }

  /** The four modules, so bring-up routines can address them individually. */
  public Module[] getModules() {
    return modules;
  }

  /** Average wheel travel across the four modules, in radians. Pairs with SysId data. */
  public double getCharacterizationPosition() {
    double sum = 0.0;
    for (Module module : modules) {
      sum += module.getWheelRadiansForCharacterization();
    }
    return sum / modules.length;
  }

  /** Switches all four drive motors between brake and coast. */
  public void setBrakeMode(boolean enabled) {
    for (Module module : modules) {
      module.setBrakeMode(enabled);
    }
  }

  /** Current wheel travel and heading for each module, in FL, FR, BL, BR order. */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /** Current wheel speed and heading for each module, in FL, FR, BL, BR order. */
  public SwerveModuleVelocity[] getModuleVelocities() {
    SwerveModuleVelocity[] velocities = new SwerveModuleVelocity[modules.length];
    for (int i = 0; i < modules.length; i++) {
      velocities[i] = modules[i].getVelocity();
    }
    return velocities;
  }

  /** Chassis velocity as measured by the modules, robot-relative. */
  public ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleVelocities());
  }

  /** Robot heading, from the gyro when present and from the modules when not. */
  public Rotation2d getRotation() {
    return rawGyroRotation;
  }

  /** Current pose estimate. */
  public Pose2d getPose() {
    return pose;
  }

  /** Overwrites the pose estimate, e.g. at the start of an autonomous routine. */
  public void setPose(Pose2d newPose) {
    pose = newPose;
    rawGyroRotation = newPose.getRotation();
    lastModulePositions = getModulePositions();
  }

  /** Zeroes the heading so that the direction the robot currently faces becomes "forward". */
  public void zeroHeading() {
    gyroIO.resetYaw();
    rawGyroRotation = Rotation2d.kZero;
    pose = new Pose2d(pose.getTranslation(), Rotation2d.kZero);
  }

  /** The drivetrain's kinematics, exposed for path-following code added later. */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /** True when all four modules are reporting healthy. */
  public boolean allModulesConnected() {
    for (Module module : modules) {
      if (!module.isConnected()) {
        return false;
      }
    }
    return true;
  }
}
