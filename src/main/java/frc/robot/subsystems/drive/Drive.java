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
    modules[0] = new Module(frontLeft, MODULE_NAMES[0]);
    modules[1] = new Module(frontRight, MODULE_NAMES[1]);
    modules[2] = new Module(backLeft, MODULE_NAMES[2]);
    modules[3] = new Module(backRight, MODULE_NAMES[3]);

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
