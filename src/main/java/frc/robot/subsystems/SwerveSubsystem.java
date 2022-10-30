// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//TODO: Measure velocity, trackwidth, and wheelbase. Integrate with commands.
package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;


public class SwerveSubsystem extends SubsystemBase {
  // Max voltage, can be capped to limit speed of robot during testing
  public static final double MAX_VOLTAGE = 12.0;
  double voltage = 12.0;
  public double setMaxOutput(double percent) {
        return voltage = percent*MAX_VOLTAGE;
  }
  public double getMaxOutput() {
        return voltage;
  }
  // TODO: Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  final static double DRIVETRAIN_TRACKWIDTH_METERS = 0.0;
  final static double DRIVETRAIN_WHEELBASE_METERS = 0.0;
    //TODO: Measure DRIVETRAIN_TRACKWIDTH_METERS and DRIVETRAIN_WHEELBASE_METERS
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  public ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public SwerveSubsystem() {
    ShuffleboardTab DRIVEBASE_TAB = Shuffleboard.getTab("Drivebase");

    //Swerve Drive Motor ID: (Front Left: (1), Front Right: (3), Back Left: (5), Back Right: (7))
    //Swerve Steer Motor ID: (Front Left; (2), Front RIght: (4), Back Left: (6), Back Right: (8))
    //Swerve Encoder ID: [Front Left: (0), Front Right: (1), Back Left: (2), Back Right: (3)
    double FRONT_LEFT_MODULE_STEER_OFFSET= 0.0;
    double FRONT_RIGHT_MODULE_STEER_OFFSET = 0.0;
    double BACK_LEFT_MODULE_STEER_OFFSET = 0.0;
    double BACK_RIGHT_MODULE_STEER_OFFSET = 0.0;
    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
            DRIVEBASE_TAB.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            1,
            // This is the ID of the steer motor
            2,
            // This is the ID of the steer encoder
            0,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
            DRIVEBASE_TAB.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            3,
            4,
            1,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
            DRIVEBASE_TAB.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            5,
            6,
            2,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
            DRIVEBASE_TAB.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            7,
            8,
            3,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    Robot.navX.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {


   if (Robot.navX.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(Robot.navX.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - Robot.navX.getYaw());
  }

  public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
        rotation *= 2.0 / Math.hypot(DRIVETRAIN_WHEELBASE_METERS, DRIVETRAIN_TRACKWIDTH_METERS);
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                    Rotation2d.fromDegrees(Robot.navX.getAngle()));
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
        states[0] = SwerveModuleState.optimize(states[0], getGyroscopeRotation());
        states[1] = SwerveModuleState.optimize(states[1], getGyroscopeRotation());
        states[2] = SwerveModuleState.optimize(states[2], getGyroscopeRotation());
        states[3] = SwerveModuleState.optimize(states[3], getGyroscopeRotation());
        m_frontLeftModule.set(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond, states[3].angle.getRadians());
    }


  @Override
  public void periodic() {
//TODO: Update stuff with sensors
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    SmartDashboard.putNumber("Front Left Module Angle", states[0].angle.getDegrees());
    SmartDashboard.putNumber("Front Right Module Angle", states[1].angle.getDegrees());
    SmartDashboard.putNumber("Back Left Module Angle", states[2].angle.getDegrees());
    SmartDashboard.putNumber("Back Right Module Angle", states[3].angle.getDegrees());

    SmartDashboard.putNumber("Gyroscope Angle", Robot.navX.getAngle());

    m_frontLeftModule.set(states[0].speedMetersPerSecond / voltage, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / voltage, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / voltage, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / voltage, states[3].angle.getRadians());
  }
}