package frc.robot;

import java.util.List;
import java.io.IOException;
import java.nio.file.Path;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap.IOConstants;
import frc.robot.RobotMap.LED;
//import frc.robot.autonomous.Autonomous;
import frc.robot.autonomous.AutonomousProgram;
import frc.robot.commands.climber.DefaultClimb;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.DriveHold;
import frc.robot.commands.drive.DriveWait;
import frc.robot.commands.drive.SetDriveSpeed;
import frc.robot.commands.led.LEDRainbowRotate;
import frc.robot.commands.led.LEDSolidColor;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {

	/* Define Robot Subsystems */
	public static final SwerveSubsystem drivebase = new SwerveSubsystem();
	public static final LEDSubsystem ledStrip = new LEDSubsystem();
	public static final ClimberSubsystem climb = new ClimberSubsystem();


	/* RoboRio Sensors */
	public static final AHRS navX = new AHRS();
	public static final SwerveDriveOdometry odometer = new SwerveDriveOdometry(SwerveSubsystem.m_kinematics,
	new Rotation2d(0));
	/* Robot IO Controls */
	public static final Joystick leftJoystick = new Joystick(IOConstants.LEFT_JOYSTICK_ID);
	public static final Joystick rightJoystick = new Joystick(IOConstants.RIGHT_JOYSTICK_ID);
	public static final XboxController operatorController = new XboxController(IOConstants.OPERATOR_CONTROLLER);

	/* Shufflebaord Tabs */
	public static final ShuffleboardTab AUTO_TAB = Shuffleboard.getTab("Autonomous");

	/* Shuffleboard Choosers */
	public static SendableChooser<Double> autoDelayChooser = new SendableChooser<>();

	/* Robot alliance color */
	public static Color allianceColor = DriverStation.getAlliance() == Alliance.Blue ? LED.RED : LED.BLUE;

	/**
	 * Defines all the options for the autonomous delay
	 */
	static {
		for (double i = 0; i < 15; i += 0.25)
			autoDelayChooser.addOption(String.format("%.2f", i), i);

		autoDelayChooser.setDefaultOption("0.0", 0.0D);

		AUTO_TAB.add("Auto Start Delay", autoDelayChooser);
	}

	// The command configured to run during auto
	private Command autonomousCommand;
	public Command getAutonomousCommand() {
		//TODO: Measure actual values of acceleration
        final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
		final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI/4;

		final double kPX_CONTROLLER = 0.0;
		final double kPY_CONTROLLER = 0.0;
		final double kPTHETA_CONTROLLER = 0.0;
		final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        SwerveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                SwerveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                        .setKinematics(SwerveSubsystem.m_kinematics);

        // 2. Generate trajectory
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         List.of(
        //                 new Translation2d(1, 0),
        //                 new Translation2d(1, -1)),
        //      	   new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        //         trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(kPX_CONTROLLER, 0, 0);
        PIDController yController = new PIDController(kPY_CONTROLLER, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                kPTHETA_CONTROLLER, 0, 0, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                SwerveSubsystem::getPose,
                SwerveSubsystem.m_kinematics,
                xController,
                yController,
                thetaController,
                drivebase::fieldOrientedDrive,
                drivebase);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> SwerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> SwerveSubsystem.stopModules()));
    }
	String trajectoryJSON = "paths/Unnamed.wpilib.json";
	Trajectory trajectory = new Trajectory();

	@Override
	public void robotInit() {
		// Configure the joystick and controller bindings
		configureButtonBindings();

		// Reset everything back to default
		navX.calibrate();
		navX.reset();

		// Reset motor encoders for all sub systems
	//	drivebase.resetEncoders(); Using Hall effect sensors now
		climb.resetEncoders();

		// This runs if no other commands are scheduled (teleop)
		drivebase.setDefaultCommand(new DefaultDrive());
		climb.setDefaultCommand(new DefaultClimb());
		ledStrip.setDefaultCommand(new LEDRainbowRotate());

		// Set limited drive speed for normal driving
		drivebase.setMaxOutput(0.75);

		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		 } catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
		}

		/* Shuffleboard Stuff */
		//Autonomous.init();
		AutonomousProgram.addAutosToShuffleboard();

		// Start Camera
		CameraServer.startAutomaticCapture();
	}

	@Override
	public void autonomousInit() {
		// Chooses which auto we do from SmartDashboard
		autonomousCommand = getAutonomousCommand();

		// Schedule the selected autonomous command group
		if (autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(
					// To achieve the configured delay, use a sequential group that contains a wait
					// command
					new SequentialCommandGroup(
							new DriveWait(autoDelayChooser.getSelected()),
							autonomousCommand));
		}

		// Match LEDs color to team
		CommandScheduler.getInstance().schedule(new LEDSolidColor(allianceColor));
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running.
		if (autonomousCommand != null)
			autonomousCommand.cancel();

		// Override whatever the brake switch is
		climb.setNeutralMode(IdleMode.kBrake);

		// Match LEDs color to team
		CommandScheduler.getInstance().schedule(new LEDSolidColor(allianceColor));
	}

	@Override
	public void disabledInit() {
		// Change LED color to signify disabled state
		CommandScheduler.getInstance().schedule(new LEDSolidColor(LED.DISABLED_COLOR));
	}

	@Override
	public void disabledPeriodic() {
		// Poll the brake switch to correctly set climber nuetral mode
		climb.updateBrakeSwitch();
	}

	private void configureButtonBindings() {
		// While holding turbo button, increase drive speed to full power
		new JoystickButton(rightJoystick, 2).whileHeld(new SetDriveSpeed(1));

		// Drive fwd
		new JoystickButton(rightJoystick, 3).whileHeld(new DriveHold(0.75,0,Robot.navX.getAngle()));
		new JoystickButton(rightJoystick, 5).whileHeld(new DriveHold(1,0,Robot.navX.getAngle()));

		// Drive back
		new JoystickButton(rightJoystick, 6).whileHeld(new DriveHold(-1,0,Robot.navX.getAngle()));
		new JoystickButton(rightJoystick, 4).whileHeld(new DriveHold(-0.75,0,Robot.navX.getAngle()));

		// Rotation counter-clockwise
		new JoystickButton(leftJoystick, 3).whileHeld(new DriveHold(0,0, Robot.navX.getAngle()-90));

		// Rotation clockwise
		new JoystickButton(leftJoystick, 3).whileHeld(new DriveHold(0,0, Robot.navX.getAngle()+90));

	}

	/*
	 * This Robot is configured to run with the WPILib CommandScheduler.
	 * ⛔ Nothing should be handled in the below methods ⛔
	 */

	@Override
	public void robotPeriodic() {
		/*
		 * Runs the Scheduler. This is responsible for polling buttons, adding
		 * newly-scheduled
		 * commands, running already-scheduled commands, removing finished or
		 * interrupted commands,
		 * and running subsystem periodic() methods. This must be called from the
		 * robot's periodic
		 * block in order for anything in the Command-based framework to work.
		 */
		CommandScheduler.getInstance().run();
	}
}
