package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
/**
 * Attempts to drive the specified distance without ensuring
 * that the robot is driving perfectly straight.
 * 
 * Instead, the average of both encoders is used to determine the
 * traveled distance
 */
public class DriveDistance extends CommandBase {

	private final double dX;
	private final double dY;
	private final double speed;
	private final double METER_CONVERSION_FACTOR = 0.0254;

	/**
	 * @param xInches The number of inches the robot will drive in the x direction
	 * @param yInches THe number of inches the robot will drivei n the y direction
	 * @param speed  The speed at which the robot will drive (-1 - 1)
	 */
	public DriveDistance(double xInches, double yInches, double speed) {
		this.dX = xInches*METER_CONVERSION_FACTOR;
		this.dY = yInches*METER_CONVERSION_FACTOR;
		this.speed = speed;

		addRequirements(Robot.drivebase);
	}

	@Override
	public void initialize() {
		//Robot.drivebase.resetEncoders();
		Robot.navX.reset();
	}

	@Override
	public void execute() {
		Translation2d newPoint = new Translation2d (dX, dY);
		
		Robot.drivebase.drive(newPoint.times(speed), 0, true);
	}

	/*@Override
	public boolean isFinished() {
		// Check if the robot has moved at least the specified distance
		return Math.abs(Robot.drivebase.getAverageEncoderDistance()) >= distance;
	}*/
}