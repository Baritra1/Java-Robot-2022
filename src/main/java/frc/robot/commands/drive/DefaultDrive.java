// Copyright (c) 2022 Team 303

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * If no other commands are running, drive based on joystick inputs
 */
public class DefaultDrive extends CommandBase {
	public DefaultDrive() {
		addRequirements(Robot.drivebase);
	}

	@Override
	public void execute() {
		// Drive based on the joystick's y position (forward and back on ours)
		Translation2d newPoint = new Translation2d (-Robot.leftJoystick.getX(), -Robot.rightJoystick.getY());
		Robot.drivebase.drive(newPoint, Robot.rightJoystick.getTwist());
	}
}
