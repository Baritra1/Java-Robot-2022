// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Robot;

public class DriveHold extends CommandBase {

	private final double xMeters;
	private final double yMeters;
	private final double angle;

	public DriveHold(double meters, double angle) {
		this(meters, meters, angle);
	}

	public DriveHold(double xMeters, double yMeters, double angle) {
		this.xMeters = xMeters;
		this.yMeters = yMeters;
		this.angle = angle;
		addRequirements(Robot.drivebase);
	}

	@Override
	public void initialize() {
		//Robot.drivebase.resetEncoders();
	}

	@Override
	public void execute() {
		Translation2d newPoint = new Translation2d (xMeters, yMeters);
		Robot.drivebase.drive(newPoint, angle, false);
	}

}
