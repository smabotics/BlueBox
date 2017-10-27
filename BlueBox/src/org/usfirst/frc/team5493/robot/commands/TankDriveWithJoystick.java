/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team5493.robot.commands;

import org.usfirst.frc.team5493.robot.Robot;
import org.usfirst.frc.team5493.robot.subsystems.IDriveBaseSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Have the robot drive tank style using the PS3 Joystick until interrupted.
 */
public class TankDriveWithJoystick extends Command {

	IDriveBaseSubsystem driveBase;
	
	public TankDriveWithJoystick(IDriveBaseSubsystem driveBase) {
		this.driveBase = driveBase;
		requires((Subsystem) driveBase);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		this.driveBase.drive(Robot.oi.getDriveJoystick());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false; // Runs until interrupted
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		this.driveBase.tankDrive(0, 0);
	}
}
