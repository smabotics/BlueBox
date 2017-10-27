//package org.usfirst.frc.team5493.robot.commands;
//
//import org.usfirst.frc.team5493.robot.Robot;
//
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.command.Command;
//
//public class AutoStraightCmd extends Command {
//
//	int inchDistance = 12;
//	double speed = 0.3;
//
//	double distanceTravelled = 0.0;
//
//	boolean isDone = false;
//
//	public AutoStraightCmd() {
//		requires(Robot.drivetrain);
//	}
//
//	// Called just before this Command runs the first time
//	@Override
//	protected void initialize() {
//		Robot.drivetrain.reset();
//	}
//
//	// Called repeatedly when this Command is scheduled to run
//	@Override
//	protected void execute() {
//
//		DriverStation.reportWarning("in execute", false);
//
//		if (distanceTravelled == 0) {
//			Robot.drivetrain.reset();
//			DriverStation.reportWarning("reset drivetrain", false);
//		}
//		
//		if (Robot.drivetrain.getDistance() < inchDistance) {
//			
//			DriverStation.reportWarning("hasn't driven all the way", false);
//			
//			Robot.drivetrain.drive(speed, speed);
//			distanceTravelled = Robot.drivetrain.getDistance();
//			
//			DriverStation.reportWarning("distanceTravelled " + distanceTravelled, false);
//			
//		} else {
//			isDone = true;
//			DriverStation.reportWarning("set is done true", false);
//		}
//
//		DriverStation.reportWarning("leaving execute", false);
//	}
//
//	// Make this return true when this Command no longer needs to run execute()
//	@Override
//	protected boolean isFinished() {
//		return isDone;
//	}
//
//	// Called once after isFinished returns true
//	@Override
//	protected void end() {
//		isDone = false;
//		distanceTravelled = 0;
//	}
//
//	// Called when another command which requires one or more of the same
//	// subsystems is scheduled to run
//	@Override
//	protected void interrupted() {
//	}
//
//}
