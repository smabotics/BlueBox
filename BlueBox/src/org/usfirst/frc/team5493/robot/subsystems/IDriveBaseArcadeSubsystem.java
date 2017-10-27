package org.usfirst.frc.team5493.robot.subsystems;

public interface IDriveBaseArcadeSubsystem {

	void driveArcade(double moveValue, double rotateValue);

	void driveArcade(double moveValue, double rotateValue, boolean squaredInputs);

	void driveArcade(double moveValue, boolean squaredInputs);

	void driveArcade(double moveValue);
    

}
