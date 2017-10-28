package org.usfirst.frc.team5493.robot.subsystems;

import org.usfirst.frc.team5493.robot.controllers.IDriveJoystick;
import org.usfirst.frc.team5493.robot.subsystems.utils.EncoderAdjustment;

public interface IDriveBaseSubsystem {

    public void tankDrive(double left, double right);
    
    public void drive(IDriveJoystick joystick);
    
    public boolean doesSupportEncoder();
    
    public EncoderAdjustment determineAdjusments(double distanceToSetpoint);
    
    public void resetEncoders();


}
