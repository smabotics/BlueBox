package org.usfirst.frc.team5493.robot;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
	
	public static final int PORT_JOYSTICK_DRIVE = 0;
//	public static final int  portMotorDriveLeft = 0;
//	public static final int  portMotorDriveRight = 1;
	public static final int  portMotorDriveLeftFront = 7;
	public static final int  portMotorDriveRightFront = 2;
	public static final int  portMotorDriveLeftBack = 8;
	public static final int  portMotorDriveRightBack = 1;
	public static final int channelGyroMain = 1;
	
	public static final int R_Encoder_A = 2,
			R_Encoder_B = 1,
			L_Encoder_A = 8,
			L_Encoder_B = 7;
	
	
}
