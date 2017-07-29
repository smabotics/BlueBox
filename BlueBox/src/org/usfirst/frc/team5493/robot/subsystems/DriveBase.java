package org.usfirst.frc.team5493.robot.subsystems;

import org.usfirst.frc.team5493.robot.RobotMap;
import org.usfirst.frc.team5493.robot.commands.TankDriveWithJoystick;
import org.usfirst.frc.team5493.robot.controllers.IDriveJoystick;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class DriveBase extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private SpeedController motorLeft = new CANTalon(RobotMap.portMotorDriveLeft);
	private SpeedController motorRight = new CANTalon(RobotMap.portMotorDriveRight);

	private RobotDrive robotDrive = new RobotDrive(motorLeft, motorRight);

	// private Encoder leftEncoder = new Encoder(1, 2);
	// private Encoder rightEncoder = new Encoder(3, 4);
	// private AnalogInput rangefinder = new AnalogInput(6);
	// private AnalogGyro gyro = new AnalogGyro(1);

	public DriveBase() {
		super();

		// Encoders may measure differently in the real world and in
		// simulation. In this example the robot moves 0.042 barleycorns
		// per tick in the real world, but the simulated encoders
		// simulate 360 tick encoders. This if statement allows for the
		// real robot to handle this difference in devices.
		// if (Robot.isReal()) {
		// leftEncoder.setDistancePerPulse(0.042);
		// rightEncoder.setDistancePerPulse(0.042);
		// } else {
		// // Circumference in ft = 4in/12(in/ft)*PI
		// leftEncoder.setDistancePerPulse((4.0 / 12.0 * Math.PI) / 360.0);
		// rightEncoder.setDistancePerPulse((4.0 / 12.0 * Math.PI) / 360.0);
		// }

		// Let's show everything on the LiveWindow
		LiveWindow.addActuator("Drive Train", "Left Motor", (CANTalon) motorLeft);

		LiveWindow.addActuator("Drive Train", "Right Motor", (CANTalon) motorRight);
		// LiveWindow.addSensor("Drive Train", "Left Encoder", leftEncoder);
		// LiveWindow.addSensor("Drive Train", "Right Encoder", rightEncoder);
		// LiveWindow.addSensor("Drive Train", "Rangefinder", rangefinder);
		// LiveWindow.addSensor("Drive Train", "Gyro", gyro);
	}

	/**
	 * When no other command is running let the operator drive around using the
	 * PS3 joystick.
	 */
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new TankDriveWithJoystick());
	}

	/**
	 * The log method puts interesting information to the SmartDashboard.
	 */
	public void log() {
		// SmartDashboard.putNumber("Left Distance", leftEncoder.getDistance());
		// SmartDashboard.putNumber("Right Distance",
		// rightEncoder.getDistance());
		// SmartDashboard.putNumber("Left Speed", leftEncoder.getRate());
		// SmartDashboard.putNumber("Right Speed", rightEncoder.getRate());
		// SmartDashboard.putNumber("Gyro", gyro.getAngle());
	}

	/**
	 * Tank style driving for the DriveTrain.
	 * 
	 * @param left
	 *            Speed in range [-1,1]
	 * @param right
	 *            Speed in range [-1,1]
	 */
	public void drive(double left, double right) {
		robotDrive.tankDrive(left, right);
	}

	/**
	 * @param joy
	 *            The ps3 style joystick to use to drive tank style.
	 */
	public void drive(IDriveJoystick joy) {
		drive(joy.getTankDriveLeft(), joy.getTankDriveRight());
	}

	/**
	 * @return The robots heading in degrees. //
	 */
	// public double getHeading() {
	// return gyro.getAngle();
	// }

	/**
	 * Reset the robots sensors to the zero states.
	 */
	public void reset() {
		// gyro.reset();
		// leftEncoder.reset();
		// rightEncoder.reset();
	}

//	/**
//	 * @return The distance driven (average of left and right encoders).
//	 */
//	public double getDistance() {
//		// return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2;
//	}
//
//	/**
//	 * @return The distance to the obstacle detected by the rangefinder.
//	 */
//	public double getDistanceToObstacle() {
//		// Really meters in simulation since it's a rangefinder...
//		// return rangefinder.getAverageVoltage();
//	}
}
