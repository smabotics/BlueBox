package org.usfirst.frc.team5493.robot.subsystems;

import org.usfirst.frc.team5493.robot.RobotMap;
import org.usfirst.frc.team5493.robot.commands.TankDriveWithJoystick;
import org.usfirst.frc.team5493.robot.controllers.IDriveJoystick;
import org.usfirst.frc.team5493.robot.subsystems.utils.EncoderAdjustment;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveBase extends Subsystem implements IDriveBaseSubsystem, IDriveBaseArcadeSubsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private SpeedController motorLeft = new CANTalon(RobotMap.portMotorDriveLeft);
	private SpeedController motorRight = new CANTalon(RobotMap.portMotorDriveRight);

	private Gyro gyro = new AnalogGyro(RobotMap.channelGyroMain);

	private final int IDX_SPEEDCONTROLLER_LEFT = 0;
	private final int IDX_SPEEDCONTROLLER_RIGHT = 1;
	private final int TOTAL_MOTORS = 2;

	private SpeedController[] speedControllers = new SpeedController[2];

	private RobotDrive robotDrive = new RobotDrive(motorLeft, motorRight);

	// private AnalogInput rangefinder = new AnalogInput(6);
	// private AnalogGyro gyro = new AnalogGyro(1);

	private boolean outputEveryXTime = true;
	private double outputEveryXTimeValue = 0.5;

	private double maxSpeed = 0;
	private double autoDistance = 0;
	private double autoWantedTurn = 0;

	private static final double gryoKP = 0.03;

	private static final double KP = 0.1;
	private static final double KI = 0.001;
	private static final double KD = 1;
	private static final double KF = 0.0001;
	private static int izone = 100;
	private static final double ramprate = 36;
	private static final int profile = 0;

	/*
	 * 
	 * Gyro for keeping straight.
	 * 
	 * As to autonomous stuff, seems like more and more teams are going to a
	 * path-planner solution. Before autonomous starts, a set of x/y coordinate
	 * waypoints are fed into the algorithm. An output is generated that indicates
	 * varying desired velocities for each side of the drivetrain over the time
	 * duration of the path traversal.
	 * 
	 * In this case, each side of the drivetrain has an encoder feeding a PID to
	 * control the actual wheel velocity to the desired velocity (from the
	 * path-planner). To augment this, a gyro may be used to provide a
	 * "correction factor", which reduces error from wheel scrub or going over rough
	 * defenses.
	 * 
	 * Net result in 2016 was a ~50% accurate high-goal autonomous routine under the
	 * low bar. Not as good as some of the really slick vision systems, but we're
	 * still pretty proud of it...
	 * 
	 * The addition of a P term to each side of your drivetrain utilizing the gyro
	 * will get you darn close to perfect.
	 * 
	 * P is the proportional, or error term. It's calculated like this:
	 * 
	 * Proportional_Term = Constant * (Desired_Gyro_Angle - Actual_Gyro_Angle)
	 * Left_Drive = Left_Drive + Proportional_Term Right_Drive = Right_Drive -
	 * Proportional_Term
	 * 
	 * 
	 * You may need to swap the +/- in the calculation depending on your drivetrain
	 * direction. Simply increase the constant term until you start driving
	 * straight. Start very small - like .05 or .1 for your Constant.
	 * 
	 * This is the most basic form of PID driving straight.
	 * 
	 */

	private static final double MAX_INTEGRAL_ERROR = 1;
	private double integral = 0.0D;
	private double pasterr;

	// https://www.reddit.com/r/FRC/comments/3zcc2d/timer_not_working/?st=j6l3jsp4&sh=83f36f3a
	private Timer outputTimer = new Timer();

	public DriveBase() {
		super();

		speedControllers[IDX_SPEEDCONTROLLER_LEFT] = motorLeft;
		speedControllers[IDX_SPEEDCONTROLLER_RIGHT] = motorRight;

		gyro.calibrate();
		
		outputTimer.reset();
		outputTimer.start();

		// Let's show everything on the LiveWindow
		LiveWindow.addActuator("Drive Train", "Left Motor", (CANTalon) motorLeft);
		LiveWindow.addActuator("Drive Train", "Right Motor", (CANTalon) motorRight);
		// LiveWindow.addSensor("Drive Train", "Rangefinder", rangefinder);
		// LiveWindow.addSensor("Drive Train", "Gyro", gyro);
	}
	
	private void initTalonsForMagClosed() {
		for (int idx = 0; idx < TOTAL_MOTORS; idx++) {

			CANTalon talon = (CANTalon) speedControllers[idx];
			talon.changeControlMode(TalonControlMode.Position);
			talon.clearStickyFaults();
			talon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		}
	}

	/**
	 * When no other command is running let the operator drive around using the PS3
	 * joystick.
	 */
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new TankDriveWithJoystick(this));
	}

	/**
	 * The log method puts interesting information to the SmartDashboard.
	 */
	public void log() {

		if (this.outputEveryXTime && outputTimer.get() > this.outputEveryXTimeValue) {
			outputTimer.reset();
			for (int idx = 0; idx < TOTAL_MOTORS; idx++) {

				CANTalon talon = (CANTalon) speedControllers[idx];

				// double currentAmps = talon.getOutputCurrent();
				// double outputVolt = talon.getOutputVoltage();
				// double busVolt = talon.getBusVoltage();
				// double quadEncoderPos = talon.getEncPosition();
				// double quadEncoderVelocity = talon.getEncVelocity();
				// double selectedSensorPos = talon.getPosition();
				// double selectedSensorSpeed = talon.getSpeed();
				// int analogPos = talon.getAnalogInPosition();
				// int analogVelocity = talon.getAnalogInPosition();
				// int closeLoopError = talon.getClosedLoopError();

				String contrl = idx == IDX_SPEEDCONTROLLER_LEFT ? "Left " : "Right ";

				SmartDashboard.putNumber(contrl + "Quad Position", talon.getEncPosition());
				SmartDashboard.putNumber(contrl + "Quad Speed", talon.getEncVelocity());

				// DriverStation.reportWarning(contrl + "currentAmps=" + currentAmps, false);
				// DriverStation.reportWarning(contrl + "outputVolt=" + outputVolt, false);
				// DriverStation.reportWarning(contrl + "busVolt=" + busVolt, false);
				// DriverStation.reportWarning(contrl + "quadEncoderPos=" + quadEncoderPos,
				// false);
				// DriverStation.reportWarning(contrl + "quadEncoderVelocity=" +
				// quadEncoderVelocity, false);
				// DriverStation.reportWarning(contrl + "selectedSensorPos=" +
				// selectedSensorPos, false);
				// DriverStation.reportWarning(contrl + "selectedSensorSpeed=" +
				// selectedSensorSpeed, false);
				// DriverStation.reportWarning(contrl + "analogPos=" + analogPos, false);
				// DriverStation.reportWarning(contrl + "analogVelocity=" + analogVelocity,
				// false);
				// DriverStation.reportWarning(contrl + "analogVelocity=" + analogVelocity,
				// false);
				// DriverStation.reportWarning(contrl + "closeLoopError=" + closeLoopError,
				// false);
			}
			outputTimer.start();
		}

		//
		//
		//
		// SmartDashboard.putNumber("Left Distance", leftEncoder.getDistance());
		// SmartDashboard.putNumber("Right Distance", rightEncoder.getDistance());
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

		DriverStation.reportWarning("drive left = " + left + " right = " + right, false);

		robotDrive.tankDrive(left, right);

		DriverStation.reportWarning("drive done", false);

		if (this.outputEveryXTime) {
			log();
		}
	}

	/**
	 * @param joy
	 *            The ps3 style joystick to use to drive tank style.
	 */
	public void drive(IDriveJoystick joy) {

		drive(joy.getTankDriveLeft(), joy.getTankDriveRight());

		if (this.outputEveryXTime) {
			log();
		}
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

		// for (int idx = 0; idx < TOTAL_MOTORS; idx++) {
		//
		// CANTalon talon = (CANTalon) speedControllers[idx];
		//
		// // String contrl = idx == IDX_SPEEDCONTROLLER_LEFT ? "Left " : "Right ";
		//
		//// talon.reset();
		//// talon.enable();
		//// talon.setPosition(0);
		//
		// }
	}

	/**
	 * @return The distance driven (average of left and right encoders).
	 */
	public double getDistance() {
		// return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2;

		double position = 0;

		for (int idx = 0; idx < TOTAL_MOTORS; idx++) {

			CANTalon talon = (CANTalon) speedControllers[idx];

			// String contrl = idx == IDX_SPEEDCONTROLLER_LEFT ? "Left " : "Right ";

			position += talon.getPosition();

		}

		DriverStation.reportWarning("Postion Updated = " + position, false);

		return position / TOTAL_MOTORS;
	}

	@Override
	public void tankDrive(double left, double right) {
		this.robotDrive.tankDrive(left, right);
	}

	@Override
	public boolean doesSupportEncoder() {
		
		return true;
	}

	@Override
	public EncoderAdjustment determineAdjusments() {

		CANTalon talonLeft = (CANTalon) speedControllers[IDX_SPEEDCONTROLLER_LEFT];
		CANTalon talonRight = (CANTalon) speedControllers[IDX_SPEEDCONTROLLER_RIGHT];

		double adjustment = 0;

		int left = Math.abs(talonLeft.getEncPosition());
		int right = Math.abs(talonRight.getEncPosition());

		double delta = Math.abs(left - right) / .01;

		// TODO: MAybe this needs to be proportional (i.e. PID)?
		if (left > right) {
			adjustment = delta;
		} else if (left < right) {
			adjustment = -delta;
		}

		return new EncoderAdjustment(EncoderAdjustment.SIDE_RIGHT, adjustment);
	}

	public double motorPID(double encoderval, double desiredval) {
		double error = desiredval - encoderval;
		if (error > MAX_INTEGRAL_ERROR)
			integral = 0.0D;
		else
			integral += error;
		double derivative = error - pasterr;
		pasterr = error;
		double out = KP * error + KI * integral + KD * derivative;
		out = Math.min(Math.max(out, -1.0D), 1.0D);
		return out;
	}

	@Override
	public void resetEncoders() {

		CANTalon talonLeft = (CANTalon) speedControllers[IDX_SPEEDCONTROLLER_LEFT];
		CANTalon talonRight = (CANTalon) speedControllers[IDX_SPEEDCONTROLLER_RIGHT];

		talonLeft.setEncPosition(0);
		talonRight.setEncPosition(0);
	}

	public void driveArcade(double moveValue) {
		driveArcade(moveValue, false);
	}

	public void driveArcade(double moveValue, boolean squaredInputs) {

		double rotateValue = gyro.getAngle();
		
		rotateValue = rotateValue * gryoKP;

		driveArcade(moveValue, rotateValue, squaredInputs);
	}

	public void driveArcade(double moveValue, double rotateValue, boolean squaredInputs) {
		this.robotDrive.arcadeDrive(moveValue, rotateValue, squaredInputs);
	}

	public void driveArcade(double moveValue, double rotateValue) {
		driveArcade(moveValue, rotateValue, false);
	}

	// /**
	// * @return The distance to the obstacle detected by the rangefinder.
	// */
	// public double getDistanceToObstacle() {
	// // Really meters in simulation since it's a rangefinder...
	// // return rangefinder.getAverageVoltage();
	// }
}
