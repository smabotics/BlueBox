package org.usfirst.frc.team5493.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicTbd extends UltrasonicRangeFinder {

	/**
	 * The volts per inch drop.
	 */
	private static final double voltsPerInch = 5.0 / 512.0;

	/**
	 * Creates a new UltraSonic range finder with a volts per inch (voltsPerInch)
	 * 
	 * @param device
	 *            - The AnalogInput of the RangeFinder
	 */
	public UltrasonicTbd(AnalogInput device) {
		super(device, voltsPerInch);
	}
}
