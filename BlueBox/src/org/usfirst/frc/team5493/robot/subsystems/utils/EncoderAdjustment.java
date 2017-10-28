package org.usfirst.frc.team5493.robot.subsystems.utils;

public class EncoderAdjustment {
	
	public final static byte SIDE_LEFT = 0x0;
	public final static byte SIDE_RIGHT = 0x1;
	public final static byte SIDE_BOTH = 0x2;
	

	public final byte sideToAdjust;
	public final double adjustment;

	public EncoderAdjustment(byte sideToAdjust, double adjustment) {
		this.sideToAdjust = sideToAdjust;
		this.adjustment= adjustment;
	}
}
