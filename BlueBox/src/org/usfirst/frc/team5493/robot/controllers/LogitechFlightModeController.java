package org.usfirst.frc.team5493.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class LogitechFlightModeController extends Joystick {

	// Mapping
	// http://team358.org/files/programming/ControlSystem2009-/Logitech-F310_ControlMapping.png

	//TODO: Need to update
	
	
	public final int BTN_ONE_RED_B = 1;
	public final int BTN_TWO_BLU_X = 2;
	public final int BTN_ZER_GRN_A = 0;
	public final int BTN_TRE_YEL_Y = 3;
	public final int BTN_FOR_LFT_T = 4;
	public final int BTN_FIV_RGT_T = 5;
	public final int BTN_SIX_BCK = 6;
	public final int BTN_SVN_STRT = 7;
	public final int BTN_EHT_LFT_P = 8;
	public final int BTN_NIN_RGT_P = 9;

	public final int AXS_ZER_LFT_X = 0;
	public final int AXS_ONE_LFT_Y = 1;
	public final int AXS_TWO_LFT_T = 2;
	public final int AXS_TRE_RGT_T = 3;
	public final int AXS_FOR_RGT_X = 4;
	public final int AXS_FIV_RGT_Y = 5;

	public final JoystickButton JoyBtnOneRedB;
	public final JoystickButton JoyBtnTwoBluX;
	public final JoystickButton JoyBtnZerGrnA;
	public final JoystickButton JoyBtnTreYelY;
	public final JoystickButton JoyBtnForLftT;
	public final JoystickButton JoyBtnFivRgtT;
	public final JoystickButton JoyBtnSixBck;
	public final JoystickButton JoyBtnSvnStrt;
	public final JoystickButton JoyBtnEhtLftP;
	public final JoystickButton JoyBtnNinRgtP;

	public LogitechFlightModeController(int port) {
		super(port);
		JoyBtnOneRedB = new JoystickButton(this, BTN_ONE_RED_B);
		JoyBtnTwoBluX = new JoystickButton(this, BTN_TWO_BLU_X);
		JoyBtnZerGrnA = new JoystickButton(this, BTN_ZER_GRN_A);
		JoyBtnTreYelY = new JoystickButton(this, BTN_TRE_YEL_Y);
		JoyBtnForLftT = new JoystickButton(this, BTN_FOR_LFT_T);
		JoyBtnFivRgtT = new JoystickButton(this, BTN_FIV_RGT_T);
		JoyBtnSixBck = new JoystickButton(this, BTN_SIX_BCK);
		JoyBtnSvnStrt = new JoystickButton(this, BTN_SVN_STRT);
		JoyBtnEhtLftP = new JoystickButton(this, BTN_EHT_LFT_P);
		JoyBtnNinRgtP = new JoystickButton(this, BTN_NIN_RGT_P);
	}

}
