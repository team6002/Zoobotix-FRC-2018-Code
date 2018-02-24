package org.usfirst.frc.team6002.robot;

import edu.wpi.first.wpilibj.Joystick;
import org.usfirst.frc.team6002.lib.util.LatchedBoolean;

public class ControlBoard {
	private static ControlBoard mInstance = new ControlBoard();
	private static LatchedBoolean mIntakeEdge = new LatchedBoolean();
	private static LatchedBoolean mIntakeReverseEdge = new LatchedBoolean();
	private static LatchedBoolean mIntakeExtendEdge = new LatchedBoolean();
	private static LatchedBoolean mElevatorEdge = new LatchedBoolean();
	
	public static ControlBoard getInstance(){
		return mInstance;
	}
	
	private Joystick mXbox;
	private Joystick mCoXbox;
	
	private ControlBoard() {
		mXbox = new Joystick(0);
		mCoXbox = new Joystick(1);
	}
	
	//DRIVER CONTROLS
	public double getThrottle(){
		return -mXbox.getRawAxis(1);
	}
	public double getTurn() {
		return mXbox.getRawAxis(4);
	}
	public boolean getQuickTurn(){
		return mXbox.getRawAxis(3) > 0.1;
	}
	public boolean getLowGear(){
		return mXbox.getRawAxis(2) > 0.1;
	}
	//INTAKE
	public boolean getIntake() {
		return mIntakeEdge.update(mXbox.getRawButton(5));
	}
	public boolean getExtendIntake() {
		return mIntakeExtendEdge.update(mXbox.getRawButton(1));
	}
//	public boolean getReverseIntake(){
//		return mIntakeReverseEdge.update(mXbox.getRawButton(3));
//	}
	//ELEVATOR
	public double getElevator() {
		return -mXbox.getRawAxis(5);
	}
	
	public boolean getClaw() {
		return mElevatorEdge.update(mXbox.getRawButton(3));
	}
	
	public boolean getElevatorTestY() {
		return mXbox.getRawButton(4);
	}
	
	public boolean getElevatorTestB() {
		return mXbox.getRawButton(2);
	}
	
	//ARM
	public double getArm() {
		return mXbox.getRawAxis(0);
	}
	
	//CLAW
	public boolean getManualClose(){
		return mXbox.getRawButton(5);
	}

}
