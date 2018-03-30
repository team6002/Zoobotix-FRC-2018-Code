package org.usfirst.frc.team6002.robot;

import edu.wpi.first.wpilibj.Joystick;
import org.usfirst.frc.team6002.lib.util.LatchedBoolean;

public class ControlBoard {
	private static ControlBoard mInstance = new ControlBoard();
	private static LatchedBoolean mIntakeEdge = new LatchedBoolean();
	private static LatchedBoolean mIntakeReverseEdge = new LatchedBoolean();
	private static LatchedBoolean mIntakeExtendEdge = new LatchedBoolean();
	private static LatchedBoolean mIntakeStopEdge = new LatchedBoolean();
	private static LatchedBoolean mElevatorEdge = new LatchedBoolean();
	private static LatchedBoolean mDeployEdge = new LatchedBoolean();
	private static LatchedBoolean mShootEdge = new LatchedBoolean();
	private static LatchedBoolean mStateUpEdge = new LatchedBoolean();
	private static LatchedBoolean mStateDownEdge = new LatchedBoolean();
	private static LatchedBoolean mHomeEdge = new LatchedBoolean();
	private static LatchedBoolean mTestEdge = new LatchedBoolean();
	
	
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
		return -mXbox.getRawAxis(4);
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
//	public boolean getExtendIntake() {
//		return mIntakeExtendEdge.update(mXbox.getRawButton(3));
//	}
	public boolean getDeployIntake() {
		return mDeployEdge.update(mXbox.getRawButton(7));
	}
	public boolean getReverseIntake(){
		return mIntakeReverseEdge.update(mXbox.getRawButton(1));
	}
	//Elevator controls
	public boolean getStateUp() {
		return mStateUpEdge.update(mXbox.getRawButton(4));
	}
	public boolean getStateDown() {
		return mStateDownEdge.update(mXbox.getRawButton(3));
	}
	public boolean getHome() {
		return mHomeEdge.update(mXbox.getRawButton(8));
	}
	public boolean getDeployCube() {
		return mElevatorEdge.update(mXbox.getRawButton(6));
	}
	public boolean getShootCube() {
		return mShootEdge.update(mXbox.getRawButton(2));
	}
	
	//Test Buttons(careful not a latched boolean)
	public boolean getTestY() {
		return mXbox.getRawButton(4);
	}
	
	public boolean getTestB() {
		return mTestEdge.update(mXbox.getRawButton(2));
	}
	
	public boolean getTestX() {
		return mXbox.getRawButton(3);
	}
	public boolean getTestA() {
		return mTestEdge.update(mXbox.getRawButton(1));
	}
	
	// manual elevator control
	public double getElevator() {
		return -mXbox.getRawAxis(5);
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
