package org.usfirst.frc.team6002.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Elevator extends Subsystem {
	
	private static Elevator sInstance = null;

    public static Elevator getInstance() {
        if (sInstance == null) {
            sInstance = new Elevator();
        }
        return sInstance;
    }
	//Hardware
    private Solenoid mClaw;
    
    //HardwareStatus
    boolean mIsClawOpen;
    public void init() {
    	mClaw = new Solenoid(1);
    	mIsClawOpen = false;
    }
    
    public void openClaw() {
    	mIsClawOpen = true;
    	mClaw.set(true);
    }
    
    public void closeClaw() {
    	mIsClawOpen = false;
    	mClaw.set(false);
    }
    
    public boolean getIsClawOpen() {
    	return mIsClawOpen;
    }
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

