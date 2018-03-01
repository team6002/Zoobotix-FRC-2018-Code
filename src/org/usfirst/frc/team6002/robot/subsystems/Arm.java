package org.usfirst.frc.team6002.robot.subsystems;

import org.usfirst.frc.team6002.robot.Constants;
import org.usfirst.frc.team6002.robot.loops.Loop;
import org.usfirst.frc.team6002.robot.subsystems.Elevator.SystemState;
import org.usfirst.frc.team6002.robot.subsystems.Elevator.WantedState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Arm extends Subsystem {

	private static Arm mInstance = new Arm();
	
	public static Arm getInstance() {
        return mInstance;
    }
	
	//Hardware
	private TalonSRX mArm;
	private Solenoid mClaw;
	
	//Variables
	int wantedPosition = 0;
	
	//Positions for arm
	private int ARM_MAX = 2000;
	private int ARM_MIN = 0;
	private int ARM_DEPLOY = 1700;
	
	public enum SystemState {
    	OPEN_LOOP,
    	HOME, //arm to home position (0)
    	DEPLOY, //arm to deploying position (~1300)
    	HOLDING
    }

    public enum WantedState {
    	OPEN_LOOP,
    	HOME,
    	DEPLOY,
    	HOLD
    }
    
    private SystemState mSystemState = SystemState.OPEN_LOOP;
    private WantedState mWantedState = WantedState.OPEN_LOOP;

	private double mCurrentStateStartTime;
    private boolean mStateChanged;
    private double mJoystick;
    
    public Loop getLoop() {
        return mLoop;
    }
    
    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Arm.this) {
            	mSystemState = SystemState.OPEN_LOOP;
                mCurrentStateStartTime = timestamp;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Arm.this) {
            	mArm.set(ControlMode.Position, wantedPosition);
            	
            	
            	SystemState newState;
                switch (mSystemState) {
                case OPEN_LOOP:
                	newState = handleOpenLoop();
                	break;
                case HOME:
                	newState = handleHome();
                	break;
                case DEPLOY:
                	newState = handleDeploy();
                	break;
                case HOLDING:
                	newState = handleHold();
                	break;
                default:
                    newState = SystemState.OPEN_LOOP;
                }
                if (newState != mSystemState) {
                    System.out.println("Elevator state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };
    
    private SystemState defaultStateTransfer() {
    	
    	switch(mWantedState) {
    	case OPEN_LOOP:
    		return SystemState.OPEN_LOOP;
    	case HOLD:
    		return SystemState.HOLDING;
    	case DEPLOY:
    		return SystemState.DEPLOY;
    	case HOME:
    		return SystemState.HOME;
    	default:
    		return SystemState.OPEN_LOOP;
    	}
    }
    
    private SystemState handleOpenLoop() {
    	//let arm run to wanted position based on joystick
    	
    	return defaultStateTransfer();
    }
    
    private SystemState handleHome() {
    	if(mStateChanged) {
    		wantedPosition = mArm.getSelectedSensorPosition(0);
    	}
    	if(mArm.getSelectedSensorPosition(0) <= ARM_MIN+25 && mArm.getSelectedSensorPosition(0) >= ARM_MIN-25) {
    		setWantedPosition(ARM_MIN);
    		mWantedState = WantedState.OPEN_LOOP;
    		return SystemState.OPEN_LOOP;
    	}
    	else {
    		setWantedPositionIncrement(-5);
    	}
    	
    	return defaultStateTransfer();
    }
    
    private SystemState handleDeploy() {
    	if(mStateChanged) {
    		wantedPosition = mArm.getSelectedSensorPosition(0);
    	}
    	if(mArm.getSelectedSensorPosition(0) > ARM_DEPLOY) {
    		setWantedPosition(ARM_DEPLOY);
    		mWantedState = WantedState.OPEN_LOOP;
    		return SystemState.OPEN_LOOP;
    	}else {
    		setWantedPositionIncrement(5);
    	}
    	
    	return defaultStateTransfer();
    }
    
    private SystemState handleHold() {
    	if(mStateChanged) {
    		wantedPosition = mArm.getSelectedSensorPosition(0);
    	}
    	
    	return defaultStateTransfer();
    }
	
    public void deploy() {
    	if(wantedPosition > (ARM_DEPLOY-25) && wantedPosition < (ARM_DEPLOY+25)) {//deadzone
    		setWantedPositionIncrement(10);
    	}else {
    		setWantedPosition(ARM_DEPLOY);
    	}
    }
    
    private Arm() {
		mArm = new TalonSRX(Constants.kArmId);
		mClaw = new Solenoid(Constants.kClawSolenoidId);
		
		mArm.setNeutralMode(NeutralMode.Brake);
		mArm.setInverted(false);
		
		//PID
		mArm.config_kF(Constants.kArmPIDLoopId, Constants.kArmKf, Constants.kTimeoutMs);
		mArm.config_kP(Constants.kArmPIDLoopId, Constants.kArmKp, Constants.kTimeoutMs);
		mArm.config_kI(Constants.kArmPIDLoopId, Constants.kArmKi, Constants.kTimeoutMs);
		mArm.config_kD(Constants.kArmPIDLoopId, Constants.kArmKd, Constants.kTimeoutMs);
		
		/* set the peak and nominal outputs, 12V means full */
		mArm.configNominalOutputForward(0, Constants.kTimeoutMs);
		mArm.configNominalOutputReverse(0, Constants.kTimeoutMs);
		mArm.configPeakOutputForward(1, Constants.kTimeoutMs);
		mArm.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		
		mArm.configAllowableClosedloopError(0, Constants.kArmPIDLoopId, Constants.kTimeoutMs);
		
		mArm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kArmPIDLoopId, Constants.kTimeoutMs);
		
		mWantedState = WantedState.OPEN_LOOP;
		mSystemState = SystemState.OPEN_LOOP;
		resetEncoder();
	}
	
	public void setOpenLoop(double joy) {
		if(Math.abs(joy) < 0.5) {
			return;
		}
		int increment = (int) (joy * 50);
		setWantedPositionIncrement(increment);
	}
	
	public void setWantedPosition(int position) {
    	if(position < ARM_MIN) {
    		position = ARM_MIN;
    	}else if(position > ARM_MAX) {
    		position = ARM_MAX;
    	}
    	wantedPosition = position;
    }
    
    public void setWantedPositionIncrement(int increment) {
    	setWantedPosition(wantedPosition + increment);
    }
	
    //CLAW STUFF
    boolean isClawOpen = false;
    public void open() {
    	mClaw.set(true);
    	isClawOpen = true;
    }
    public void close() {
    	mClaw.set(false);
    	isClawOpen = false;
    }
    public boolean getIsClawOpen() {
    	return isClawOpen;
    }	
    
	private void stop() {
		
	}
	
	public void resetEncoder() {
		mArm.setSelectedSensorPosition(0, Constants.kArmPIDLoopId, Constants.kTimeoutMs);
	}
	
	public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }
	
	public void OutputToSmartDashboard() {
    	SmartDashboard.putNumber("Arm Position", mArm.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("Arm Wanted Position", wantedPosition);
//    	SmartDashboard.putString("Arm SystemState", mSystemState.name());
    	
    }
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

