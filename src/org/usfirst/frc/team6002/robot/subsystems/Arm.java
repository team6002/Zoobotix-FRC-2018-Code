package org.usfirst.frc.team6002.robot.subsystems;

import org.usfirst.frc.team6002.robot.Constants;
import org.usfirst.frc.team6002.robot.loops.Loop;
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
	private Solenoid mClaw, mPiston;
	
	
	//Variables
	int wantedPosition = 0;
	
	//Positions for arm
	private int ARM_MAX = 2000;
	private int ARM_MIN = 0;
	private int ARM_DEPLOY = 1500;
	private int ARM_CAPTURE = 1000;
	
	public enum Stage {
    	ONE,
    	TWO,
    	THREE,
    	COMPLETE
    }
    private Stage mStage = Stage.ONE;
	
	public enum SystemState {
    	OPEN_LOOP,
    	HOME, //arm to home position (0)
    	CAPTURED, //arm straight up, claw closed
    	VAULT, //arm to 180 degrees behind robot for vault.
    	DEPLOY, //arm to deploying position 
    	DEPLOY_LOW, //arm to low deploy, straight in front of robot
    	DEPLOY_BACK, //arm to back deploy, in the home position (0)
    	HOLDING,
    	IDLE,
    }

    public enum WantedState {
    	OPEN_LOOP,
    	HOME,
    	CAPTURE,
    	VAULT,
    	DEPLOY,
    	DEPLOY_LOW,
    	DEPLOY_BACK,
    	HOLD,
    	IDLE
    }
    
    public enum ArmState{
    	HOME,
    	CAPTURED,
    	DEPLOYED,
    	DEPLOYED_LOW,
    	DEPLOYED_BACK,
    	VAULT,
    	UNKNOWN,
    }
    
    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;
    private ArmState mArmState = ArmState.HOME;

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
            	mSystemState = SystemState.IDLE;
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
                	newState = handleIdle();//handleOpenLoop()
                	break;
                case HOME:
                	newState = handleHome();
                	break;
                case CAPTURED:
                	newState = handleCapture();
                	break;
                case VAULT:
                	newState = handleVault();
                	break;
                case DEPLOY:
                	newState = handleDeploy();
                	break;
                case DEPLOY_LOW:
                	newState = handleDeployLow();
                	break;
                case DEPLOY_BACK:
                	newState = handleDeployBack();
                	break;
                case HOLDING:
                	newState = handleHold();
                	break;
                case IDLE:
                	newState = handleIdle();
                	break;
                default:
                    newState = SystemState.IDLE;
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
    		return SystemState.IDLE;
    	case CAPTURE:
    		return SystemState.CAPTURED;
    	case HOLD:
    		return SystemState.HOLDING;
    	case VAULT:
    		return SystemState.VAULT;
    	case DEPLOY:
    		return SystemState.DEPLOY;
    	case DEPLOY_LOW:
    		return SystemState.DEPLOY_LOW;
    	case DEPLOY_BACK:
    		return SystemState.DEPLOY_BACK;
    	case HOME:
    		return SystemState.HOME;
    	case IDLE:
    		return SystemState.IDLE;
    	default:
    		return SystemState.IDLE;
    	}
    }
    
    private SystemState handleIdle() {
    	if(mStateChanged) {
    		//let elevator rest at wanted position
    	}
    	
    	return defaultStateTransfer();
    }
    
//    private SystemState handleOpenLoop() {
//    	if(mStateChanged) {
//    		setWantedPosition(mArm.getSelectedSensorPosition(0));
//    		mArmState = ArmState.UNKNOWN; // don't know where arm is exactly
//    	}
//    	//let arm run to wanted position based on joystick
//    	
//    	return defaultStateTransfer();
//    }
    
    private SystemState handleHome() {
    	if(mArmState == ArmState.HOME) {
    		return defaultStateTransfer();
    	}
    	if(mStateChanged) {
    		mStage = Stage.ONE;
    	}
    	switch(mStage) {
    	case ONE:
    		if(getArmPosition() <= ARM_MIN + 10) {//if arm is close to 0, set to 0
    			setWantedPosition(ARM_MIN);
    			mStage = Stage.COMPLETE;
    		}
    		else {//else, slowly move arm towards 0    			
    			setWantedPositionIncrement(-10);
    		}
    		break;
    	case COMPLETE:
    		mArmState = ArmState.HOME;
    		mWantedState = WantedState.IDLE;
    		break;
    	}
    	return defaultStateTransfer();
    }
    
    private SystemState handleCapture() {
    	if(mStateChanged) {
    		wantedPosition = mArm.getSelectedSensorPosition(0);
    		mStage = Stage.ONE;
    	}
    	switch(mStage) {
    	case ONE:
    		if(getArmPosition() >= ARM_CAPTURE + 10) {//if arm is above capture position, move it down
    			setWantedPositionIncrement(-5);
    		}else if(getArmPosition() <= ARM_CAPTURE - 10) {//if arm is below capture position, move it up
    			setWantedPositionIncrement(5);
    		}else {// if arm is within 20 encoder ticks of target, complete
    			mStage = Stage.COMPLETE;
    		}
    		break;
    	case COMPLETE:
    		setWantedPosition(ARM_CAPTURE);//hold position.
    		mArmState = ArmState.CAPTURED;
    		mWantedState = WantedState.IDLE;
    		break;
    	}
    	
    	return defaultStateTransfer();
    }
    
    private SystemState handleVault() {
    	if(mArmState == ArmState.VAULT) {
    		return defaultStateTransfer();
    	}
    	if(mStateChanged) {
    		mStage = Stage.ONE;
    	}
    	
    	switch(mStage) {
    	case ONE:
    		if(mArm.getSelectedSensorPosition(0) >= ARM_MAX - 1000) {
    			mStage = Stage.TWO;
    		}else {
    			setWantedPositionIncrement(20);
    		}
    		break;
    	case TWO:
    		if(mArm.getSelectedSensorPosition(0) >= ARM_MAX - 100) {
    			mStage = Stage.COMPLETE;
    		}else {
    			setWantedPositionIncrement(10);
    		}
    		break;
    	case COMPLETE:
    		setWantedPosition(ARM_MAX);
    		mArmState = ArmState.VAULT;
    		mWantedState = WantedState.IDLE;
    		break;
    	}
    	
    	return defaultStateTransfer();
    }
    private SystemState handleDeploy() {
    	if(mArmState == ArmState.DEPLOYED) {
    		return defaultStateTransfer();
    	}
    	if(mStateChanged) {
    		mStage = Stage.ONE;
    		wantedPosition = getArmPosition();
    	}
    	switch (mStage) {
    	case ONE:
	    	if(getArmPosition() <= ARM_DEPLOY-100) {// if arm is below position, move up
	    		setWantedPositionIncrement(5);
	    	}else if(getArmPosition() >= ARM_DEPLOY + 100) {//if arm is above position, move down
	    		setWantedPositionIncrement(-5);
	    	}
	    	else {//else, both conditions met, so complete
	    		mStage = Stage.COMPLETE;
	    	}
	    	break;
    	case COMPLETE:
	    		setWantedPosition(ARM_DEPLOY);//hold position
	    		mArmState = ArmState.DEPLOYED;
	    		mWantedState = WantedState.IDLE;
	    		break;

    	}
    	return defaultStateTransfer();
    }
    private SystemState handleDeployLow() {
    	if(mArmState == ArmState.DEPLOYED_LOW) {
    		return defaultStateTransfer();
    	}
    	if(mStateChanged) {
    		mStage = Stage.ONE;
    		wantedPosition = getArmPosition();
    	}
    	switch (mStage) {
    	case ONE:
	    	if(getArmPosition() <= ARM_MAX-100) {// if arm is below position, move up
	    		setWantedPositionIncrement(5);
	    	}else if(getArmPosition() >= ARM_MAX + 100) {//if arm is above position, move down
	    		setWantedPositionIncrement(-5);
	    	}
	    	else {//else, both conditions met, so complete
	    		mStage = Stage.COMPLETE;
	    	}
	    	break;
    	case COMPLETE:
	    		setWantedPosition(ARM_MAX);//hold position
	    		mArmState = ArmState.DEPLOYED;
	    		mWantedState = WantedState.IDLE;
	    		break;

    	}
    	return defaultStateTransfer();
    }
    private SystemState handleDeployBack() {
    	if(mArmState == ArmState.DEPLOYED_BACK) {
    		return defaultStateTransfer();
    	}
    	if(mStateChanged) {
    		mStage = Stage.ONE;
    		wantedPosition = getArmPosition();
    	}
    	switch (mStage) {
    	case ONE:
	    	if(getArmPosition() <= ARM_MIN-100) {// if arm is below position, move up
	    		setWantedPositionIncrement(5);
	    	}else if(getArmPosition() >= ARM_MIN + 100) {//if arm is above position, move down
	    		setWantedPositionIncrement(-5);
	    	}
	    	else {//else, both conditions met, so complete
	    		mStage = Stage.COMPLETE;
	    	}
	    	break;
    	case COMPLETE:
	    		setWantedPosition(ARM_MIN);//hold position
	    		mArmState = ArmState.DEPLOYED_BACK;
	    		mWantedState = WantedState.IDLE;
	    		break;

    	}
    	return defaultStateTransfer();
    }
    
    private SystemState handleHold() {
    	if(mStateChanged) {
    		wantedPosition = mArm.getSelectedSensorPosition(0);
    	}
    	
    	return defaultStateTransfer();
    }

//    public void deploy() {
//    	if(wantedPosition > (ARM_DEPLOY-25) && wantedPosition < (ARM_DEPLOY+25)) {//deadzone
//    		setWantedPositionIncrement(10);
//    	}
//    	else {
//    		setWantedPosition(ARM_DEPLOY);
//    	}
//    }
    
    private Arm() {
		mArm = new TalonSRX(Constants.kArmId);
		mClaw = new Solenoid(Constants.kClawSolenoidId);
		mPiston = new Solenoid(Constants.kPistonSolenoidId);
		
		
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
		
		mWantedState = WantedState.IDLE;
		mSystemState = SystemState.IDLE;
		mArmState = ArmState.HOME;
		clawClose(true);
		resetEncoder();
	}
	
	public void setOpenLoop(double joy) {
		if(Math.abs(joy) < 0.5) {
			return;
		}
		int increment = (int) (joy * 10);
		setWantedPositionIncrement(increment);
		mArmState = ArmState.UNKNOWN;
		mWantedState = WantedState.OPEN_LOOP;
	}
	
	public void setWantedPosition(int position) {
    	if(position < ARM_MIN) {
    		position = ARM_MIN;
    	}else if(position > ARM_MAX) {
    		position = ARM_MAX;
    	}
    	wantedPosition = position;
    }
    
    public void setWantedPositionIncrement(double increment) {
    	setWantedPosition(wantedPosition + (int)increment);
    }
    
    public int getArmPosition() {
    	return mArm.getSelectedSensorPosition(0);
    }
	
    //CLAW STUFF
    boolean isClawOpen = false;
    public void clawClose(boolean on) {
    	mClaw.set(!on); //reverse boolean, claw is reversed
    }
    public void open() {
    	clawClose(true);
    	isClawOpen = true;
    }
    public void close() {
    	clawClose(false);
    	isClawOpen = false;
    }
    public boolean getIsClawOpen() {
    	return isClawOpen;
    }
    
    boolean isPistonExtended = false;
    public boolean getIsPistonExtended() {
    	return isPistonExtended;
    }
    public void extendPiston() {
    	mPiston.set(true);
    	isPistonExtended = true;
    }
    public void retractPiston() {
    	mPiston.set(false);
    	isPistonExtended = false;
    }
    
	private void stop() {
		
	}
	public ArmState getArmState() {
		return mArmState;
	}
	
	boolean isDone = false;
	public boolean isDone() {
		return isDone();
	}
	public void setIsDone(boolean done) {
		isDone = done;
	}
	
	public void setArmPosition(int position) {
		mArm.setSelectedSensorPosition(position, Constants.kArmPIDLoopId, Constants.kTimeoutMs);
	}
	 
	public void resetEncoder() {
		mArm.setSelectedSensorPosition(0, Constants.kArmPIDLoopId, Constants.kTimeoutMs);
	}
	
	public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }
	
	public boolean isComplete() {
		if(mStage == Stage.COMPLETE) {
			return true;
		}else {
			return false;
		}
	}
	
	public void resetState() { //resets states of robot to starting config
		mSystemState = SystemState.IDLE;
	    mWantedState = WantedState.IDLE;
	    mArmState = ArmState.HOME;
    }
	
	public void OutputToSmartDashboard() {
    	SmartDashboard.putNumber("Arm Position", mArm.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("Arm Wanted Position", wantedPosition);
    	SmartDashboard.putString("Arm State", mArmState.name());
    	
    	SmartDashboard.putString("Arm SystemState", mSystemState.name());
    	
    }
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

