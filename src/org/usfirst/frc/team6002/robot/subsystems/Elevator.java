package org.usfirst.frc.team6002.robot.subsystems;

import org.usfirst.frc.team6002.robot.Constants;
import org.usfirst.frc.team6002.robot.loops.Loop;
import org.usfirst.frc.team6002.robot.subsystems.Arm.Stage;
import org.usfirst.frc.team6002.robot.subsystems.Arm.WantedState;
import org.usfirst.frc.team6002.robot.subsystems.Superstructure.SystemState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *	This subsytem deals with the elevator mechanism.  It has the lift motors and the claw arm.
 *	It's main tasks are acquiring cube, lifting cube, and setting cube on scale or switch.
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
    private TalonSRX mElevatorMaster, mElevatorSlave;
    private AnalogInput UltraSonicSensor; //Maxbotics ultrasonic sensor
    
    //Position Numbers for Talons
   
    
    
    public enum SystemState {
    	OPEN_LOOP,
    	HOME, //elevator to home
    	HOME_PLUS_FOUR, //elevator to home plus 4 inches
    	VAULT, //elevator in position for vault
    	SWITCH, // elevator to switch position
    	LOW_SCALE, //elevator to controlled position of scale
    	MID_SCALE, //elevator to balanced position of scale
    	HIGH_SCALE,//elevator to uncontrolled position of scale
    	DEPLOY, //Elevator to switch or scale based on distance sensor reading
    	SEEKING, //use distance sensor to detect switch and scale
    	CAPTURED, //hold powercube up after intaking
    	HOLDING,
    	IDLE,
    }

    public enum WantedState {
    	OPEN_LOOP,
    	UP,
    	DOWN,
    	HOME,
    	HOME_PLUS_FOUR,
    	VAULT,
    	SWITCH,
    	LOW_SCALE,
    	MID_SCALE,
    	HIGH_SCALE,
    	DEPLOY,
    	SEEK,
    	CAPTURE,
    	HOLD,
    	IDLE
    }
    
    private enum Stage {
    	ONE,
    	TWO,
    	THREE,
    	FOUR,
    	FIVE,
    	SIX,
    	SEVEN,
    	COMPLETE
    }
    
    public enum ElevatorState {
    	MIN,
    	CAPTURED,
    	VAULT,
    	HOME,
    	PLUS_FOUR,
    	SWITCH,
    	LOW_SCALE,
    	MID_SCALE,
    	HIGH_SCALE,
    	MAX,
    	UNKNOWN
    }
    static int ELEVATOR_MAX = 27300;
    public static int ELEVATOR_HOME_PLUS_FOUR = 2500;
    static int ELEVATOR_MIN = 0;
    static int SENSING_SPEED = 30;
    static int SWITCH = 10000;
    static int SEEKING_START_SCALE_HEIGHT = 15000;
    static int CAPTURED_HEIGHT = 4000;
    private int SCALE_CONTROLLED =  20150;//20150;
    private int SCALE_BALANCED = 25000;//22000;
    private int SCALE_UNCONTROLLED = ELEVATOR_MAX;//25000;
    
//    int[] elevatorPositionTicks = {0,0,0,0,5000,10000,20150,25000,27700,27700};
    
    public enum SeekState {
    	SENSING, BOTTOM, OPEN_SLOT,
    }
    
    private SeekState mSeekState = SeekState.SENSING;
    private Stage mStage = Stage.ONE;
    private ElevatorState mElevatorState = ElevatorState.HOME;
    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;
    
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
            synchronized (Elevator.this) {
            	mWantedState = WantedState.OPEN_LOOP;
                mSystemState = SystemState.OPEN_LOOP;
                
                mCurrentStateStartTime = timestamp;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Elevator.this) {
            	mElevatorMaster.set(ControlMode.Position, wantedPosition);
            	
            	SystemState newState;
                switch (mSystemState) {
                case OPEN_LOOP:
                	newState = handleOpenLoop();
                	break;
                case HOME:
                	newState = handleHome();
                	break;
                case HOME_PLUS_FOUR:
                	newState = handleHomePlusFour();
                	break;
                case VAULT:
                	newState = handleVault();
                	break;
                case SWITCH:
                	newState = handleSwitch();
                	break;
                case LOW_SCALE:
                	newState = handleLowScale();
                	break;
                case MID_SCALE:
                	newState = handleMidScale();
                	break;
                case HIGH_SCALE:
                	newState = handleHighScale();
                	break;
                case DEPLOY:
                	newState = handleDeploy();
                	break;
                case SEEKING:
                	newState = handleSeeking();
                	break;
                case CAPTURED:
                	newState = handleCapture();
                	break;
                case HOLDING:
                	newState = handleHolding();
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
    	switch (mWantedState) {
        
    	case OPEN_LOOP:
    		return SystemState.OPEN_LOOP;
    	case SEEK:
    		return SystemState.SEEKING;
    	case HOME:
    		return SystemState.HOME;
    	case HOME_PLUS_FOUR:
    		return SystemState.HOME_PLUS_FOUR;
    	case VAULT:
    		return SystemState.VAULT;
    	case SWITCH:
    		return SystemState.SWITCH;
    	case LOW_SCALE:
    		return SystemState.LOW_SCALE;
    	case MID_SCALE:
    		return SystemState.MID_SCALE;
    	case HIGH_SCALE:
    		return SystemState.HIGH_SCALE;
    	case DEPLOY:
    		return SystemState.DEPLOY;
    	case CAPTURE:
    		return SystemState.CAPTURED;
    	case HOLD:
    		return SystemState.HOLDING;
    	default:
            return SystemState.IDLE;  
        }
    }
    
    private SystemState handleIdle() {
    	if(mStateChanged) {
    		mStage = Stage.ONE;
    	}//elevator rests at the wantedPosition
    	
    	return defaultStateTransfer();
    }
    
    private SystemState handleOpenLoop() {
    	if(mStateChanged) {// set elevator to its position if its already trying to move to a wanted position
    		setWantedPosition(getElevatorPosition());
    		mStage = Stage.ONE;
    	}
    	//let elevator run to wanted position
//    	mElevatorState = ElevatorState.UNKNOWN;
    	return defaultStateTransfer();
    }
    
    private SystemState handleHome() {
    	if(mElevatorState == ElevatorState.HOME) {
    		return defaultStateTransfer();
    	}
    	if(mStateChanged) {
    		mStage = Stage.ONE;
    	}
    	switch(mStage) {
    	case ONE:
    		if(getElevatorPosition() <= 4000) {
    			mStage = Stage.TWO;
    		}else {
    			setWantedPositionIncrement(-125);
    		}
    		break;
    	case TWO:
    		if (getElevatorPosition() <= 100) {
    			wantedPosition = ELEVATOR_MIN;
    			mStage = Stage.COMPLETE;
    		}else {
    			setWantedPositionIncrement(-35);
    		}
    		break;
    	case COMPLETE:
    		mElevatorState = ElevatorState.HOME;
    		mWantedState = WantedState.IDLE;
    		break;
    	}
    	
    	return defaultStateTransfer();
    }
    
    private SystemState handleHomePlusFour() {//move elevator to ~4 inches above home position
    	if(mElevatorState == ElevatorState.PLUS_FOUR) {
    		return defaultStateTransfer();
    	}
    	if(mStateChanged) {
    		mStage = Stage.ONE;
//    		mWantedState = WantedState.HOME;
    	}
    	switch(mStage) {
    	case ONE:
    		if(getElevatorPosition() <= ELEVATOR_HOME_PLUS_FOUR + 1000) {
    			mStage = Stage.TWO;
    		}else {
    			setWantedPositionIncrement(-125);
    		}
    		break;
    	case TWO:
    		if (getElevatorPosition() <= ELEVATOR_HOME_PLUS_FOUR + 100) {
    			wantedPosition = ELEVATOR_HOME_PLUS_FOUR;
    			mStage = Stage.COMPLETE;
    		}else {
    			setWantedPositionIncrement(-25);
    		}
    		break;
    	case COMPLETE:
    		mElevatorState = ElevatorState.PLUS_FOUR;
    		mWantedState = WantedState.IDLE;
    		break;
    	}  	
    	return defaultStateTransfer();
    }
    
    private SystemState handleVault() {
    	if(mStateChanged) {
    		mStage = Stage.ONE;
    	}
    	switch (mStage) {
    	case ONE:
	    	if(getElevatorPosition() <= 500) {
	   			mStage = Stage.TWO;
	   		}else {
	   			setWantedPositionIncrement(-50);
	   		}
	    	break;
    	case TWO:
	   		if (getElevatorPosition() <= 100) {
    			wantedPosition = ELEVATOR_MIN;
    			mStage = Stage.COMPLETE;
    		}else {
    			setWantedPositionIncrement(-10);
    		}
	   		break;
    	case COMPLETE:
    		mElevatorState = ElevatorState.VAULT;
    		mWantedState = WantedState.IDLE;
    		break;
    	}
    	return defaultStateTransfer();
    }
    
    private SystemState handleSwitch() {
    	if(mElevatorState == ElevatorState.SWITCH) {
    		return defaultStateTransfer();
    	}
    	if(mStateChanged) {
    		mStage = Stage.ONE;
    	}
    	switch(mStage) {
    	case ONE:
    		if(getElevatorPosition() >= SWITCH-1000){
    			mStage = Stage.TWO;
    		}else {
    			setWantedPositionIncrement(175);
    		}
    		break;
    	case TWO:
    		if(getElevatorPosition() >= SWITCH-100) {
    			mStage = Stage.COMPLETE;
    		}else {
    			setWantedPositionIncrement(25);
    		}
    		break;
    	case COMPLETE:
    		wantedPosition = SWITCH;
    		mElevatorState = ElevatorState.SWITCH;
    		mWantedState = WantedState.IDLE;
    		break;
    	}
    	
    	return defaultStateTransfer();
    }
    private SystemState handleLowScale() {
    	if(mStateChanged) {
    		
    		mStage = Stage.ONE;
    	}
    	switch (mStage) {
    	case ONE:
    		if(getElevatorPosition() >= SCALE_CONTROLLED-1000) {
    			mStage = Stage.TWO;
    		}else {
    			setWantedPositionIncrement(200);
    		}
    	case TWO:
    		if(getElevatorPosition() >= SCALE_CONTROLLED) {
    			mStage = Stage.COMPLETE;
    		}else {
    			setWantedPositionIncrement(25);
    		}
    	case COMPLETE:
    		wantedPosition = SCALE_CONTROLLED;
    		mElevatorState = ElevatorState.LOW_SCALE;
    		mWantedState = WantedState.IDLE;
    		break;
    	}
    	
    	return defaultStateTransfer();
    }
    private SystemState handleMidScale() {
    	if(mStateChanged) {
    		
    		mStage = Stage.ONE;
    	}
    	switch (mStage) {
    	case ONE:
    		if(getElevatorPosition() >= SCALE_BALANCED-1000) {
    			mStage = Stage.TWO;
    		}else {
    			setWantedPositionIncrement(200);
    		}
    	case TWO:
    		if(getElevatorPosition() >= SCALE_BALANCED) {
    			mStage = Stage.COMPLETE;
    		}else {
    			setWantedPositionIncrement(25);
    		}
    	case COMPLETE:
    		wantedPosition = SCALE_BALANCED;
    		mElevatorState = ElevatorState.MID_SCALE;
    		mWantedState = WantedState.IDLE;
    		break;
    	}
    	
    	return defaultStateTransfer();
    }
    private SystemState handleHighScale() {
    	if(mStateChanged) {
    		
    		mStage = Stage.ONE;
    	}
    	switch (mStage) {
    	case ONE:
    		if(getElevatorPosition() >= SCALE_UNCONTROLLED-1000) {
    			mStage = Stage.TWO;
    		}else {
    			setWantedPositionIncrement(200);
    		}
    	case TWO:
    		if(getElevatorPosition() >= SCALE_UNCONTROLLED) {
    			mStage = Stage.COMPLETE;
    		}else {
    			setWantedPositionIncrement(25);
    		}
    	case COMPLETE:
    		wantedPosition = SCALE_UNCONTROLLED;
    		mElevatorState = ElevatorState.HIGH_SCALE;
    		mWantedState = WantedState.IDLE;
    		break;
    	}
    	
    	return defaultStateTransfer();
    }
    
    private SystemState handleDeploy() {
    	if(mStage == Stage.ONE) {//detect
    		if(getDistanceInches() <= 24) {//go for the switch
    			mStage = Stage.TWO;
    		}else {						//go for the scale
    			mStage = Stage.THREE;
    		}
    	}
    	if(mStage == Stage.TWO) { //switch
    		setWantedPosition(SWITCH);
    		isDone = true;
    	}
    	if(mStage == Stage.THREE) { //elevator at ~4ft (controlled)
    		if(getElevatorPosition() >= SCALE_CONTROLLED-250) {
    			wantedPosition = SCALE_CONTROLLED;
    			if(getDistanceInches() <= 24) {//found something
    				mStage = Stage.FOUR;
    			}else { //open spot for cube to go
    				isDone = true;
    			}
    		}else {
    			setWantedPositionIncrement(200);
    		}
    	}else if(mStage == Stage.FOUR) { //elevator at ~5ft (balanced)
    		if(getElevatorPosition() >= SCALE_BALANCED-250) {
    			wantedPosition = SCALE_BALANCED;
    			if(getDistanceInches() <= 24) {//found something
    				mStage = Stage.FIVE;
    			}else { //open spot for cube to go
    				isDone = true;
    			}
    		}else {
    			setWantedPositionIncrement(200);
    		}
    	}else if(mStage == Stage.FIVE) { //elevator at ~6ft (uncontrolled)
    		if(getElevatorPosition() >= SCALE_UNCONTROLLED-250) {
    			wantedPosition = SCALE_UNCONTROLLED;
    			if(getDistanceInches() <= 24) {//found something
//    				mStage = Stage.SIX;
    			}else { //open spot for cube to go
    				isDone = true;
    			}
    		}else {
    			setWantedPositionIncrement(200);
    		}
    	}
    	
    	return defaultStateTransfer();
    }
    
    int wantedPosition = 0;
    private SystemState handleSeeking() {
    	if(mStateChanged) {
    		wantedPosition = mElevatorMaster.getSelectedSensorPosition(0);
    		if(getDistanceInches() < 24) {// found switch already
    			mSeekState = SeekState.OPEN_SLOT;
    			wantedPosition = SWITCH;
    			mWantedState = WantedState.OPEN_LOOP;
    			return SystemState.OPEN_LOOP;
    		}else {
    			mSeekState = SeekState.SENSING;
    			wantedPosition = SEEKING_START_SCALE_HEIGHT;
    		}
    	}
    	if(mSeekState == SeekState.SENSING) {
    		
    		if(getDistanceInches() < 24) {//found bottom
    			mSeekState = SeekState.BOTTOM;
    		}else if(getElevatorPosition() > 30000) {
    			//failed!
    			mSeekState = SeekState.OPEN_SLOT;
    			mWantedState = WantedState.OPEN_LOOP;
    			return SystemState.OPEN_LOOP;
    		}else {
    			//keep looking
    			setWantedPositionIncrement(SENSING_SPEED);
    		}
    	}else if (mSeekState == SeekState.BOTTOM) {
    		if(getDistanceInches() > 24) {//found open slot
    			mSeekState = SeekState.OPEN_SLOT;
    			mWantedState = WantedState.OPEN_LOOP;
    			return SystemState.OPEN_LOOP;
    		}else if(getElevatorPosition() > 30000) {
    			//failed!
    			mSeekState = SeekState.OPEN_SLOT;
    			mWantedState = WantedState.OPEN_LOOP;
    			return SystemState.OPEN_LOOP;
    		}else {
    			//keep looking
    			setWantedPositionIncrement(SENSING_SPEED);
    		}	
    	}
    	return SystemState.SEEKING;

    }
    private SystemState handleCapture() {
    	if(mElevatorState == ElevatorState.CAPTURED) {
    		return defaultStateTransfer();
    	}
    	if(mStateChanged) {
    		mStage = Stage.ONE;
    	}
    	switch (mStage) {
    	case ONE:
    		if(getElevatorPosition() <= CAPTURED_HEIGHT - 1000) {
    			mStage = Stage.TWO;
    		}else {
    			setWantedPositionIncrement(-30);
    		}
    		break;
    	case TWO:
    		if(getElevatorPosition() <= CAPTURED_HEIGHT - 100) {
    			mStage = Stage.COMPLETE;
    		}else {
    			setWantedPositionIncrement(-10);
    		}
    		break;
    	case COMPLETE:
    		wantedPosition = CAPTURED_HEIGHT;
    		mElevatorState = ElevatorState.CAPTURED;
    		mWantedState = WantedState.IDLE;
    		break;
    	}
    	
    	return defaultStateTransfer();
    }
    
    private SystemState handleHolding() {
    	if(mStateChanged) {
    		wantedPosition = mElevatorMaster.getSelectedSensorPosition(0);
    	}
    	
    	return defaultStateTransfer();
    }
    
    public Elevator() {
    	UltraSonicSensor = new AnalogInput(0);//Maxbotic Sensor
    	
    	mElevatorMaster = new TalonSRX(Constants.kElevatorMasterId); // left side
    	mElevatorSlave = new TalonSRX(Constants.kElevatorSlaveId); // right side

    	mElevatorMaster.set(ControlMode.Position, 0);
    	mElevatorSlave.set(ControlMode.Follower, Constants.kElevatorMasterId);
    	
    	mElevatorMaster.setNeutralMode(NeutralMode.Brake);
    	mElevatorSlave.setNeutralMode(NeutralMode.Brake);
    	
    	mElevatorMaster.setInverted(true);
    	mElevatorSlave.setInverted(false);
    	
    	mElevatorMaster.setSensorPhase(true);
		
    	/* set closed loop gains for elevator, typically kF stays zero. */
		mElevatorMaster.config_kF(Constants.kElevatorPIDLoopId, Constants.kElevatorKf, Constants.kTimeoutMs);
		mElevatorMaster.config_kP(Constants.kElevatorPIDLoopId, Constants.kElevatorKp, Constants.kTimeoutMs);
		mElevatorMaster.config_kI(Constants.kElevatorPIDLoopId, Constants.kElevatorKi, Constants.kTimeoutMs);
		mElevatorMaster.config_kD(Constants.kElevatorPIDLoopId, Constants.kElevatorKd, Constants.kTimeoutMs);
		
		/* set the peak and nominal outputs, 12V means full */
		mElevatorMaster.configNominalOutputForward(0, Constants.kTimeoutMs);
		mElevatorMaster.configNominalOutputReverse(0, Constants.kTimeoutMs);
		mElevatorMaster.configPeakOutputForward(1, Constants.kTimeoutMs);
		mElevatorMaster.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		mElevatorMaster.configAllowableClosedloopError(0, Constants.kElevatorPIDLoopId, Constants.kTimeoutMs);
    	mElevatorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kElevatorPIDLoopId, Constants.kTimeoutMs);
    	mElevatorSlave.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kElevatorPIDLoopId, Constants.kTimeoutMs);
    	
    	mSystemState = SystemState.OPEN_LOOP;
        mWantedState = WantedState.OPEN_LOOP;
//    	int absolutePosition = mElevatorMaster.getSensorCollection().getPulseWidthPosition();
    	
    	resetEncoder();
    }

    
    public void setOpenLoop(double joy) {
    	if(Math.abs(joy) < 0.5) {
    		return;
    	}
    	int increment = (int) (joy * 25);//700
    	setWantedPosition(increment + wantedPosition);
//    	mElevatorState = ElevatorState.UNKNOWN;
    	mWantedState = WantedState.OPEN_LOOP;
    }
    
    public void setWantedPosition(int position) {
    	if(position < ELEVATOR_MIN) {
    		position = ELEVATOR_MIN;
    	}else if(position > ELEVATOR_MAX) {
    		position = ELEVATOR_MAX;
    	}
    	wantedPosition = position;
    }
    
    public void setWantedPositionIncrement(int increment) {
    	setWantedPosition(wantedPosition + increment);
    }

    public int getElevatorPosition() {
    	return mElevatorMaster.getSelectedSensorPosition(0);
    }
    
    public SeekState getSeekState() {
    	return mSeekState;
    }

    public void resetEncoder() { //reset elevator's relative position
    	mElevatorMaster.setSelectedSensorPosition(0, Constants.kElevatorPIDLoopId, 0);
    }
    
    public void resetState() { //resets states of robot to starting config
    	mSeekState = SeekState.SENSING;
        mStage = Stage.ONE;
        mElevatorState = ElevatorState.HOME;
        mSystemState = SystemState.IDLE;
        mWantedState = WantedState.IDLE;
    }
    
    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }
    
    public synchronized void setSeekState(SeekState state) {
    	mSeekState = state;
    }
    
    public ElevatorState getElevatorState() {
    	return mElevatorState;
    }
    
    boolean isDone = false;
    public boolean isDone() {
    	return isDone();
    }
    public void setIsDone(boolean done) {
    	isDone = done;
    }
    
    private double getDistanceInches() {
    	return ((UltraSonicSensor.getValue() * 5)/25.4)/4; //convert analog voltage to mm then to inches.
    }
    
    public boolean isComplete() {
		if(mStage == Stage.COMPLETE) {
			return true;
		}else {
			return false;
		}
	}
    
    public void OutputToSmartDashboard() {
    	SmartDashboard.putNumber("Elevator Master Position", mElevatorMaster.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("Distance Sensor", getDistanceInches());
    	SmartDashboard.putNumber("wanted position", wantedPosition);
    	SmartDashboard.putString("Elevator SystemState", mSystemState.name());
    	SmartDashboard.putString("Elevator Stage", mStage.name());
    	SmartDashboard.putString("Elevator State", mElevatorState.name());
//    	SmartDashboard.putString("Seek State", mSystemState.name());
//    	System.out.println("Elevator State " + mSystemState + " Seek State " + mSeekState);
    }
    
    public void stop() {
    	
    }
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
	//Conversions for encoders
	private static double inchesToTicks(double inches) { //elevator spool travels about 1043.5669 ticks to one inch.
		return (4096 / (Constants.kElevatorDiameterInches * Math.PI)) * inches;
	}
    private static double inchesToRotations(double inches) {
        return inches / (Constants.kElevatorDiameterInches * Math.PI);
    }
    
    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kElevatorDiameterInches * Math.PI);
    }
    
}

