package org.usfirst.frc.team6002.robot.subsystems;

import org.usfirst.frc.team6002.robot.Constants;
import org.usfirst.frc.team6002.robot.loops.Loop;
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
    static int ELEVATOR_MAX = 30000;
    static int ELEVATOR_MIN = 0;
    static int SENSING_SPEED = 30;
    static int SWITCH_HEIGHT = 10000;
    static int SCALE_HEIGHT = 0;
      
    public enum SystemState {
    	OPEN_LOOP,
    	HOME, //elevator to home position (0)
    	SEEKING,
    	HOLDING
    }

    public enum WantedState {
    	OPEN_LOOP,
    	HOME,
    	SEEK,
    	HOLD
    }
    
    private enum SeekState {
    	SENSING, BOTTOM, OPEN_SLOT,
    }
    private SeekState mSeekState = SeekState.SENSING;
    
    private SystemState mSystemState = SystemState.HOME;
    private WantedState mWantedState = WantedState.HOME;
    
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
                mSystemState = SystemState.HOME;
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
                case SEEKING:
                	newState = handleSeeking();
                	break;
                case HOLDING:
                	newState = handleHolding();
                	break;
                default:
                    newState = SystemState.HOME;
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
    	case HOLD:
    		return SystemState.HOLDING;
    	default:
            return SystemState.SEEKING;  
        }
    }
    private SystemState handleOpenLoop() {
    	//let elevator run to wanted position
    	return defaultStateTransfer();
    }
    
    private SystemState handleHome() {
    	wantedPosition = 0;
    	
    	return defaultStateTransfer();
    }
    
    int wantedPosition = 0;
    private SystemState handleSeeking() {
    	if(mStateChanged) {
    		wantedPosition = mElevatorMaster.getSelectedSensorPosition(0);
    		if(getDistanceInches() < 24) {// found switch already
    			mSeekState = SeekState.OPEN_SLOT;
    			wantedPosition = SWITCH_HEIGHT;
    			return SystemState.OPEN_LOOP;
    		}else {
    			mSeekState = SeekState.SENSING;
//    			wantedPosition = SCALE_HEIGHT;
    		}
    	}
    	if(mSeekState == SeekState.SENSING) {
    		
    		if(getDistanceInches() < 24) {//found bottom
    			mSeekState = SeekState.BOTTOM;
    		}else if(getElevatorPosition() > 30000) {
    			//failed!
    			return SystemState.HOLDING;
    		}else {
    			//keep looking
    			setWantedPositionIncrement(SENSING_SPEED);
    		}
    	}else if (mSeekState == SeekState.BOTTOM) {
    		if(getDistanceInches() > 24) {//found open slot
    			mSeekState = SeekState.OPEN_SLOT;
    			return SystemState.HOLDING;
    		}else if(getElevatorPosition() > 30000) {
    			//failed!
//    			mWantedState = Elevator.WantedState.HOME;
    			return SystemState.HOLDING;
    		}else {
    			//keep looking
    			setWantedPositionIncrement(SENSING_SPEED);
    		}	
    	}

    	switch (mWantedState) {
    	case SEEK:
    		return SystemState.SEEKING;
    	case HOME:
    		return SystemState.HOME;
    	case HOLD:
    		return SystemState.HOLDING;
    	default:
            return SystemState.SEEKING;
    	}
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
    	
    	mSystemState = SystemState.HOME;
        mWantedState = WantedState.HOME;
//    	int absolutePosition = mElevatorMaster.getSensorCollection().getPulseWidthPosition();
    	
    	resetEncoder();
    }

    
    public void setOpenLoop(double joy) {
    	if(Math.abs(joy) < 0.5) {
    		return;
    	}
    	int increment = (int) (joy * 1200);
    	setWantedPosition(increment + wantedPosition);
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

    public void resetEncoder() { //reset elevator's relative position
    	mElevatorMaster.setSelectedSensorPosition(0, Constants.kElevatorPIDLoopId, 0);
    }
    
    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }
    
    private double getDistanceInches() {
    	return ((UltraSonicSensor.getValue() * 5)/25.4)/4; //convert analog voltage to mm then to inches.
    }
    
    public void OutputToSmartDashboard() {
    	SmartDashboard.putNumber("Elevator Master Position", mElevatorMaster.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("Distance Sensor", getDistanceInches());
    	SmartDashboard.putNumber("wanted position", wantedPosition);
    	System.out.println("Elevator State " + mSystemState + " Seek State " + mSeekState);
    }
    
    public void stop() {
    	mWantedState = Elevator.WantedState.HOME;
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

