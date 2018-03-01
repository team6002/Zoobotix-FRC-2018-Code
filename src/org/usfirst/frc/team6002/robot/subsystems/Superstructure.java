package org.usfirst.frc.team6002.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team6002.robot.Constants;
import org.usfirst.frc.team6002.robot.Robot;
import org.usfirst.frc.team6002.robot.RobotState;
//import org.usfirst.frc.team6002.robot.ShooterAimingParameters;
import org.usfirst.frc.team6002.robot.loops.Loop;
import org.usfirst.frc.team6002.robot.loops.Looper;
import org.usfirst.frc.team6002.robot.subsystems.Elevator.SeekState;
import org.usfirst.frc.team6002.robot.subsystems.Elevator.SystemState;

import java.util.Optional;

/**
 * The superstructure subsystem is the overarching superclass containing all components of the superstructure: the
 * intake, hopper, feeder, shooter and LEDs. The superstructure subsystem also contains some miscellaneous hardware that
 * is located in the superstructure but isn't part of any other subsystems like the compressor, pressure sensor, and
 * hopper wall pistons.
 * 
 * Instead of interacting with subsystems like the feeder and intake directly, the {@link Robot} class interacts with
 * the superstructure, which passes on the commands to the correct subsystem.
 * 
 * The superstructure also coordinates actions between different subsystems like the feeder and shooter.
 * 
 * @see Intake
 * @see Hopper
 * @see Feeder
 * @see Shooter
 * @see LED
 * @see Subsystem
 */
public class Superstructure extends Subsystem {

    static Superstructure mInstance = null;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

//    private final Feeder mFeeder = Feeder.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Elevator mElevator = Elevator.getInstance();
    private final Arm mArm = Arm.getInstance();
    
    private double mCurrentStateStartTime;
    private boolean mStateChanged;
    
    // Superstructure doesn't own the drive, but needs to access it
    private final Drive mDrive = Drive.getInstance();

    // Intenal state of the system
    public enum SystemState {
        IDLE,
        PREPARING_TO_DEPLOY,
        HOME,
        UNJAMMING, // unjamming the feeder and hopper

       
    };

    // Desired function from user
    public enum WantedState {
        IDLE, PREPARE_TO_DEPLOY, HOME, UNJAM,
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

//    private double mCurrentTuningRpm = Constants.kShooterTuningRpmFloor;
    private double mLastGoalRange = 0.0;

    private boolean mCompressorOverride = false;

    

    public boolean isOnTargetToKeepShooting() {
        return true;
    }

    private Loop mLoop = new Loop() {

        // Every time we transition states, we update the current state start
        // time and the state changed boolean (for one cycle)
        private double mWantStateChangeStartTime;

        @Override
        public void onStart(double timestamp) {
            synchronized (Superstructure.this) {
                mWantedState = WantedState.IDLE;
                mCurrentStateStartTime = timestamp;
                mWantStateChangeStartTime = timestamp;
                
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Superstructure.this) {
                SystemState newState = mSystemState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle(mStateChanged);
                    break;
                case PREPARING_TO_DEPLOY:
                	newState = handlePrepareToDeploy();
                	break;
                case HOME:
                	newState = handleHome();
                	break;
                default:
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("Superstructure state " + mSystemState + " to " + newState + " Timestamp: "
                            + Timer.getFPGATimestamp());
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
    	case PREPARE_TO_DEPLOY:
    		return SystemState.PREPARING_TO_DEPLOY;
    	case HOME:
    		return SystemState.HOME;
    	case IDLE:
    		return SystemState.IDLE;
    	default:
    		return SystemState.IDLE;
        }
    }
    
    private SystemState handlePrepareToDeploy() {
    	if(mElevator.getSeekState() != SeekState.OPEN_SLOT) {//elevator not in position
    		mElevator.setWantedState(Elevator.WantedState.SEEK);
    	}else {
    		//deploy arm and stop elevator in position
    		mArm.deploy();
    		mElevator.setSeekState(Elevator.SeekState.SENSING);
    		mWantedState = WantedState.IDLE;
    		return SystemState.IDLE;
    	}
    	
    	return defaultStateTransfer();
    }
    
    private SystemState handleHome() {
    	mElevator.setWantedState(Elevator.WantedState.HOME);
    	mArm.setWantedState(Arm.WantedState.HOME);
    	
    	return defaultStateTransfer();
    }
    
    private SystemState handleIdle(boolean stateChanged) {
        if (stateChanged) {
            stop();
            mElevator.setWantedState(Elevator.WantedState.OPEN_LOOP);
            mArm.setWantedState(Arm.WantedState.OPEN_LOOP);
//            mLED.setWantedState(LED.WantedState.OFF);
//            mFeeder.setWantedState(Feeder.WantedState.IDLE);
//            mHopper.setWantedState(Hopper.WantedState.IDLE);
        }
//        mCompressor.setClosedLoopControl(!mCompressorOverride);

       return defaultStateTransfer();
    }

    public synchronized double getCurrentRange() {
        return mLastGoalRange;
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }
    
    public SystemState getSystemState() {
    	return mSystemState;
    }

//    @Override
    public void OutputAllToSmartDashboard() {
//    	mDrive.OutputToSmartDashboard();
    	mElevator.OutputToSmartDashboard();
    	mArm.OutputToSmartDashboard();
    	mIntake.OutputToSmartDashboard();
    	SmartDashboard.putString("Superstructure SystemState", mSystemState.name());
    }

//    @Override
    public void stop() {
    	
    }
    
    public void resetAll() {
    	mArm.setWantedState(Arm.WantedState.OPEN_LOOP);
    	mElevator.setWantedState(Elevator.WantedState.OPEN_LOOP);
    	setWantedState(WantedState.IDLE);
    }
//    @Override
    public void zeroSensors() {
    	mDrive.resetEncoders();
    	mElevator.resetEncoder();
    	mArm.resetEncoder();
    }

//    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    public void setWantIntakeReversed() {
        mIntake.setReverse();
    }

    public void setWantIntakeStopped() {
        mIntake.setOff();
    }

    public void setWantIntakeOn() {
        mIntake.setOn();
    }

    public void setOverrideCompressor(boolean force_off) {
        mCompressorOverride = force_off;
    }

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}