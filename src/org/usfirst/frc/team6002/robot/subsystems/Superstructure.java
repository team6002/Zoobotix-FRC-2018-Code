package org.usfirst.frc.team6002.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team6002.robot.Constants;
import org.usfirst.frc.team6002.robot.Robot;
import org.usfirst.frc.team6002.robot.RobotState;
//import org.usfirst.frc.team6002.robot.ShooterAimingParameters;
import org.usfirst.frc.team6002.robot.loops.Loop;
import org.usfirst.frc.team6002.robot.loops.Looper;


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
//    private final Hopper mHopper = Hopper.getInstance();
//    private final Shooter mShooter = Shooter.getInstance();
//    private final LED mLED = LED.getInstance();
//    private final Solenoid mHopperSolenoid = Constants.makeSolenoidForId(Constants.kHopperSolenoidId);
    private final Compressor mCompressor = new Compressor(0);
//    private final RevRoboticsAirPressureSensor mAirPressureSensor = new RevRoboticsAirPressureSensor(3);
    
    private double mCurrentStateStartTime;
    private boolean mStateChanged;
    
    // Superstructure doesn't own the drive, but needs to access it
    private final Drive mDrive = Drive.getInstance();

    // Intenal state of the system
    public enum SystemState {
        IDLE,
        UNJAMMING, // unjamming the feeder and hopper
        HANGING, //Prepare for Hang
        LIFTING, //Elevator Lift up
       
    };

    // Desired function from user
    public enum WantedState {
        IDLE, UNJAM, HANG, LIFT
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
                case UNJAMMING:
                    newState = handleUnjamming();
                    break;
                case HANGING:
                	newState = handleHang();
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

    private SystemState handleIdle(boolean stateChanged) {
        if (stateChanged) {
            stop();
//            mLED.setWantedState(LED.WantedState.OFF);
//            mFeeder.setWantedState(Feeder.WantedState.IDLE);
//            mHopper.setWantedState(Hopper.WantedState.IDLE);
        }
        mCompressor.setClosedLoopControl(!mCompressorOverride);

        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleUnjamming() {
        
        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleHang() {
//        mCompressor.setClosedLoopControl(false);
//        mFeeder.setWantedState(Feeder.WantedState.IDLE);
//        mHopper.setWantedState(Hopper.WantedState.IDLE);
//        mShooter.setOpenLoop(-12.0);
//
        switch (mWantedState) {
        case HANG:
            return SystemState.HANGING;
        default:
            return SystemState.IDLE;
        }
    }

    public synchronized double getCurrentRange() {
        return mLastGoalRange;
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }


//    @Override
    public void outputToSmartDashboard() {
 
    }

//    @Override
    public void stop() {

    }

//    @Override
    public void zeroSensors() {

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