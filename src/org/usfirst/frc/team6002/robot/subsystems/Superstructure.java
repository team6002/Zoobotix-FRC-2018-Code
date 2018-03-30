package org.usfirst.frc.team6002.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
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
import org.usfirst.frc.team6002.robot.subsystems.Arm.ArmState;
import org.usfirst.frc.team6002.robot.subsystems.Arm.WantedState;
import org.usfirst.frc.team6002.robot.subsystems.Elevator.ElevatorState;

import java.util.Optional;

/**
 * The superstructure subsystem is the overarching superclass containing all
 * components of the superstructure: the intake, hopper, feeder, shooter and
 * LEDs. The superstructure subsystem also contains some miscellaneous hardware
 * that is located in the superstructure but isn't part of any other subsystems
 * like the compressor, pressure sensor, and hopper wall pistons.
 * 
 * Instead of interacting with subsystems like the feeder and intake directly,
 * the {@link Robot} class interacts with the superstructure, which passes on
 * the commands to the correct subsystem.
 * 
 * The superstructure also coordinates actions between different subsystems like
 * the feeder and shooter.
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

	// private final Feeder mFeeder = Feeder.getInstance();
	private final Intake mIntake = Intake.getInstance();
	private final Elevator mElevator = Elevator.getInstance();
	private final Arm mArm = Arm.getInstance();

	// beam break sensor for cube
	private DigitalInput mLaser = new DigitalInput(Constants.kLaserId);

	private double mCurrentStateStartTime;
	private boolean mStateChanged;

	// Superstructure doesn't own the drive, but needs to access it
	private final Drive mDrive = Drive.getInstance();

	// Master State of the System
	public enum SystemState {
		IDLE, OPEN_LOOP, CAPTURING, // detected powercube has been intaked and is in the claw.
		CAPTURED_HOME, CAPTURED_VAULT, CAPTURED_SWITCH, CAPTURED_LOW_SCALE, CAPTURED_MID_SCALE, CAPTURED_HIGH_SCALE, 
		PREPARING_TO_DEPLOY_HIGH, // move cube to be deployed in a high position (like this / )
		PREPARING_TO_DEPLOY_LOW, // move cube to be deployed in a low position (like this R--[p] )
		PREPARING_TO_DEPLOY_BACK, //move cube to be deployed in home position [p]--R )
		DROP_DEPLOY, // drop cube by only releasing claw.
		SHOOT_DEPLOY, // push cube with piston while releasing claw.
		READY_FOR_INTAKE, VAULT, HOME, UNJAMMING, // unjamming the feeder and hopper
		UP, DOWN,
	};

	// Desired Function from robot and user
	public enum WantedState {
		IDLE, OPEN_LOOP, CAPTURE, HOLD_CUBE_IN_HOME, PREPARE_FOR_INTAKE, PREPARE_TO_DEPLOY_TO_VAULT, PREPARE_TO_DEPLOY_TO_SWITCH, PREPARE_TO_DEPLOY_TO_LOW_SCALE, PREPARE_TO_DEPLOY_TO_MID_SCALE, PREPARE_TO_DEPLOY_TO_HIGH_SCALE, PREPARE_TO_DEPLOY_HIGH, PREPARE_TO_DEPLOY_LOW, PREPARE_TO_DEPLOY_BACK, PREPARE_TO_DEPLOY_CUBE, DROP_CUBE, SHOOT_CUBE, HOME, UNJAM, UP, // move
																																																																																					// up
																																																																																					// a
																																																																																					// stage
		DOWN, // move down a stage
	}

	// holds what state the superstructure has the robot in.
	public enum SuperstructureState {
		HOME, READY_FOR_INTAKE, CAPTURED, SWITCH, LOW_SCALE, MID_SCALE, HIGH_SCALE,
	}

	// various stages for the different superstructure states
	public enum HomeStage {
		CHECK, PLUS_FOUR, HOME, COMPLETE,
	}

	public enum SwitchStage {
		STAGE_1, STAGE_2, COMPLETE,
	}

	public enum LowScaleStage {
		ELEVATOR, ARM, COMPLETE,
	}

	public enum MidScaleStage {
		ELEVATOR, ARM, COMPLETE,
	}

	public enum HighScaleStage {
		ELEVATOR, ARM, COMPLETE,
	}

	public enum PrepareForIntakeStage {
		HOME, INTAKE_AND_CLAW, COMPLETE,
	}

	public enum CaptureStage {
		CLAW, ELEVATOR, ARM, COMPLETE
	}

	public enum Stage {
		STAGE_1, STAGE_2, STAGE_3,
	}

	private SuperstructureState mSuperstructureState = SuperstructureState.HOME;
	private SystemState mSystemState = SystemState.IDLE;
	private WantedState mWantedState = WantedState.IDLE;
	private Stage mStage = Stage.STAGE_1;
	private HomeStage mHomeStage = HomeStage.CHECK;
	private SwitchStage mSwitchStage = SwitchStage.STAGE_1;
	private LowScaleStage mLowScaleStage = LowScaleStage.ELEVATOR;
	private MidScaleStage mMidScaleStage = MidScaleStage.ELEVATOR;
	private HighScaleStage mHighScaleStage = HighScaleStage.ELEVATOR;
	private PrepareForIntakeStage mPrepareForIntakeStage = PrepareForIntakeStage.HOME;
	private CaptureStage mCaptureStage = CaptureStage.CLAW;

	// private double mCurrentTuningRpm = Constants.kShooterTuningRpmFloor;
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
					newState = handleIdle();
					break;
				case OPEN_LOOP:
					newState = handleOpenLoop();
					break;
				case CAPTURING:
					newState = handleCapture();
					break;
				case CAPTURED_HOME:
					newState = handleHoldCubeInHome();
					break;
				case CAPTURED_VAULT:
					newState = handlePrepareToDeployToVault();
					break;
				case CAPTURED_SWITCH:
					newState = handlePrepareToDeployToSwitch();
					break;
				case CAPTURED_LOW_SCALE:
					newState = handlePrepareToDeployToLowScale();
					break;
				case CAPTURED_MID_SCALE:
					newState = handlePrepareToDeployToMidScale();
					break;
				case CAPTURED_HIGH_SCALE:
					newState = handlePrepareToDeployToHighScale();
					break;
				case READY_FOR_INTAKE:
					newState = handlePrepareForIntake();
					break;
				case PREPARING_TO_DEPLOY_HIGH:
					newState = handlePrepareToDeployHigh();
					break;
				case PREPARING_TO_DEPLOY_LOW:
					newState = handlePrepareToDeployLow();
					break;
				case PREPARING_TO_DEPLOY_BACK:
					newState = handlePrepareToDeployBack();
					break;
				case DROP_DEPLOY:
					newState = handleDropCube();
					break;
				case SHOOT_DEPLOY:
					newState = handleShootCube();
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
		case OPEN_LOOP:
			return SystemState.OPEN_LOOP;
		case CAPTURE:
			return SystemState.CAPTURING;
		case PREPARE_FOR_INTAKE:
			return SystemState.READY_FOR_INTAKE;
		case HOLD_CUBE_IN_HOME:
			return SystemState.CAPTURED_HOME;
		case HOME:
			return SystemState.HOME;
		case IDLE:
			return SystemState.IDLE;
		default:
			return SystemState.IDLE;
		}
	}

	// state order
	// captured
	// v
	// vault -> home -> switch -> low_scale -> mid_scale -> high_scale
	private SystemState handleHome() {
		if (mStateChanged) {
			mHomeStage = HomeStage.CHECK;
			mArm.retractPiston();
		}

		switch (mHomeStage) {
		case CHECK:
			// if elevator is already at home, don't move anything
			if (mElevator.getElevatorState() == ElevatorState.HOME && mArm.getArmState() == ArmState.HOME
			// || mSuperstructureState == SuperstructureState.HOME
			) {
				mHomeStage = HomeStage.COMPLETE;
			} else {
				mHomeStage = HomeStage.PLUS_FOUR;
			}
		case PLUS_FOUR:
			// if elevator is at 4 inches AND claw is at 0, advance to stage 2
			if (mElevator.getElevatorState() == ElevatorState.PLUS_FOUR && mArm.getArmState() == ArmState.HOME) {
				mHomeStage = HomeStage.HOME;

			} else {
				mArm.retractPiston();
				mArm.clawClose(true);
				// if elevator is not at 4 inches, move elevator to position
				mElevator.setWantedState(Elevator.WantedState.HOME_PLUS_FOUR);
				// if arm is not at 0, continue to move arm
				mArm.setWantedState(Arm.WantedState.HOME);

			}
			break;
		case HOME:
			// if elevator is at 0, stop elevator and return to idle
			if (mElevator.getElevatorState() == ElevatorState.HOME) {
				mHomeStage = HomeStage.COMPLETE;
			} else {
				mArm.clawClose(true);
				mElevator.setWantedState(Elevator.WantedState.HOME);
			} // else, continue to lower elevator

			break;
		case COMPLETE:
			mSuperstructureState = SuperstructureState.HOME;
			break;
		}

		return defaultStateTransfer();
	}

	private SystemState handlePrepareForIntake() {
		if (mStateChanged) {
			mPrepareForIntakeStage = PrepareForIntakeStage.HOME;
		}

		switch (mPrepareForIntakeStage) {
		case HOME:
			if (mElevator.getElevatorState() == ElevatorState.HOME && mArm.getArmState() == ArmState.HOME) {
				mPrepareForIntakeStage = PrepareForIntakeStage.INTAKE_AND_CLAW;
			} else {
				mWantedState = WantedState.HOME;
			}
			break;
		case INTAKE_AND_CLAW:
			mArm.clawClose(false);
			mIntake.deploy();
			mIntake.extend();
			setWantIntakeOn();
			mPrepareForIntakeStage = PrepareForIntakeStage.COMPLETE;
			break;
		case COMPLETE:
//			if (getLaser()) {
//				Timer.delay(0.5);
//				mWantedState = WantedState.CAPTURE;
//			}
			break;
		}

		switch (mWantedState) {
		case CAPTURE:
			return SystemState.CAPTURING;
		case PREPARE_FOR_INTAKE:
			return SystemState.READY_FOR_INTAKE;
		case PREPARE_TO_DEPLOY_CUBE:
			mWantedState = WantedState.CAPTURE;
			return SystemState.CAPTURING;
		case HOME:
			return SystemState.HOME;
		case IDLE:
			return SystemState.IDLE;
		default:
			return SystemState.IDLE;
		}
	}

	private SystemState handleCapture() {
		if (mStateChanged) {
			mCaptureStage = CaptureStage.CLAW;
		}
		switch (mCaptureStage) {
		case CLAW:
			mArm.clawClose(true);
			setWantIntakeStopped();
			Timer.delay(0.3);
			mIntake.stow();
			Timer.delay(0.5);
			mCaptureStage = CaptureStage.ARM;
			break;
		case ARM:// if arm is in captured position, move elevator
			if (mArm.getArmState() == ArmState.CAPTURED) {
				mCaptureStage = CaptureStage.COMPLETE;
			} else {// else move arm into position
				mArm.setWantedState(Arm.WantedState.CAPTURE);
			}
			break;
		// case ELEVATOR://if elevator is in captured position, complete
		// if(mElevator.getElevatorState() == ElevatorState.CAPTURED) {
		// mCaptureStage = CaptureStage.COMPLETE;
		// }else {//else, move elevator to position
		// mElevator.setWantedState(Elevator.WantedState.CAPTURE);
		// }
		// break;
		case COMPLETE:
			mWantedState = WantedState.HOLD_CUBE_IN_HOME;
			break;
		}

		switch (mWantedState) {
		case OPEN_LOOP:
			return SystemState.OPEN_LOOP;
		case CAPTURE:
			return SystemState.CAPTURING;
		case HOLD_CUBE_IN_HOME:
			return SystemState.CAPTURED_HOME;
		case IDLE:
			return SystemState.IDLE;
		default:
			return SystemState.IDLE;
		}
	}

	private SystemState handleHoldCubeInHome() {
		if (mStateChanged) {
			mCaptureStage = CaptureStage.CLAW;
		}
		switch (mCaptureStage) {
		case CLAW:
			mArm.clawClose(true);
			mCaptureStage = CaptureStage.ARM;
			break;
		case ARM:// if arm is in captured position, move elevator
			if (mArm.getArmState() == ArmState.CAPTURED) {
				mCaptureStage = CaptureStage.COMPLETE;
			} else {// else move arm into position
				mArm.setWantedState(Arm.WantedState.CAPTURE);
			}
			break;
		// case ELEVATOR://if elevator is in captured position(home), complete
		// if(mElevator.getElevatorState() == ElevatorState.CAPTURED) {
		// mCaptureStage = CaptureStage.COMPLETE;
		// }else {//else, move elevator to position
		// mElevator.setWantedState(Elevator.WantedState.CAPTURE);
		// }
		// break;
			
		case COMPLETE:
			break;
		}

		switch (mWantedState) {
		case OPEN_LOOP:
			return SystemState.OPEN_LOOP;
		case CAPTURE:
			return SystemState.CAPTURING;
		case HOLD_CUBE_IN_HOME:
			return SystemState.CAPTURED_HOME;
		case PREPARE_TO_DEPLOY_CUBE:// stay in home if button to deploy cube accidently pressed
			mWantedState = WantedState.HOLD_CUBE_IN_HOME;
			return SystemState.CAPTURED_HOME;
		case SHOOT_CUBE:
			mWantedState = WantedState.HOLD_CUBE_IN_HOME;
			return SystemState.CAPTURED_HOME;
		case PREPARE_TO_DEPLOY_TO_SWITCH:
			return SystemState.CAPTURED_SWITCH;
		case PREPARE_TO_DEPLOY_TO_LOW_SCALE:
			return SystemState.CAPTURED_LOW_SCALE;
		case PREPARE_TO_DEPLOY_TO_MID_SCALE:
			return SystemState.CAPTURED_MID_SCALE;
		case PREPARE_TO_DEPLOY_TO_HIGH_SCALE:
			return SystemState.CAPTURED_HIGH_SCALE;
		case HOME:
			return SystemState.HOME;
		case UP:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_TO_SWITCH;
			return SystemState.CAPTURED_SWITCH;
		case DOWN:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_TO_VAULT;
			return SystemState.CAPTURED_VAULT;
		case IDLE:
			return SystemState.IDLE;
		default:
			return SystemState.IDLE;
		}
	}

	private SystemState handlePrepareToDeployToVault() {
		mElevator.setWantedState(Elevator.WantedState.VAULT);
		mArm.setWantedState(Arm.WantedState.VAULT);

		switch (mWantedState) {
		case OPEN_LOOP:
			return SystemState.OPEN_LOOP;
		case PREPARE_FOR_INTAKE:
			return SystemState.READY_FOR_INTAKE;
		case HOME:
			return SystemState.HOME;
		case PREPARE_TO_DEPLOY_TO_VAULT:
			return SystemState.CAPTURED_VAULT;
		case UP:
			mWantedState = WantedState.HOLD_CUBE_IN_HOME;
			return SystemState.CAPTURED_HOME;
		case DOWN:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_TO_VAULT;
			return SystemState.CAPTURED_VAULT;
		case PREPARE_TO_DEPLOY_CUBE:
			mWantedState = WantedState.DROP_CUBE;
			return SystemState.DROP_DEPLOY;
		case SHOOT_CUBE:
			return SystemState.SHOOT_DEPLOY;
		case IDLE:
			return SystemState.IDLE;
		default:
			return SystemState.IDLE;
		}
	}

	private SystemState handlePrepareToDeployToSwitch() {
		if (mStateChanged) {
			mSwitchStage = SwitchStage.STAGE_1;
		}

		switch (mSwitchStage) {
		case STAGE_1:
			// if elevator is at switch position, proceed to stage 2
			if (mElevator.getElevatorState() == ElevatorState.SWITCH) {
				mSwitchStage = SwitchStage.COMPLETE;
			} else {// else, move elevator to switch position
				mElevator.setWantedState(Elevator.WantedState.SWITCH);
			}
		case COMPLETE:
			mSuperstructureState = SuperstructureState.SWITCH;
			break;
		}

		switch (mWantedState) {
		case OPEN_LOOP:
			return SystemState.OPEN_LOOP;
		case PREPARE_FOR_INTAKE:
			return SystemState.READY_FOR_INTAKE;
		case HOME:
			return SystemState.HOME;
		case PREPARE_TO_DEPLOY_TO_SWITCH:
			return SystemState.CAPTURED_SWITCH;
		case UP:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_TO_LOW_SCALE;
			return SystemState.CAPTURED_LOW_SCALE;
		case DOWN:
			mWantedState = WantedState.HOLD_CUBE_IN_HOME;
			return SystemState.CAPTURED_HOME;
		case PREPARE_TO_DEPLOY_CUBE:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_LOW;
			return SystemState.PREPARING_TO_DEPLOY_LOW;
		case PREPARE_TO_DEPLOY_LOW:
			return SystemState.PREPARING_TO_DEPLOY_LOW;
		case PREPARE_TO_DEPLOY_HIGH:
			return SystemState.PREPARING_TO_DEPLOY_HIGH;
		case IDLE:
			return SystemState.IDLE;
		default:
			return SystemState.IDLE;
		}
	}

	private SystemState handlePrepareToDeployToLowScale() {
		if (mStateChanged) {
			mLowScaleStage = LowScaleStage.ELEVATOR;
		}

		switch (mLowScaleStage) {
		case ELEVATOR:
			// if elevator is at switch position, proceed to stage 2
			if (mElevator.getElevatorState() == ElevatorState.LOW_SCALE) {
				mLowScaleStage = LowScaleStage.COMPLETE;
			} else {// else, move elevator to switch position
				mElevator.setWantedState(Elevator.WantedState.LOW_SCALE);
			}
		case COMPLETE:
			mSuperstructureState = SuperstructureState.LOW_SCALE;
			break;
		}
		switch (mWantedState) {
		case OPEN_LOOP:
			return SystemState.OPEN_LOOP;
		case PREPARE_FOR_INTAKE:
			return SystemState.READY_FOR_INTAKE;
		case HOME:
			return SystemState.HOME;
		case PREPARE_TO_DEPLOY_TO_LOW_SCALE:
			return SystemState.CAPTURED_LOW_SCALE;
		case UP:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_TO_MID_SCALE;
			return SystemState.CAPTURED_MID_SCALE;
		case DOWN:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_TO_SWITCH;
			return SystemState.CAPTURED_SWITCH;
		case PREPARE_TO_DEPLOY_CUBE:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_HIGH;
			return SystemState.PREPARING_TO_DEPLOY_HIGH;
		case PREPARE_TO_DEPLOY_HIGH:
			return SystemState.PREPARING_TO_DEPLOY_HIGH;
		case PREPARE_TO_DEPLOY_LOW:
			return SystemState.PREPARING_TO_DEPLOY_LOW;
		case IDLE:
			return SystemState.IDLE;
		default:
			return SystemState.IDLE;
		}
	}

	private SystemState handlePrepareToDeployToMidScale() {
		if (mStateChanged) {
			mMidScaleStage = MidScaleStage.ELEVATOR;
		}

		switch (mMidScaleStage) {
		case ELEVATOR:
			// if elevator is at switch position, proceed to stage 2
			if (mElevator.getElevatorState() == ElevatorState.MID_SCALE) {
				mMidScaleStage = MidScaleStage.COMPLETE;
			} else {// else, move elevator to switch position
				mElevator.setWantedState(Elevator.WantedState.MID_SCALE);
			}
		case COMPLETE:
			mSuperstructureState = SuperstructureState.MID_SCALE;
			break;
		}
		switch (mWantedState) {
		case OPEN_LOOP:
			return SystemState.OPEN_LOOP;
		case PREPARE_FOR_INTAKE:
			return SystemState.READY_FOR_INTAKE;
		case HOME:
			return SystemState.HOME;
		case PREPARE_TO_DEPLOY_TO_MID_SCALE:
			return SystemState.CAPTURED_MID_SCALE;
		case UP:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_TO_HIGH_SCALE;
			return SystemState.CAPTURED_HIGH_SCALE;
		case DOWN:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_TO_LOW_SCALE;
			return SystemState.CAPTURED_LOW_SCALE;
		case PREPARE_TO_DEPLOY_CUBE:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_HIGH;
			return SystemState.PREPARING_TO_DEPLOY_HIGH;
		case PREPARE_TO_DEPLOY_HIGH:
			return SystemState.PREPARING_TO_DEPLOY_HIGH;
		case PREPARE_TO_DEPLOY_LOW:
			return SystemState.PREPARING_TO_DEPLOY_LOW;
		case IDLE:
			return SystemState.IDLE;
		default:
			return SystemState.IDLE;
		}
	}

	private SystemState handlePrepareToDeployToHighScale() {
		if (mStateChanged) {
			mHighScaleStage = mHighScaleStage.ELEVATOR;
		}

		switch (mHighScaleStage) {
		case ELEVATOR:
			// if elevator is at switch position, proceed to stage 2
			if (mElevator.getElevatorState() == ElevatorState.HIGH_SCALE) {
				mHighScaleStage = mHighScaleStage.COMPLETE;
			} else {// else, move elevator to switch position
				mElevator.setWantedState(Elevator.WantedState.HIGH_SCALE);
			}
		case COMPLETE:
			mSuperstructureState = SuperstructureState.HIGH_SCALE;
			break;
		}
		switch (mWantedState) {
		case OPEN_LOOP:
			return SystemState.OPEN_LOOP;
		case PREPARE_FOR_INTAKE:
			return SystemState.READY_FOR_INTAKE;
		case HOME:
			return SystemState.HOME;
		case PREPARE_TO_DEPLOY_TO_HIGH_SCALE:
			return SystemState.CAPTURED_HIGH_SCALE;
		case UP:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_TO_HIGH_SCALE;
			return SystemState.CAPTURED_HIGH_SCALE;
		case DOWN:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_TO_MID_SCALE;
			return SystemState.CAPTURED_MID_SCALE;
		case PREPARE_TO_DEPLOY_CUBE:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_HIGH;
			return SystemState.PREPARING_TO_DEPLOY_HIGH;
		case PREPARE_TO_DEPLOY_HIGH:
			return SystemState.PREPARING_TO_DEPLOY_HIGH;
		case PREPARE_TO_DEPLOY_LOW:
			return SystemState.PREPARING_TO_DEPLOY_LOW;
		case IDLE:
			return SystemState.IDLE;
		default:
			return SystemState.IDLE;
		}
	}

	private SystemState handlePrepareToDeployHigh() {
		mArm.setWantedState(Arm.WantedState.DEPLOY);

		switch (mWantedState) {
		case OPEN_LOOP:
			return SystemState.OPEN_LOOP;
		case PREPARE_FOR_INTAKE:
			return SystemState.READY_FOR_INTAKE;
		case HOME:
			return SystemState.HOME;
		case PREPARE_TO_DEPLOY_HIGH:
			return SystemState.PREPARING_TO_DEPLOY_HIGH;
		case UP:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_HIGH;
			return SystemState.PREPARING_TO_DEPLOY_HIGH;
		case DOWN:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_LOW;
			return SystemState.PREPARING_TO_DEPLOY_LOW;
		case PREPARE_TO_DEPLOY_CUBE:
			mWantedState = WantedState.DROP_CUBE;
			return SystemState.DROP_DEPLOY;
		case SHOOT_CUBE:
			return SystemState.SHOOT_DEPLOY;
		case IDLE:
			return SystemState.IDLE;
		default:
			return SystemState.IDLE;
		}
	}

	private SystemState handlePrepareToDeployLow() {
		mArm.setWantedState(Arm.WantedState.DEPLOY_LOW);

		switch (mWantedState) {
		case OPEN_LOOP:
			return SystemState.OPEN_LOOP;
		case PREPARE_FOR_INTAKE:
			return SystemState.READY_FOR_INTAKE;
		case HOME:
			return SystemState.HOME;
		case PREPARE_TO_DEPLOY_LOW:
			return SystemState.PREPARING_TO_DEPLOY_LOW;
		case UP:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_HIGH;
			return SystemState.PREPARING_TO_DEPLOY_HIGH;
		case DOWN:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_LOW;
			return SystemState.PREPARING_TO_DEPLOY_LOW;
		case PREPARE_TO_DEPLOY_CUBE:
			mWantedState = WantedState.DROP_CUBE;
			return SystemState.DROP_DEPLOY;
		case SHOOT_CUBE:
			return SystemState.SHOOT_DEPLOY;
		case IDLE:
			return SystemState.IDLE;
		default:
			return SystemState.IDLE;
		}

	}
	
	private SystemState handlePrepareToDeployBack() {//deploy arm to home position
		mArm.setWantedState(Arm.WantedState.DEPLOY_BACK);
		
		switch (mWantedState) {
		case OPEN_LOOP:
			return SystemState.OPEN_LOOP;
		case PREPARE_FOR_INTAKE:
			return SystemState.READY_FOR_INTAKE;
		case HOME:
			return SystemState.HOME;
		case PREPARE_TO_DEPLOY_BACK:
			return SystemState.PREPARING_TO_DEPLOY_BACK;
		case UP:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_HIGH;
			return SystemState.PREPARING_TO_DEPLOY_HIGH;
		case DOWN:
			mWantedState = WantedState.PREPARE_TO_DEPLOY_LOW;
			return SystemState.PREPARING_TO_DEPLOY_LOW;
		case PREPARE_TO_DEPLOY_CUBE:
			mWantedState = WantedState.DROP_CUBE;
			return SystemState.DROP_DEPLOY;
		case SHOOT_CUBE:
			return SystemState.SHOOT_DEPLOY;
		case IDLE:
			return SystemState.IDLE;
		default:
			return SystemState.IDLE;
		}
	}

	private SystemState handleDropCube() {
		mArm.clawClose(false);
		// Timer.delay(1.5);
		mWantedState = WantedState.IDLE;
		return SystemState.IDLE;
	}

	private SystemState handleShootCube() {
		mArm.clawClose(false);
		mArm.extendPiston();
		Timer.delay(0.75);
		mArm.retractPiston();
		mWantedState = WantedState.IDLE;
		return SystemState.IDLE;
	}

	private SystemState handleOpenLoop() {
		mElevator.setWantedState(Elevator.WantedState.OPEN_LOOP);
		mArm.setWantedState(Arm.WantedState.OPEN_LOOP);

		switch (mWantedState) {
		case OPEN_LOOP:
			return SystemState.OPEN_LOOP;
		case CAPTURE:
			return SystemState.CAPTURING;
		case PREPARE_FOR_INTAKE:
			return SystemState.READY_FOR_INTAKE;
		case PREPARE_TO_DEPLOY_CUBE:
			mWantedState = WantedState.DROP_CUBE;
			return SystemState.DROP_DEPLOY;
		case SHOOT_CUBE:
			return SystemState.SHOOT_DEPLOY;
		case HOME:
			return SystemState.HOME;
		case IDLE:
			return SystemState.IDLE;
		default:
			return SystemState.IDLE;
		}
	}

	private SystemState handleIdle() {// default state to go to and do nothing.
		if (mStateChanged) {
			setWantIntakeStopped();
		}

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

	// @Override
	public void OutputAllToSmartDashboard() {
		mDrive.OutputToSmartDashboard();
		mElevator.OutputToSmartDashboard();
		mArm.OutputToSmartDashboard();
		mIntake.OutputToSmartDashboard();
		SmartDashboard.putString("Superstructure SystemState", mSystemState.name());
		SmartDashboard.putString("Superstructure WantedState", mWantedState.name());
		SmartDashboard.putString("Superstructure HomeState", mHomeStage.name());
		SmartDashboard.putBoolean("Cube Detector", mLaser.get());
	}

	// @Override
	public void stop() {

	}

	public void resetAll() {
		mSuperstructureState = SuperstructureState.HOME;
		mStage = Stage.STAGE_1;
		mHomeStage = HomeStage.CHECK;
		mSwitchStage = SwitchStage.STAGE_1;
		mLowScaleStage = LowScaleStage.ELEVATOR;
		mPrepareForIntakeStage = PrepareForIntakeStage.HOME;
		mCaptureStage = CaptureStage.CLAW;
		mArm.resetState();
		mElevator.resetState();
		mArm.setWantedState(Arm.WantedState.OPEN_LOOP);
		mElevator.setWantedState(Elevator.WantedState.OPEN_LOOP);
		setWantedState(WantedState.IDLE);
		zeroSensors();
	}

	// @Override
	public void zeroSensors() {
		mDrive.resetEncoders();
		mElevator.resetEncoder();
		mArm.resetEncoder();
	}

	// @Override
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
	
	public void OpenClaw() {
		mArm.open();
	}
	
	public void CloseClaw() {
		mArm.close();
		mArm.retractPiston();
	}
	
	public void ShootCube() {
		mArm.open();
		mArm.extendPiston();
	}
	
	public boolean getLaser() {
		return mLaser.get();
	}

	public void setOverrideCompressor(boolean force_off) {
		mCompressorOverride = force_off;
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}
}