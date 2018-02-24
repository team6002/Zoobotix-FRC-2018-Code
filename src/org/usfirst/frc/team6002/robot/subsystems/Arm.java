package org.usfirst.frc.team6002.robot.subsystems;

import org.usfirst.frc.team6002.robot.Constants;
import org.usfirst.frc.team6002.robot.loops.Loop;
import org.usfirst.frc.team6002.robot.subsystems.Elevator.SystemState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
	
	//Variables
	int wantedPosition = 0;
	
	//Positions for arm
	private int ARM_MAX = 1000;
	private int ARM_MIN = 0;
	private int ARM_DEPLOY = 1000;

	private Arm() {
		mArm = new TalonSRX(Constants.kArmId);
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
		
		resetEncoder();
	}

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
                mCurrentStateStartTime = timestamp;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Arm.this) {
            	mArm.set(ControlMode.Position, wantedPosition);
            	
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };
	
	
	public void setOpenLoop(double joy) {
		if(Math.abs(joy) < 0.5) {
			return;
		}
		int increment = (int) (joy * 75);
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
	
	private void stop() {
		
	}
	
	public void resetEncoder() {
		mArm.setSelectedSensorPosition(0, Constants.kArmPIDLoopId, Constants.kTimeoutMs);
	}
	
	public void OutputToSmartDashboard() {
    	SmartDashboard.putNumber("Arm Position", mArm.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("Arm Wanted Position", wantedPosition);
    	
    }
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

