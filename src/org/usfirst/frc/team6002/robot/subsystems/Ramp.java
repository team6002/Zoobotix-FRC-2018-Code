package org.usfirst.frc.team6002.robot.subsystems;

import org.usfirst.frc.team6002.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Ramp extends Subsystem{
	private static Ramp sInstance = null;

    public static Ramp getInstance() {
        if (sInstance == null) {
            sInstance = new Ramp();
        }
        return sInstance;
    }
    
    //Hardware
    private TalonSRX mLeftRamp, mRightRamp;
    
    private Ramp() {
    	mLeftRamp = new TalonSRX(Constants.kLeftRampId);
    	mRightRamp = new TalonSRX(Constants.kRightRampId);
    	
    	mLeftRamp.setSensorPhase(false);
    	mLeftRamp.setInverted(true);
    	mRightRamp.setInverted(false);
    	
    	mLeftRamp.setNeutralMode(NeutralMode.Brake);
    	mRightRamp.setNeutralMode(NeutralMode.Brake);
    	
    	/* set closed loop gains for elevator, typically kF stays zero. */
		mLeftRamp.config_kF(Constants.kRampPIDLoopId, Constants.kRampKf, Constants.kTimeoutMs);
		mLeftRamp.config_kP(Constants.kRampPIDLoopId, Constants.kRampKp, Constants.kTimeoutMs);
		mLeftRamp.config_kI(Constants.kRampPIDLoopId, Constants.kRampKi, Constants.kTimeoutMs);
		mLeftRamp.config_kD(Constants.kRampPIDLoopId, Constants.kRampKd, Constants.kTimeoutMs);
		
		/* set the peak and nominal outputs, 12V means full */
		mLeftRamp.configNominalOutputForward(0, Constants.kTimeoutMs);
		mLeftRamp.configNominalOutputReverse(0, Constants.kTimeoutMs);
		mLeftRamp.configPeakOutputForward(1, Constants.kTimeoutMs);
		mLeftRamp.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		mLeftRamp.configAllowableClosedloopError(0, Constants.kRampPIDLoopId, Constants.kTimeoutMs);
    	mLeftRamp.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kRampPIDLoopId, Constants.kTimeoutMs);
    	mRightRamp.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kRampPIDLoopId, Constants.kTimeoutMs);
    	
    }
    
    public void setOpenLoop(double joy) {
    	mLeftRamp.set(ControlMode.PercentOutput, joy);
    	mRightRamp.set(ControlMode.PercentOutput, joy);
    }

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}
