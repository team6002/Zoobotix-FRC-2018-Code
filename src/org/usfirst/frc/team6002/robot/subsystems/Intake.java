package org.usfirst.frc.team6002.robot.subsystems;

import org.usfirst.frc.team6002.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Intake extends Subsystem {
	private static Intake sInstance = null;

    public static Intake getInstance() {
        if (sInstance == null) {
            sInstance = new Intake();
        }
        return sInstance;
    }
    
    //Hardware
	private TalonSRX mMasterIntake, mSlaveIntake;
	//Variables
	boolean IsIntakeOn = false;
	private static double INTAKE_ON = 1;
	private static double INTAKE_REVERSE = -1;
	
	public void init() {
		mMasterIntake = new TalonSRX(Constants.kMasterIntakeId);
		mSlaveIntake = new TalonSRX(Constants.kSlaveIntakeId);
		
		mSlaveIntake.setInverted(true);
		
		mMasterIntake.setNeutralMode(NeutralMode.Brake);
		mSlaveIntake.setNeutralMode(NeutralMode.Brake);
		
		mMasterIntake.set(ControlMode.PercentOutput, 0);
		mSlaveIntake.set(ControlMode.Follower, Constants.kMasterIntakeId);
	}
	
	public void setIntakeRoller(double volt) {
		mMasterIntake.set(ControlMode.PercentOutput, volt);
	}
	
	public void setOn() {
		IsIntakeOn = true;
		setIntakeRoller(INTAKE_ON);
	}
	
	public void setOff() {
		IsIntakeOn = false;
		setIntakeRoller(0);
	}
	
	public void setReverse() {
		setIntakeRoller(-INTAKE_REVERSE);
	}
	
	public boolean getIsIntakeOn() {
		return IsIntakeOn;
		
	}
	public void OutputToSmartDashboard() {
		SmartDashboard.putBoolean("Intake", IsIntakeOn);
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}
