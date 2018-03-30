package org.usfirst.frc.team6002.robot.subsystems;

import org.usfirst.frc.team6002.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
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
	private TalonSRX mLeftIntake, mRightIntake;
	private Solenoid mIntakeSolenoid, mDeploySolenoid;
	//Variables
	boolean IsIntakeOn, IsIntakeExtended;
	private double INTAKE_ON = 0.5;
	private double INTAKE_REVERSE = -0.5;
	
	public Intake() {
		IsIntakeOn = false;
		IsIntakeExtended = false;
		mIntakeSolenoid = new Solenoid(Constants.kIntakeSolenoidId);
		mDeploySolenoid = new Solenoid(Constants.kDeploySolenoidId);
		mLeftIntake = new TalonSRX(Constants.kLeftIntakeId);
		mRightIntake = new TalonSRX(Constants.kRightIntakeId);
		
		mLeftIntake.setInverted(true);
		mRightIntake.setInverted(false);
		
		mLeftIntake.setNeutralMode(NeutralMode.Brake);
		mRightIntake.setNeutralMode(NeutralMode.Brake);
		
		mLeftIntake.set(ControlMode.PercentOutput, 0);
		mRightIntake.set(ControlMode.PercentOutput, 0);
		
		mIntakeSolenoid.set(false);
		mDeploySolenoid.set(false);
	}
	
	private void setOpenLoop(double volt, double volt2) {
		mLeftIntake.set(ControlMode.PercentOutput, volt);
		mRightIntake.set(ControlMode.PercentOutput, volt2);
	}
	
	boolean deployed = false;
	public void deploy() {
		deployed = true;
		mDeploySolenoid.set(true);
	}
	public void stow() {
		deployed = false;
		mDeploySolenoid.set(false);
	}
	public boolean getDeployed() {
		return deployed;
	}
	
	public void setOn() {
		IsIntakeOn = true;
		setOpenLoop(0.6, 0.9);
	}
	
	public void spin() {
		IsIntakeOn = true;
		setOpenLoop(-0.3, 0.3);
	}
	
	public void setOff() {
		IsIntakeOn = false;
		setOpenLoop(0,0);
	}
	
	public void extend() {
		IsIntakeExtended = true;
		mIntakeSolenoid.set(true);
	}
	public void retract() {
		IsIntakeExtended = false;
		mIntakeSolenoid.set(false);
	}
	public boolean getIsIntakeExtended() {
		return IsIntakeExtended;
	}
	
	public void setReverse() {
		IsIntakeOn = true;
		setOpenLoop(INTAKE_REVERSE, INTAKE_REVERSE);
	}
	
	public boolean getIsIntakeOn() {
		return IsIntakeOn;
		
	}
	public void OutputToSmartDashboard() {
		SmartDashboard.putBoolean("Intake", IsIntakeOn);
		SmartDashboard.putBoolean("Extended", getIsIntakeExtended());
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}
