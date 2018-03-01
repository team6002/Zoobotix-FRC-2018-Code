
package org.usfirst.frc.team6002.robot;

//import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team6002.robot.subsystems.Arm;
import org.usfirst.frc.team6002.robot.subsystems.Drive;
import org.usfirst.frc.team6002.robot.subsystems.Intake;
import org.usfirst.frc.team6002.robot.subsystems.Superstructure;
import org.usfirst.frc.team6002.robot.subsystems.Superstructure.WantedState;
import org.usfirst.frc.team6002.robot.subsystems.Elevator;
//import org.usfirst.frc.team6002.robot.subsystems.TestSolenoid;

import org.usfirst.frc.team6002.robot.subsystems.I2CArduino;
import org.usfirst.frc.team6002.auto.AutoModeExecuter;
import org.usfirst.frc.team6002.robot.ControlBoard;
import org.usfirst.frc.team6002.lib.util.CrashTracker;
import org.usfirst.frc.team6002.lib.util.DriveSignal;
import org.usfirst.frc.team6002.robot.CheesyDriveHelper;
import org.usfirst.frc.team6002.robot.loops.Looper;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static OI oi;
    private Compressor compressor_;
    
    //Initialize Superstructure
    private Superstructure mSuperstructure = Superstructure.getInstance();
	//S U B S Y S T E M
	Drive mDrive = Drive.getInstance();
	Intake mIntake = Intake.getInstance();
	Elevator mElevator = Elevator.getInstance();
	Arm mArm = Arm.getInstance();
	I2CArduino mArduino = new I2CArduino();
	
	//Counter for slowing down print speed.
	int _loops = 0;
	
	//other parts of robot
	ControlBoard mControls = ControlBoard.getInstance();
	CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
	private AutoModeExecuter mAutoModeExecuter = null;
	
//	SmartDashboardInteractions mSmartDashBoardInteractions = new SmartDashboardInteractions();
	
	// Enabled looper is called at 100Hz whenever the robot is enabled
    Looper mEnabledLooper = new Looper();
    // Disabled looper is called at 100Hz whenever the robot is disabled
    Looper mDisabledLooper = new Looper();
	
    Timer timer;
    
	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
//		oi = new OI();
//		chooser.addDefault("Default Auto", new ExampleCommand());
//		// chooser.addObject("My Auto", new MyAutoCommand());
//		SmartDashboard.putData("Auto mode", chooser);
		timer = new Timer();
		timer.start();
		
		try{
			CameraServer.getInstance().startAutomaticCapture(0);
			CameraServer.getInstance().startAutomaticCapture(1);
//			AxisCamera gearCamera = CameraServer.getInstance().addAxisCamera("axis-camera.local");
//			gearCamera.setResolution(640, 480);
//			CameraServer.getInstance().startAutomaticCapture(dev);
			mEnabledLooper.register(mDrive.getLoop());
			mEnabledLooper.register(mElevator.getLoop());
			mEnabledLooper.register(mArm.getLoop());
			mSuperstructure.registerEnabledLoops(mEnabledLooper);

			mSuperstructure.OutputAllToSmartDashboard();
			AutoModeSelector.initAutoModeSelector();
			
			compressor_ = new Compressor(Constants.kCompressorId);
			compressor_.setClosedLoopControl(true);
			mDrive.setHighGear(false);
			mDrive.resetEncoders();
			mElevator.resetEncoder();
			mArm.resetEncoder();
			
			mArduino.init();
			
			timer.start();
		}catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
		}
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		if (mAutoModeExecuter != null) {
            mAutoModeExecuter.stop();
        }
        mAutoModeExecuter = null;
		mDrive.resetEncoders();
		mIntake.setOff();
//		mElevator.stop();
		mSuperstructure.resetAll();
//		mArm.setWantedState(Arm.WantedState.OPEN_LOOP);
//		mGearArm.closeClaw();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
//		autonomousCommand = chooser.getSelected();
//
//		/*
//		 * String autoSelected = SmartDashboard.getString("Auto Selector",
//		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
//		 * = new MyAutoCommand(); break; case "Default Auto": default:
//		 * autonomousCommand = new ExampleCommand(); break; }
//		 */
//
//		// schedule the autonomous command (example)
//		if (autonomousCommand != null)
//			autonomousCommand.start();
		try {
            CrashTracker.logAutoInit();
            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            
            mAutoModeExecuter = null;
            
            // Configure loopers
            mDisabledLooper.stop();
            mEnabledLooper.start();

            mAutoModeExecuter = new AutoModeExecuter();
//            mAutoModeExecuter.setAutoMode(mSmartDashBoardInteractions.getSelectedAutonMode());
            mAutoModeExecuter.setAutoMode(AutoModeSelector.getSelectedAutoMode());
            mAutoModeExecuter.start();
			
			
            mDrive.setHighGear(false);
            mDrive.resetEncoders();
            
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		try {
            mSuperstructure.OutputAllToSmartDashboard();
//            updateDriverFeedback();
			
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    private static final String COLOR_BOX_COLOR_KEY = "color_box_color";
    private static final String COLOR_BOX_TEXT_KEY = "color_box_text";

    private void updateDriverFeedback() {
 //       boolean hasBall = mSuperstructure.getIntake().hasBall();
 //       mHasBallLightOutput.set(hasBall);
 //       if (hasBall) {
  //          SmartDashboard.putString(COLOR_BOX_COLOR_KEY, "#00ff00");
 //           SmartDashboard.putString(COLOR_BOX_TEXT_KEY, "HAVE BALL");
 //       } else {
 //           SmartDashboard.putString(COLOR_BOX_COLOR_KEY, "#ff0000");
 //           SmartDashboard.putString(COLOR_BOX_TEXT_KEY, "NO BALL");
 //       }
    }
	

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
//		if (autonomousCommand != null)
//			autonomousCommand.cancel();
		try{
			// Configure loopers
			if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;
            mDisabledLooper.stop();
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(false);
            mDrive.resetEncoders();
            mElevator.resetEncoder();//ELEVATOR ENCODER RESET HERE!!
		}catch (Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		try{
			double timestamp = Timer.getFPGATimestamp();
			
			double throttle = mControls.getThrottle();
            double turn = mControls.getTurn();
                mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControls.getQuickTurn()));
                mDrive.setHighGear(!mControls.getLowGear());

//                if(mControls.getDropGear()){
//                	mArduino.Arduino.write(84, mArduino.toSend[0]);
//                }
                if(mControls.getIntake()) {
                	if(!mIntake.getIsIntakeOn()) {
                		mSuperstructure.setWantIntakeOn();
                	}
                	else {
                		mSuperstructure.setWantIntakeStopped();
                	}
                }
//                if(mControls.getReverseIntake()) {
//            		if(!mIntake.getIsIntakeOn()) {
////            			mSuperstructure.setWantIntakeReversed();	
//            			mIntake.setReverse();
//            		}
//            		else {
//            			mSuperstructure.setWantIntakeStopped();
//            		}
//        		}
                if(mControls.getExtendIntake()) {
                	if(!mIntake.getIsIntakeExtended()) {
                		mIntake.set(true);
                	}
                	else {
                		mIntake.set(false);
                	}
                }
                if(mControls.getClaw()) {
                	if(mArm.getIsClawOpen()) {
                		mArm.close();
                	}else {
                		mArm.open();
                	}
                }
//                if(mControls.getElevatorTestB()) {
//                	mElevator.setWantedState(Elevator.WantedState.HOME);
//                }
                if(mControls.getTestB()) {
                	mSuperstructure.setWantedState(Superstructure.WantedState.HOME);
                }
                if (mControls.getTestY()) {
                	mSuperstructure.setWantedState(Superstructure.WantedState.PREPARE_TO_DEPLOY);
                }
               

                mElevator.setOpenLoop(mControls.getElevator());
                mArm.setOpenLoop(mControls.getArm());

                
                

                
                /*
        		 * print every 30 loops, printing too much too fast is generally bad
        		 * for performance
        		 */
        		if (++_loops >= 30) {
        			_loops = 0;
        			mSuperstructure.OutputAllToSmartDashboard();
//        			OutputAllToSmartDashboard();
        		}
                
                
                
                
	}catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
        
	}
}
//	public void OutputAllToSmartDashboard() {
//		mArm.OutputToSmartDashboard();
//		mElevator.OutputToSmartDashboard();
//		mSuperstructure.OutputToSmartDashboard();
//		mIntake.OutputToSmartDashboard();
//	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
//		LiveWindow.run();
	}
}
