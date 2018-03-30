
package org.usfirst.frc.team6002.robot;

//import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team6002.robot.subsystems.Arm;
import org.usfirst.frc.team6002.robot.subsystems.Drive;
import org.usfirst.frc.team6002.robot.subsystems.Intake;
import org.usfirst.frc.team6002.robot.subsystems.Ramp;
import org.usfirst.frc.team6002.robot.subsystems.Superstructure;
import org.usfirst.frc.team6002.robot.subsystems.Arm.ArmState;
import org.usfirst.frc.team6002.robot.subsystems.Elevator.ElevatorState;
import org.usfirst.frc.team6002.robot.subsystems.Superstructure.WantedState;
import org.usfirst.frc.team6002.robot.subsystems.Elevator;
//import org.usfirst.frc.team6002.robot.subsystems.TestSolenoid;

import org.usfirst.frc.team6002.robot.subsystems.I2CArduino;
import org.usfirst.frc.team6002.auto.AutoModeExecuter;
import org.usfirst.frc.team6002.robot.ControlBoard;
import org.usfirst.frc.team6002.lib.util.CrashTracker;
import org.usfirst.frc.team6002.lib.util.DriveSignal;
import org.usfirst.frc.team6002.lib.util.math.RigidTransform2d;
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
//	private RobotState mRobotState = RobotState.getInstance();
//	Ramp mRamp = Ramp.getInstance();
	I2CArduino mArduino = new I2CArduino();
	
	//Autonomous gamedata
	String gameData;
	
	
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
	
	public void zeroAllSensors() {
		mDrive.resetEncoders();
		mElevator.resetEncoder();
		mArm.resetEncoder();
//		mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
	}
	
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
			zeroAllSensors();
			gameData = DriverStation.getInstance().getGameSpecificMessage();
			
//			
//			mArduino.init();
			
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
        mEnabledLooper.stop();
        mArm.clawClose(true);
        mArm.retractPiston();
        mDrive.setOpenLoop(DriveSignal.NEUTRAL);
		mIntake.setOff();
		gameData = DriverStation.getInstance().getGameSpecificMessage();
//		mSuperstructure.resetAll();
	}

	@Override
	public void disabledPeriodic() {
//		zeroAllSensors();
//		mSuperstructure.resetAll();
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		SmartDashboard.putString("Game data", gameData);//show game data acquired from FMS
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
//            CrashTracker.logAutoInit();
//            if (mAutoModeExecuter != null) {
//                mAutoModeExecuter.stop();
//            }
            
            mAutoModeExecuter = null;
            
            // Configure loopers
            mDisabledLooper.stop();
            mEnabledLooper.start();

//            mAutoModeExecuter = new AutoModeExecuter();
//            mAutoModeExecuter.setAutoMode(mSmartDashBoardInteractions.getSelectedAutonMode());
//            mAutoModeExecuter.setAutoMode(AutoModeSelector.getSelectedAutoMode());
//            mAutoModeExecuter.start();
			
			
            mDrive.setHighGear(false);
            mDrive.resetEncoders();
            //run the autonomous, if we have game data from FMS
            if(gameData.length() > 0) {
            	RIGHTSTART();
            }
            /*
             * Autonomous options:
             * 	MIDDLESTART();
             * 	RIGHTSTART();
             */
            
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
	}
	
	//for gyroTurn, a negative value is to the left, positive to the right
	public void MIDDLESTART() {
		System.out.println("Starting Middle Autonomous");
		System.out.println("gameData is " + gameData);
		mSuperstructure.setWantedState(Superstructure.WantedState.HOLD_CUBE_IN_HOME);//immediately start putting the arm up for autonomous
		if(gameData.charAt(0) == 'L') {//alliance switch on left side
			mDrive.driveSetInches(24, 0.4); //move off wall
			mDrive.gyroTurn(-90);//turn 90 to the left
			mDrive.driveSetInches(48, 0.4);//move to the left towards switch
			mDrive.gyroTurn(90);//turn towards switch
			mDrive.driveSetInches(36, 0.4);//move towards switch
			while(mElevator.getElevatorState() != ElevatorState.SWITCH) {//elevator up to CAPTURED_SWITCH position
				mSuperstructure.setWantedState(Superstructure.WantedState.PREPARE_TO_DEPLOY_TO_SWITCH);
			}
			while(mArm.getArmState() != ArmState.DEPLOYED_LOW) {//deploy cube to the back
				mSuperstructure.setWantedState(Superstructure.WantedState.PREPARE_TO_DEPLOY_LOW);
			}
//			mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT_CUBE);//shoot in the cube
			mSuperstructure.ShootCube();
			timer.delay(0.75);
			mSuperstructure.CloseClaw();
			
		}else {//alliance switch on right side
			mDrive.driveSetInches(24, 0.4); //move off wall
			mDrive.gyroTurn(90);//turn 90 to the right
			mDrive.driveSetInches(48, 0.4);//move to the right towards switch
			mDrive.gyroTurn(-90);//turn towards switch
			mDrive.driveSetInches(36, 0.4);//move towards switch
			while(mElevator.getElevatorState() != ElevatorState.SWITCH) {//elevator up to CAPTURED_SWITCH position
				mSuperstructure.setWantedState(Superstructure.WantedState.PREPARE_TO_DEPLOY_TO_SWITCH);
			}
			while(mArm.getArmState() != ArmState.DEPLOYED_LOW) {//deploy cube to the back
				mSuperstructure.setWantedState(Superstructure.WantedState.PREPARE_TO_DEPLOY_LOW);
			}
//			mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT_CUBE);//shoot in the cube
			mSuperstructure.ShootCube();
			timer.delay(0.75);
			mSuperstructure.CloseClaw();
		}
	}
	public void RIGHTSTART() {
		System.out.println("Starting right side scale");
		if(gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') { //NO SCALE, RIGHTSIDE SWITCH
			//move to the right side of the switch
			//turn robot left towards switch
			//position elevator for switch and launch
			//home elevator and arm
			//back off of switch
			//turn 90 degrees to the right towards scale
			//move forward to in between switch and scale
			//turn 90 degrees to the left
			//move forward to 1st cube
			//turn 90 degrees to the left towards the cube
			//prepare for intake
			//move for cube and capture it
			//position for switch and shoot the cube in.
		}
		else if(gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {//RIGHT SIDE SCALE, NO SWITCH
			mSuperstructure.setWantedState(Superstructure.WantedState.HOLD_CUBE_IN_HOME);
			mDrive.driveSetInches(264, 0.8);//move to scale
			mDrive.gyroTurn(-25);//turn towards scale
			while (mElevator.getElevatorState() != ElevatorState.MID_SCALE) {// move elevator to balanced scale position
				mSuperstructure.setWantedState(Superstructure.WantedState.PREPARE_TO_DEPLOY_TO_MID_SCALE);
			}
			while (mArm.getArmState() != ArmState.DEPLOYED) {// move arm to deployed position
				mSuperstructure.setWantedState(Superstructure.WantedState.PREPARE_TO_DEPLOY_HIGH);
			}
//			mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT_CUBE);
			mSuperstructure.ShootCube();
			timer.delay(0.75);
			mSuperstructure.CloseClaw();
		}
		else if(gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {// RIGHT SIDE SCALE AND SWITCH
			mSuperstructure.setWantedState(Superstructure.WantedState.HOLD_CUBE_IN_HOME);
			mDrive.driveSetInches(264, 0.8);//move to scale
			mDrive.gyroTurn(-25);//turn towards scale
			while (mElevator.getElevatorState() != ElevatorState.MID_SCALE) {// move elevator to balanced scale position
				mSuperstructure.setWantedState(Superstructure.WantedState.PREPARE_TO_DEPLOY_TO_MID_SCALE);
			}
			while (mArm.getArmState() != ArmState.DEPLOYED) {// move arm to deployed position
				mSuperstructure.setWantedState(Superstructure.WantedState.PREPARE_TO_DEPLOY_HIGH);
			}
//			mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT_CUBE);
			mSuperstructure.ShootCube();
			timer.delay(0.75);
			mSuperstructure.CloseClaw();
			double startTime = timer.get();
			while(timer.get() - startTime < 1) {
				// wait for one second
			}
			while (mElevator.getElevatorState() != ElevatorState.HOME) {//home elevator to prepare for switch
				mSuperstructure.setWantedState(Superstructure.WantedState.HOME);
			}
			mDrive.gyroTurn(90);// turn 90 to the right to face back towards scale;
			mSuperstructure.setWantedState(Superstructure.WantedState.PREPARE_FOR_INTAKE);//get intake ready to acquire cube
			mDrive.driveSetInches(48, 0.6); // drive to cube
			if(mSuperstructure.getSystemState() == Superstructure.SystemState.CAPTURED_HOME) {//position for switch and shoot, if cube is detected to be acquired
				while(mElevator.getElevatorState() != ElevatorState.SWITCH) {//deploy elevator to switch position
					mSuperstructure.setWantedState(Superstructure.WantedState.PREPARE_TO_DEPLOY_TO_SWITCH);
				}
				while(mArm.getArmState() != ArmState.DEPLOYED_BACK) {// deploy arm to home position
					mSuperstructure.setWantedState(Superstructure.WantedState.PREPARE_TO_DEPLOY_BACK);
				}
//				mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT_CUBE);//launch cube safely into switch
				mSuperstructure.ShootCube();
				timer.delay(0.75);
				mSuperstructure.CloseClaw();
			}else {//miss! give up
				
			}
			
		}
		else { // no switch or scale, run past the baseline.
			mSuperstructure.setWantedState(Superstructure.WantedState.HOLD_CUBE_IN_HOME);
			mDrive.driveSetInches(140, 0.8);
		}
		
		System.out.println("Autonomous completed fully");
	}
	
	public void BASELINE(){
		System.out.println("Starting Baseline");
		mDrive.driveSetInches(268, 0.3);
		System.out.println("Autonomous completed fully");
	}
	public void TURNTEST() {
		System.out.println("Starting TurnTest");
		mDrive.gyroTurn(90);
		System.out.print("Autonomous completed fully");
	}
	
	public void captureCube() {//used at the beginning of autonomous to secure cube inside robot
		mSuperstructure.setWantedState(Superstructure.WantedState.HOLD_CUBE_IN_HOME);
	}
	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		try {
            mSuperstructure.OutputAllToSmartDashboard();
            SmartDashboard.putString("Game data", gameData);//show game data acquired from FMS
//            mRobotState.outputToSmartDashboard();
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
//			if (mAutoModeExecuter != null) {
//                mAutoModeExecuter.stop();
//            }
//            mAutoModeExecuter = null;
            mDisabledLooper.stop();
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(false);
//            zeroAllSensors();
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
			
			//Drivebase
			double throttle = mControls.getThrottle();
            double turn = mControls.getTurn();
            mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControls.getQuickTurn()));
            mDrive.setHighGear(mControls.getLowGear());

//          if(mControls.getDropGear()){
//            	mArduino.Arduino.write(84, mArduino.toSend[0]);
//           }+
            
            if(mControls.getIntake()) {
                if(!mIntake.getIsIntakeOn()) {
                	mSuperstructure.setWantedState(Superstructure.WantedState.PREPARE_FOR_INTAKE);
                }
                else { //stop intake
                	mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
                }
            }else if(mControls.getReverseIntake()) {
            	if(!mIntake.getIsIntakeOn()) {
                	mSuperstructure.setWantIntakeReversed();
                }
                else { //stop intake
                	mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
                	mSuperstructure.setWantIntakeStopped();
                	
                }
            }
            if(mControls.getDeployIntake()) {
            	if(!mIntake.getDeployed()) {
            		mIntake.deploy();
            	}else {
            		mIntake.stow();
            	}
            }
            
            if(mControls.getDeployCube()) {// use stateUp and stateDown to switch between deploying high, and deploying low.
            	mSuperstructure.setWantedState(Superstructure.WantedState.PREPARE_TO_DEPLOY_CUBE);
//                if(mArm.getIsClawOpen()) {
//                	mArm.close();
//                	mSuperstructure.setWantIntakeStopped();
//                }else {
//                	mArm.open();
//                }
            	
            }else  if(mControls.getShootCube()) {
            	mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT_CUBE);
//            	mSuperstructure.ShootCube();
            }
            

            if(mControls.getHome()) {
                mSuperstructure.setWantedState(Superstructure.WantedState.HOME);
            }
           
            if(mControls.getStateUp()) {
            	mSuperstructure.setWantedState(Superstructure.WantedState.UP);
            }
            if(mControls.getStateDown()) {
            	mSuperstructure.setWantedState(Superstructure.WantedState.DOWN);
            }
            
            
            
            if(Math.abs(mControls.getElevator()) > 0.5) {
                mSuperstructure.setWantedState(Superstructure.WantedState.OPEN_LOOP);//set open loop when using joysticks.
                mElevator.setOpenLoop(mControls.getElevator());
            }
            if(Math.abs(mControls.getArm()) > 0.5) {
                mSuperstructure.setWantedState(Superstructure.WantedState.OPEN_LOOP);//set open loop when using joysticks.
                mArm.setOpenLoop(mControls.getArm());
            }
            


                
             /*
       		 * print every 30 loops, printing too much too fast is generally bad
        	 * for performance
        	 */
        	if (++_loops >= 25) {
       			_loops = 0;
        		mSuperstructure.OutputAllToSmartDashboard();
//        		OutputAllToSmartDashboard();
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
//		LiveWinow.run();
	}
}
