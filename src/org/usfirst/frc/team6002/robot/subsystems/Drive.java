package org.usfirst.frc.team6002.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import com.ctre.TalonSRX.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import java.util.Set;
import org.usfirst.frc.team6002.lib.util.drivers.NavX;
import org.usfirst.frc.team6002.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team6002.lib.util.math.Rotation2d;
import org.usfirst.frc.team6002.lib.util.math.Twist2d;
import org.usfirst.frc.team6002.lib.util.DriveSignal;
import org.usfirst.frc.team6002.lib.util.SynchronousPID;
import org.usfirst.frc.team6002.lib.util.control.AdaptivePurePursuitController;
import org.usfirst.frc.team6002.lib.util.control.Path;
import org.usfirst.frc.team6002.lib.util.control.PathFollower;
import org.usfirst.frc.team6002.lib.util.control.Lookahead;
import org.usfirst.frc.team6002.lib.util.ReflectingCSVWriter;
import org.usfirst.frc.team6002.robot.Constants;
import org.usfirst.frc.team6002.robot.Kinematics;
import org.usfirst.frc.team6002.robot.RobotState;
import org.usfirst.frc.team6002.robot.loops.Loop;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Drive extends Subsystem {
	private static Drive mInstance = new Drive();
	
	public static Drive getInstance() {
        return mInstance;
    }
    
	protected static final int kVelocityControlSlot = 0;
    
    private double mLastHeadingErrorDegrees = 0;
    
    private PathFollower mPathFollower;

    // These gains get reset below!!
    private Rotation2d mTargetHeading = new Rotation2d();
    private Path mCurrentPath = null;
    
 // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        VELOCITY_SETPOINT, // velocity PID control
        PATH_FOLLOWING, // used for autonomous driving
        TURN_TO_HEADING, // turn in place
        PREPARE_TO_DEPLOY, // move drive base to be correct distance away from scale
    }
    
 // Control states
    private DriveControlState mDriveControlState;
    
    /**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(DriveControlState state) {
        if (state == DriveControlState.VELOCITY_SETPOINT || state == DriveControlState.PATH_FOLLOWING) {
            return true;
        }
        return false;
    }

    /**
     * Check if the drive talons are configured for position control
     */
    protected static boolean usesTalonPositionControl(DriveControlState state) {
        if (state == DriveControlState.TURN_TO_HEADING)
                {
        	return true;
        }
        return false;
    }
    
    public static class VelocityHeadingSetpoint {
        private final double leftSpeed_;
        private final double rightSpeed_;
        private final Rotation2d headingSetpoint_;

        // Constructor for straight line motion
        public VelocityHeadingSetpoint(double leftSpeed, double rightSpeed, Rotation2d headingSetpoint) {
            leftSpeed_ = leftSpeed;
            rightSpeed_ = rightSpeed;
            headingSetpoint_ = headingSetpoint;
        }

        public double getLeftSpeed() {
            return leftSpeed_;
        }

        public double getRightSpeed() {
            return rightSpeed_;
        }

        public Rotation2d getHeading() {
            return headingSetpoint_;
        }
    }
    //Hardware
    private TalonSRX leftMaster_, rightMaster_; // leftSlave_, rightSlave_
    private VictorSPX leftVictorSlave_, rightVictorSlave_;
    private final NavX mNavXBoard;
    private Solenoid shifter_;
    
    //Hardware States
    private boolean isBrakeMode_ = true;
    private boolean mIsOnTarget = false;
    private boolean mIsHighGear = false;
    
    //Controllers
    private RobotState mRobotState = RobotState.getInstance();
    private AdaptivePurePursuitController pathFollowingController_;
    
    private DriveControlState driveControlState_;
    private VelocityHeadingSetpoint velocityHeadingSetpoint_;
    private SynchronousPID velocityHeadingPid_;
    
    // Logging
    private final ReflectingCSVWriter<PathFollower.DebugOutput> mCSVWriter;
    
    public Loop getLoop() {
        return mLoop;
    }
    
    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(DriveSignal.NEUTRAL);
                setBrakeMode(false);
                setVelocitySetpoint(0, 0);
                mNavXBoard.reset();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                case OPEN_LOOP:
                    return;
                case VELOCITY_SETPOINT:
                    return;
                case PATH_FOLLOWING:
                    if (mPathFollower != null) {
                        updatePathFollower(timestamp);
                        mCSVWriter.add(mPathFollower.getDebug());
                    }
                    return;
                case TURN_TO_HEADING:
                    updateTurnToHeading(timestamp);
                    return;
                case PREPARE_TO_DEPLOY:
                	return;
                default:
                    System.out.println("Unexpected drive control state: " + mDriveControlState);
                    break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            mCSVWriter.flush();
        }
    };
    
    private Drive(){
    	leftMaster_ = new TalonSRX(Constants.kLeftDriveMasterId);
    	leftVictorSlave_ = new VictorSPX(Constants.kLeftDriveSlaveId);
    	
    	rightMaster_ = new TalonSRX(Constants.kRightDriveMasterId);
        rightVictorSlave_ = new VictorSPX(Constants.kRightDriveSlaveId);
        
        mNavXBoard = new NavX(SPI.Port.kMXP);
        shifter_ = new Solenoid(0);
        shifter_.set(false);
        // Get status at 100Hz
//        leftMaster_.setStatusFrameRatePeriod(StatusFrameRatePeriod.Feedback, 10);
//        rightMaster_.setStatusFrameRatePeriod(TalonSRX.StatusFrameRate.Feedback, 10);
        
        leftMaster_.set(ControlMode.PercentOutput, 0);
        leftVictorSlave_.follow(leftMaster_);
        
        rightMaster_.set(ControlMode.PercentOutput, 0);
        rightVictorSlave_.follow(rightMaster_);

        setBrakeMode(true);
        
     // Set up the encoders
        leftMaster_.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kDriveEncTimeoutMs);
        leftMaster_.setSensorPhase(true);
        leftMaster_.setInverted(false);
        
        leftVictorSlave_.setInverted(false);
        
        rightMaster_.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kDriveEncTimeoutMs);
        rightMaster_.setSensorPhase(false);
        rightMaster_.setInverted(false);

        rightVictorSlave_.setInverted(false);


        leftMaster_.configNominalOutputForward(0, Constants.kDriveTimeoutMs);
        leftMaster_.configNominalOutputReverse(0, Constants.kDriveTimeoutMs);
        leftMaster_.configPeakOutputForward(1, Constants.kDriveTimeoutMs);
        leftMaster_.configPeakOutputReverse(-1, Constants.kDriveTimeoutMs);

        rightMaster_.configNominalOutputForward(0, Constants.kDriveTimeoutMs);
        rightMaster_.configNominalOutputReverse(0, Constants.kDriveTimeoutMs);
        rightMaster_.configPeakOutputForward(1, Constants.kDriveTimeoutMs);
        rightMaster_.configPeakOutputReverse(-1, Constants.kDriveTimeoutMs);

        leftMaster_.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, 0);
        leftMaster_.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, 0);
        leftMaster_.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, 0);
        leftMaster_.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, 0);
        leftMaster_.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone, 0);
        leftMaster_.configClosedloopRamp(Constants.kDriveVelocityRampRate, 0);

        rightMaster_.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, 0);
        rightMaster_.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, 0);
        rightMaster_.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, 0);
        rightMaster_.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, 0);
        rightMaster_.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone, 0);
        rightMaster_.configClosedloopRamp(Constants.kDriveVelocityRampRate, 0);
//        velocityHeadingPid_ = new SynchronousPID(Constants.kDriveHeadingVelocityKp, Constants.kDriveHeadingVelocityKi,
//                Constants.kDriveHeadingVelocityKd);
//        velocityHeadingPid_.setOutputRange(-30, 30);
//        
        mCSVWriter = new ReflectingCSVWriter<PathFollower.DebugOutput>("/home/lvuser/PATH-FOLLOWER-LOGS.csv",
                PathFollower.DebugOutput.class);
        setOpenLoop(DriveSignal.NEUTRAL);
    }
    
    public void setHighGear(boolean high_gear){
    	shifter_.set(high_gear);
    	mIsHighGear = high_gear;
    }
    
    protected synchronized void setLeftRightPower(double left, double right) {
        leftMaster_.set(ControlMode.PercentOutput, left);
        rightMaster_.set(ControlMode.PercentOutput, -right);
    }
    
    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (driveControlState_ != DriveControlState.OPEN_LOOP) {
            leftMaster_.set(ControlMode.PercentOutput, 0);
            rightMaster_.set(ControlMode.PercentOutput, 0);
            driveControlState_ = DriveControlState.OPEN_LOOP;
        }
        setLeftRightPower(signal.leftMotor, signal.rightMotor);
    }
    
    public synchronized Rotation2d getGyroAngle() {
        return mNavXBoard.getYaw();
    }
    public synchronized double getGyroVelocityDegreesPerSec() {
        return mNavXBoard.getYawRateDegreesPerSec();
    }
    
    /**
     * Start up velocity mode. This sets the drive train in high gear as well.
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }
    
    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (usesTalonVelocityControl(mDriveControlState)) {
            final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
                    ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;
            leftMaster_.set(ControlMode.Velocity, inchesPerSecondToRpm(left_inches_per_sec * scale));
            rightMaster_.set(ControlMode.Velocity, inchesPerSecondToRpm(right_inches_per_sec * scale));
        } else {
            System.out.println("Hit a bad velocity control state");
            leftMaster_.set(ControlMode.PercentOutput, 0);
            rightMaster_.set(ControlMode.PercentOutput, 0);
        }
    }
    
    /**
     * Turn the robot to a target heading.
     * 
     * Is called periodically when the robot is auto-aiming towards the boiler.
     */
    private void updateTurnToHeading(double timestamp) {
        final Rotation2d field_to_robot = mRobotState.getLatestFieldToVehicle().getValue().getRotation();

        // Figure out the rotation necessary to turn to face the goal.
        final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);

        // Check if we are on target
        final double kGoalPosTolerance = 0.75; // degrees
        final double kGoalVelTolerance = 5.0; // inches per second
        if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance
                && Math.abs(getLeftVelocityInchesPerSec()) < kGoalVelTolerance
                && Math.abs(getRightVelocityInchesPerSec()) < kGoalVelTolerance) {
            // Use the current setpoint and base lock.
            mIsOnTarget = true;
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
            return;
        }

        Kinematics.DriveVelocity wheel_delta = Kinematics
                .inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
        updatePositionSetpoint(wheel_delta.left + getLeftDistanceInches(),
                wheel_delta.right + getRightDistanceInches());
    }
    /**
     * Adjust position setpoint (if already in position mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updatePositionSetpoint(double left_position_inches, double right_position_inches) {
        if (usesTalonPositionControl(mDriveControlState)) {
            leftMaster_.set(ControlMode.Velocity, inchesToRotations(left_position_inches));
            rightMaster_.set(ControlMode.Velocity, inchesToRotations(right_position_inches));
        } else {
            System.out.println("Hit a bad position control state");
            leftMaster_.set(ControlMode.PercentOutput, 0);
            rightMaster_.set(ControlMode.PercentOutput, 0);
        }
    }
    /**
     * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
     * pose, distance driven, and velocity, the updates the wheel velocity setpoints.
     */
    private void updatePathFollower(double timestamp) {
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose,
                RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
        if (!mPathFollower.isFinished()) {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updateVelocitySetpoint(setpoint.left, setpoint.right);
        } else {
            updateVelocitySetpoint(0, 0);
        }
    }
    
    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     * 
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            configureTalonsForSpeedControl();
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(
                            new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                    Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
                            Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                            Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                            Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                            Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
                            Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
                            Constants.kPathStopSteeringDistance));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setVelocitySetpoint(0, 0);
        }
    }
    
    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }
    
    double target = 12.0;
    public synchronized void handlePrepareToDeploy(double distance, boolean done) {
    	if (done) {
    		if(distance > target) { //move towards distance sensor; backwards
    			
    		}else if (distance < target) { // move away from distance sensor; forwards
    			
    		}else {// on target!
    			
    		}
    	}
    	
    }
    
    public void setMotors(double speed, double speed2){
    	leftMaster_.set(ControlMode.PercentOutput, speed);
    	rightMaster_.set(ControlMode.PercentOutput, -speed2);
    }
    /**
     * Path Markers are an optional functionality that name the various
     * Waypoints in a Path with a String. This can make defining set locations
     * much easier.
     * 
     * @return Set of Strings with Path Markers that the robot has crossed.
     */
//    public synchronized Set<String> getPathMarkersCrossed() {
//        if (driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
//            return null;
//        } else {
//            return pathFollowingController_.getMarkersCrossed();
//        }
//    }
   
    
    public synchronized void resetEncoders() {
    	leftMaster_.set(ControlMode.PercentOutput, 0);
    	rightMaster_.set(ControlMode.PercentOutput, 0);
        leftMaster_.setSelectedSensorPosition(0, 0, 0);
        rightMaster_.setSelectedSensorPosition(0, 0, 0);

    }
    private void configureTalonsForSpeedControl() {
        if (driveControlState_ != DriveControlState.VELOCITY_SETPOINT) {
            leftMaster_.set(ControlMode.Velocity, 0);
//            leftMaster_.setProfile(kVelocityControlSlot);
//            leftMaster_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
            rightMaster_.set(ControlMode.Velocity, 0);
//            rightMaster_.setProfile(kVelocityControlSlot);
//            rightMaster_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
//            setHighGear(true);
            setBrakeMode(true);
        }
    }
    public synchronized boolean isOnTarget() {
        // return true;
        return mIsOnTarget;
    }
    public double getLeftDistanceInches() {
        return rotationsToInches(leftMaster_.getSelectedSensorPosition(0));
    }

    public double getRightDistanceInches() {
        return rotationsToInches(rightMaster_.getSelectedSensorPosition(0));
    }
    
    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }
    
    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }
    
    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }
    
    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }
    
    public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(leftMaster_.getSelectedSensorVelocity(0));
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(rightMaster_.getSelectedSensorVelocity(0));
    }
    
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }
    

    public void setBrakeMode(boolean on) {
        if (isBrakeMode_ != on) {
            leftMaster_.setNeutralMode(NeutralMode.Brake);
            leftVictorSlave_.setNeutralMode(NeutralMode.Brake);
            rightMaster_.setNeutralMode(NeutralMode.Brake);
            rightVictorSlave_.setNeutralMode(NeutralMode.Brake);
            isBrakeMode_ = on;
        }
    }
    /**
     * VelocityHeadingSetpoints are used to calculate the robot's path given the
     * speed of the robot in each wheel and the polar coordinates. Especially
     * useful if the robot is negotiating a turn and to forecast the robot's
     * location.
     */
    
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

	public synchronized void setGyroAngle(Rotation2d angle) {
	        mNavXBoard.reset();
	        mNavXBoard.setAngleAdjustment(angle);
	}
	
	public void OutputToSmartDashboard() {
		final double left_speed = getLeftVelocityInchesPerSec();
        final double right_speed = getRightVelocityInchesPerSec();
		SmartDashboard.putNumber("left speed", left_speed);
		SmartDashboard.putNumber("right speed", right_speed);
		synchronized (this) {
            if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
                SmartDashboard.putNumber("drive CTE", mPathFollower.getCrossTrackError());
                SmartDashboard.putNumber("drive ATE", mPathFollower.getAlongTrackError());
            } else {
                SmartDashboard.putNumber("drive CTE", 0.0);
                SmartDashboard.putNumber("drive ATE", 0.0);
            }
        }
        SmartDashboard.putNumber("left position (rotations)", leftMaster_.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("right position (rotations)", rightMaster_.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("gyro vel", getGyroVelocityDegreesPerSec());
        SmartDashboard.putNumber("gyro pos", getGyroAngle().getDegrees());
        SmartDashboard.putBoolean("drive on target", isOnTarget());

	}
	

}