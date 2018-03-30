package org.usfirst.frc.team6002.robot;

public class Constants {
	
	public static double kLooperDt = 0.01;
	public static int kTimeoutMs = 0;
	
	//DRIVE ID
	public static int kLeftDriveMasterId = 14;
	public static int kLeftDriveSlaveId = 15;
	public static int kRightDriveMasterId = 1;
	public static int kRightDriveSlaveId = 0;
	
	//INTAKE ID
	public static int kIntakeSolenoidId = 1;
	public static int kDeploySolenoidId = 2;
	public static int kLeftIntakeId = 13;
	public static int kRightIntakeId = 2;
	
	//ELEVATOR ID
	public static int kElevatorMasterId = 12; 
	public static int kElevatorSlaveId = 3;
	
	//ARM ID
	public static int kArmId = 4;
	public static int kClawSolenoidId = 5;
	public static int kPistonSolenoidId = 6;// SET TO CORRECT NUMBER
	public static int kLaserId = 0;
	
	//RAMP ID
	public static int kLeftRampId = 10;
	public static int kRightRampId = 5;
	
	//ELEVATOR PID
	public static int kElevatorPIDLoopId = 0;
	public static double kElevatorKf = 0.0;
	public static double kElevatorKp = 0.37;
	public static double kElevatorKi = 0.0;
	public static double kElevatorKd = 0.05;
		
	//Arm PID
	public static int kArmPIDLoopId = 0;
	public static double kArmKf = 0.0;
	public static double kArmKp = 2.5;
	public static double kArmKi = 0.0;
	public static double kArmKd = 0.0;
	
	//RAMP PID
	public static int kRampPIDLoopId = 0;
	public static double kRampKf = 0.0;
	public static double kRampKp = 0.0;
	public static double kRampKi = 0.0;
	public static double kRampKd = 0.0;
	
	
	//DRIVE PID
	public static int kDriveEncTimeoutMs = 10;
	public static int kDriveTimeoutMs = 0;
	public static double kDriveVelocityKf = 0.28;
	public static double kDriveVelocityKp = 0.0;
	public static double kDriveVelocityKi = 0.0;
	public static double kDriveVelocityKd = 0.0;
	
	public static int kDriveVelocityIZone = 0;
	public static double kDriveVelocityRampRate = 0.0;
	
	//SHOOTER PID
	public static int kShooterTimeOutMS = 10;
	public static double kFShooterVelocity = 2.8416;
	public static double kPShooterVelocity = 10;//50;
	public static double kIShooterVelocity = 0.0;
	public static double kDShooterVelocity = 0.0;
	
	//Wheels
	public static double kDriveWheelDiameterInches = 6;
	public static double kTrackScrubFactor = 0.5;
	public static double kTrackLengthInches = 8.265;
    public static double kTrackWidthInches = 23.8;
    public static double kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches
            + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
	

	//Elevator Specs
	public static double kElevatorDiameterInches = 1.66;
	
	
    
    //Compressor
    public static int kCompressorId = 0;
    
    //PWM
    public static int kClimberId = 1;
    public static int kClimber2Id = 4;//second climber on comp robot
    public static int kSerializerId = 0;
    public static int kConveyorId = 3;
    
    //SHOOTER and SERIALIZZER CONSTANTS
    public static double kSerializerVoltage = 0.6;
	public static double kShooterSpeed = 875;

	//CONVEYOR CONSTANTS
	public static double kConveyorVoltage = 0.45;
	
	//PATH FOLLOWING CONSTANTS
	// Path following constants
    public static double kMinLookAhead = 12.0; // inches
    public static double kMinLookAheadSpeed = 9.0; // inches per second
    public static double kMaxLookAhead = 24.0; // inches
    public static double kMaxLookAheadSpeed = 120.0; // inches per second
    public static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
                                                     // our speed
                                                     // in inches per sec
    public static double kSegmentCompletionTolerance = 0.1; // inches
 // Path following constants
    public static double kPathFollowingLookahead = 24.0; // inches
    public static double kPathFollowingMaxVel = 120.0; // inches/sec
    public static double kPathFollowingMaxAccel = 80.0; // inches/sec^2
    public static double kPathFollowingProfileKp = 5.00;
    public static double kPathFollowingProfileKi = 0.03;
    public static double kPathFollowingProfileKv = 0.02;
    public static double kPathFollowingProfileKffv = 1.0;
    public static double kPathFollowingProfileKffa = 0.05;
    public static double kPathFollowingGoalPosTolerance = 0.75;
    public static double kPathFollowingGoalVelTolerance = 12.0;
    public static double kPathStopSteeringDistance = 9.0;
    
 // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveHighGearVelocityKp = 1.2;
    public static double kDriveHighGearVelocityKi = 0.0;
    public static double kDriveHighGearVelocityKd = 6.0;
    public static double kDriveHighGearVelocityKf = .15;
    public static int kDriveHighGearVelocityIZone = 0;
    public static double kDriveHighGearVelocityRampRate = 240.0;
    public static double kDriveHighGearNominalOutput = 0.5;
    public static double kDriveHighGearMaxSetpoint = 17.0 * 12.0; // 17 fps

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveLowGearPositionKp = 1.0;
    public static double kDriveLowGearPositionKi = 0.002;
    public static double kDriveLowGearPositionKd = 100.0;
    public static double kDriveLowGearPositionKf = .45;
    public static int kDriveLowGearPositionIZone = 700;
    public static double kDriveLowGearPositionRampRate = 240.0; // V/s
    public static double kDriveLowGearNominalOutput = 0.5; // V
    public static double kDriveLowGearMaxVelocity = 6.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 6 fps
                                                                                                               // in RPM
    public static double kDriveLowGearMaxAccel = 18.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 18 fps/s
                                                                                                             // in RPM/s

    public static double kDriveVoltageCompensationRampRate = 0.0;
    // Goal tracker constants
    public static double kMaxGoalTrackAge = 1.0;
    public static double kMaxTrackerDistance = 18.0;
    public static double kCameraFrameRate = 30.0;
    public static double kTrackReportComparatorStablityWeight = 1.0;
    public static double kTrackReportComparatorAgeWeight = 1.0;
    
 // Pose of the camera frame w.r.t. the robot frame
    public static double kCameraXOffset = -3.3211;
    public static double kCameraYOffset = 0.0;
    public static double kCameraZOffset = 20.9;
    public static double kCameraPitchAngleDegrees = 29.56; // Measured on 4/26
    public static double kCameraYawAngleDegrees = 0.0;
    public static double kCameraDeadband = 0.0;
}
