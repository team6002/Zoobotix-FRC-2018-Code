package org.usfirst.frc.team6002.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import org.usfirst.frc.team6002.robot.RobotState;
import org.usfirst.frc.team6002.paths.PathContainer;
import org.usfirst.frc.team6002.robot.subsystems.Drive;
import org.usfirst.frc.team6002.lib.util.math.RigidTransform2d;

/**
 * Resets the robot's current pose based on the starting pose stored in the pathContainer object.
 * 
 * @see PathContainer
 * @see Action
 * @see RunOnceAction
 */
public class ResetPoseFromPathAction extends RunOnceAction {

    protected PathContainer mPathContainer;

    public ResetPoseFromPathAction(PathContainer pathContainer) {
        mPathContainer = pathContainer;
    }

    @Override
    public synchronized void runOnce() {
        RigidTransform2d startPose = mPathContainer.getStartPose();
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), startPose);
        Drive.getInstance().setGyroAngle(startPose.getRotation());
    }
}