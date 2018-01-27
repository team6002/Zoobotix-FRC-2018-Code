package org.usfirst.frc.team6002.auto.modes;

import org.usfirst.frc.team6002.auto.AutoModeBase;
import org.usfirst.frc.team6002.auto.AutoModeEndedException;
import org.usfirst.frc.team6002.auto.actions.DrivePathAction;
import org.usfirst.frc.team6002.auto.actions.ResetPoseFromPathAction;
import org.usfirst.frc.team6002.auto.actions.WaitAction;
import org.usfirst.frc.team6002.paths.ForwardSomeDistance;
import org.usfirst.frc.team6002.paths.PathContainer;


/**
 * Scores the preload gear onto the center peg then shoots the 10 preloaded fuel
 * 
 * @see AutoModeBase
 */
public class DriveForwardMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        PathContainer gearPath = new ForwardSomeDistance();
//        runAction(new ResetPoseFromPathAction(gearPath));
//        runAction(new DrivePathAction(gearPath));
        runAction(new WaitAction(15));
 
    }
}