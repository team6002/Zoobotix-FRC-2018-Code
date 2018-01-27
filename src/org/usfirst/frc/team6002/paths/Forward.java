package org.usfirst.frc.team6002.paths;

import java.util.ArrayList;

import org.usfirst.frc.team6002.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team6002.lib.util.control.Path;
import org.usfirst.frc.team6002.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team6002.lib.util.math.Rotation2d;
import org.usfirst.frc.team6002.lib.util.math.Translation2d;

public class Forward implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(50,50,0,0));
        sWaypoints.add(new Waypoint(70,70,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(50, 50), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":50,"y":50},"speed":0,"radius":0,"comment":""},{"position":{"x":70,"y":70},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: UntitledPath
}