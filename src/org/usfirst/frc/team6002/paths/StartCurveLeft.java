package org.usfirst.frc.team6002.paths;

import java.util.ArrayList;

import org.usfirst.frc.team6002.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team6002.lib.util.control.Path;
import org.usfirst.frc.team6002.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team6002.lib.util.math.Rotation2d;
import org.usfirst.frc.team6002.lib.util.math.Translation2d;

public class StartCurveLeft implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,160,0,0));
        sWaypoints.add(new Waypoint(40,160,0,10));
        sWaypoints.add(new Waypoint(60,160,0,10));
        sWaypoints.add(new Waypoint(80,180,0,10));
        sWaypoints.add(new Waypoint(100,200,0,10));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(20, 160), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":20,"y":160},"speed":0,"radius":0,"comment":""},{"position":{"x":40,"y":160},"speed":60,"radius":0,"comment":""},{"position":{"x":60,"y":160},"speed":60,"radius":0,"comment":""},{"position":{"x":80,"y":180},"speed":60,"radius":0,"comment":""},{"position":{"x":100,"y":200},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: UntitledPath
}