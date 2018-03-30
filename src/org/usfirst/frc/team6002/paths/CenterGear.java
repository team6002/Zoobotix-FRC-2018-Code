package org.usfirst.frc.team6002.paths;

import org.usfirst.frc.team6002.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team6002.lib.util.control.Path;
import org.usfirst.frc.team6002.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team6002.lib.util.math.Rotation2d;
import org.usfirst.frc.team6002.lib.util.math.Translation2d;

import java.util.ArrayList;

/**
 * Path from the blue alliance wall to the blue center peg.
 * 
 * Used in CenterGearToShootBlue
 * 
 * @see CenterGearToShootBlue
 * @see PathContainer
 */
public class CenterGear implements PathContainer {

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(16, 160, 0, 0));
        sWaypoints.add(new Waypoint(17, 160, 0, 10));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(16, 160), Rotation2d.fromDegrees(180.0));
    }

    @Override
    public boolean isReversed() {
        return true;
    }
    // WAYPOINT_DATA:
    // [{"position":{"x":16,"y":160},"speed":0,"radius":0,"comment":""},{"position":{"x":90,"y":160},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: true
    // FILE_NAME: StartToCenterGearBlue
}