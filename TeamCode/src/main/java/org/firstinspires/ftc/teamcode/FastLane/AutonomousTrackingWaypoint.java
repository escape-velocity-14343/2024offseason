package org.firstinspires.ftc.teamcode.FastLane;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

/**
 * Turn to point extension of AutonomousWaypoint.
 */
public class AutonomousTrackingWaypoint extends AutonomousWaypoint {
    private Point trackingPoint;

    public static AutonomousTrackingWaypoint fromInvertibleWaypoint(InvertibleWaypoint waypoint, Point trackingPoint) {
        return new AutonomousTrackingWaypoint(waypoint.getPoint(), trackingPoint);
    }

    public static AutonomousTrackingWaypoint fromRobotCentric(Point movement, Point trackingPoint, Pose2d robotPose) {
        return new AutonomousTrackingWaypoint(Point.fromPose2d(robotPose).difference(movement).setHeading(movement.heading), trackingPoint);
    }

    public AutonomousTrackingWaypoint(Point p, Point trackingPoint) {
        super(p);
        this.trackingPoint = trackingPoint;
        // prevent heading from exiting
        this.waypoint.setHeading(Double.NEGATIVE_INFINITY);
    }

    // TODO: this is broken and exit condition does not work as intended.

    /**
     * CURRENTLY BROKEN DO NOT USE
     */
    @Override
    public boolean exit(Pose2d robotPose, Pose2d robotVelocity) {
        return super.exit(robotPose, robotVelocity) || Math.abs(robotPose.getRotation().getRadians() - getTargetHeading(robotPose)) < headingTolerance;
    }

    /**
     * Returns the vector from the robot to the waypoint, from your robot's perspective.
     * @return A robot-centric differential Pose2d where X is forward and Y is left.
     */
    @Override
    public Pose2d getRobotCentricVector(Pose2d robotPose) {
        Pose2d fcpose = getFieldCentricVector(robotPose);
        double x = fcpose.getX();
        double y = fcpose.getY();

        // reverse heading to get correct vector
        double h = -robotPose.getRotation().getRadians();

        // rotate by -h (keep heading the same in the end)
        return new Pose2d(Math.cos(h)*x - Math.sin(h)*y, Math.sin(h)*x + Math.cos(h)*y, new Rotation2d(getTargetHeading(robotPose)));
    }

    /**
     * Returns the vector from the robot to the waypoint, in a field-centric perspective.
     */
    @Override
    public Pose2d getFieldCentricVector(Pose2d robotPose) {
        Pose2d vector = waypoint.difference(Point.fromPose2d(robotPose)).toPose2d();
        vector.rotate(getTargetHeading(robotPose) - vector.getRotation().getRadians());
        return vector;
    }

    public Point getTrackingPoint() {
        return this.trackingPoint;
    }

    /**
     * Gets the angle of the vector from the robot to the tracking point.
     * @return Target heading, in radians.
     */
    private double getTargetHeading(Pose2d robotPose) {
        Point diff = trackingPoint.difference(Point.fromPose2d(robotPose));
        return Math.atan2(diff.y, diff.x);
    }
}
