package org.firstinspires.ftc.teamcode.FastLane;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

// TODO: custom exit conditions (use lambdas)

/**
 * Waypoint class for autonomous navigation. Represents a single 3D point (x, y, heading).<br>
 * Has built-in exit condition checking (tolerance, timeout), with support for additional
 * exit conditions.
 */
@Config
public class AutonomousWaypoint {

    public static double DEFAULT_TOLERANCE = 0.5;
    /**
     * Default heading tolerance. In radians.
     */
    public static double DEFAULT_HTOLERANCE = Math.toRadians(5);

    public static double DEFAULT_STRICT_TOLERANCE = 0.1;
    /**
     * Default strict heading tolerance. In radians.
     */
    public static double DEFAULT_STRICT_HTOLERANCE = Math.toRadians(1);

    /**
     * Default timeout. Set to -1 for infinity.
     */
    private static double DEFAULT_TIMEOUT = -1;

    protected Point waypoint;
    protected double tolerance;
    protected double headingTolerance;

    private ElapsedTime timeoutTimer;
    /**
     * Time for the robot to give up on reaching the point. In seconds.
     */
    private double timeout;

    public static AutonomousWaypoint fromRobotCentric(Point movement, Pose2d robotPose) {
        return new AutonomousWaypoint(Point.fromPose2d(robotPose).offset(movement).setHeading(movement.heading));
    }

    public static AutonomousWaypoint fromInvertibleWaypoint(InvertibleWaypoint waypoint) {
        return new AutonomousWaypoint(waypoint.getPoint());
    }

    public AutonomousWaypoint setTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public AutonomousWaypoint setHeadingTolerance(double headingTolerance) {
        this.headingTolerance = headingTolerance;
        return this;
    }

    public AutonomousWaypoint setTolerances(double tolerance, double headingTolerance) {
        setTolerance(tolerance);
        setHeadingTolerance(headingTolerance);
        return this;
    }

    /**
     * Sets tolerances to default strict tolerances. Will overwrite custom tolerances.
     */
    public AutonomousWaypoint setStrict() {
        this.tolerance = DEFAULT_STRICT_TOLERANCE;
        this.headingTolerance = DEFAULT_STRICT_HTOLERANCE;
        return this;
    }

    public void setTimeout(double timeout) {
        this.timeout = timeout;
    }

    /**
     * Shorthand for setTimeout(-1). Sets timeout time to infinity.
     */
    public void setForever() {
        setTimeout(-1);
    }

    /**
     * Call when you begin targeting the waypoint. Ensures timeout works properly.
     */
    public void beginTracking() {
        this.timeoutTimer.reset();
    }

    public boolean exit(Pose2d robotPose, Pose2d robotVelocity) {
        if (timeoutTimer.seconds() > timeout && timeout >= 0) {
            return true;
        }

        Point robotPos = Point.fromPose2d(robotPose);
        if (Math.abs(waypoint.heading - robotPos.heading) < headingTolerance
                && Point.distance(waypoint, robotPos) < tolerance
               //&& Point.distance(Point.fromPose2d(robotVelocity), new Point(0, 0, 0)) < tolerance)
        ){
            return true;
        }

        return false;
    }

    /**
     * Returns the vector from the robot to the waypoint, from your robot's perspective.
     * @return A robot-centric differential Pose2d where X is forward and Y is left.
     */
    public Pose2d getRobotCentricVector(Pose2d robotPose) {
        Pose2d fcpose = getFieldCentricVector(robotPose);
        double x = fcpose.getX();
        double y = fcpose.getY();

        // reverse heading to get correct vector
        double h = -robotPose.getRotation().getRadians();

        // rotate by -h (keep heading the same in the end)
        return new Pose2d(Math.cos(h)*x - Math.sin(h)*y, Math.sin(h)*x + Math.cos(h)*y, new Rotation2d(-h));
    }

    /**
     * Returns the vector from the robot to the waypoint, in a field-centric perspective.
     */
    public Pose2d getFieldCentricVector(Pose2d robotPose) {
        return waypoint.difference(Point.fromPose2d(robotPose)).toPose2d();
    }

    /**
     * Get the location of the waypoint, from a field-centric perspective.
     */
    public Point getPoint() {
        return waypoint;
    }

    /**
     * Internal constructor. Builds an AutonomousWaypoint based off a single point. Keep all other constructors referencing this one.
     */
    protected AutonomousWaypoint(Point p) {
        this.waypoint = p;
        this.tolerance = DEFAULT_TOLERANCE;
        this.headingTolerance = DEFAULT_HTOLERANCE;
        this.timeout = DEFAULT_TIMEOUT;
        this.timeoutTimer = new ElapsedTime();
    }
}
