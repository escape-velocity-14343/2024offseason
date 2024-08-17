package org.firstinspires.ftc.teamcode.FastLane;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

// TODO: custom exit conditions (use lambdas), turn to point

/**
 * Waypoint class for autonomous navigation. Represents a single 3D point (x, y, heading).<br>
 * Has built-in exit condition checking (tolerance, timeout), with support for additional
 * exit conditions.
 */
public class AutonomousWaypoint {

    private static double DEFAULT_TOLERANCE = 0.5;
    /**
     * Default heading tolerance. In radians.
     */
    private static double DEFAULT_HTOLERANCE = Math.toRadians(5);

    private static double DEFAULT_STRICT_TOLERANCE = 0.1;
    /**
     * Default strict heading tolerance. In radians.
     */
    private static double DEFAULT_STRICT_HTOLERANCE = Math.toRadians(1);

    /**
     * Default timeout. Set to -1 for infinity.
     */
    private static double DEFAULT_TIMEOUT = -1;

    private Point waypoint;
    private double tolerance;
    private double headingTolerance;

    private ElapsedTime timeoutTimer;
    /**
     * Time for the robot to give up on reaching the point. In seconds.
     */
    private double timeout;

    public static AutonomousWaypoint getRobotCentric(Point movement, Pose2d robotPose) {
        return new AutonomousWaypoint(Point.fromPose2d(robotPose).offset(movement));
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
     * Call when you begin targetting the waypoint. Ensures timeout works properly.
     */
    public void beginTracking() {
        this.timeoutTimer.reset();
    }

    public boolean exit(Pose2d robotPose) {
        if (timeoutTimer.seconds() > timeout && timeout >= 0) {
            return true;
        }

        Point robotPos = Point.fromPose2d(robotPose);
        if (Math.abs(waypoint.heading - robotPos.heading) < headingTolerance && Point.distance(waypoint, robotPos) < tolerance) {
            return true;
        }

        return false;
    }


    private AutonomousWaypoint(Point p) {
        this.waypoint = p;
        this.tolerance = DEFAULT_TOLERANCE;
        this.headingTolerance = DEFAULT_HTOLERANCE;
        this.timeout = DEFAULT_TIMEOUT;
        this.timeoutTimer = new ElapsedTime();
    }
}
