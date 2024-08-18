package org.firstinspires.ftc.teamcode.FastLane;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class InvertibleWaypoint {

    private static boolean isRed = true;
    private static boolean isBackstage = true;

    /**
     * For points that need less precision (ex a point that is a control point but not an end point on the path)
     */
    public static double DEFAULT_IMPRECISE_TOLERANCE = 2;
    public static double DEFAULT_IMPRECISE_HEADING_TOLERANCE = 0.4;
    private static Point GLOBAL_AUDIENCE_OFFSET = new Point(0, 0, 0);
    private static Point GLOBAL_BLUE_OFFSET = new Point(0, 0, 0);

    private Point redBackstagePoint;

    /**
     * This variable exists so we can account for different starting positions and still have individual offsets for the audience autos.
     */
    private Point audienceOffset = new Point(0, 0, 0);

    /**
     * This variable exists so we can account for different starting positions and still have individual offsets for the blue autos.
     */
    private Point blueOffset = new Point(0, 0, 0);

    private boolean blueHeadingIsReversed = false;

    /**
     * Offsets for different run positions.
     * Indexed in this order: Red Audience, Blue Backstage, Blue Audience
     */
    ArrayList<Point> offsets = new ArrayList<>(Arrays.asList(new Point(0, 0, 0), new Point(0, 0, 0), new Point(0, 0, 0)));

    public static void configAuto(boolean isRed, boolean isBackstage) {
        InvertibleWaypoint.isRed = isRed;
        InvertibleWaypoint.isBackstage = isBackstage;
    }

    /**
     * Apply for Red Audience coordinates.
     */
    public static void setGlobalAudienceOffset(double x, double y, double rot) {
        InvertibleWaypoint.GLOBAL_AUDIENCE_OFFSET = new Point(x, y, rot);
    }

    /**
     * Regular offset. Apply from FC coordinates.
     */
    public static void setGlobalBlueOffset(double x, double y, double rot) {
        InvertibleWaypoint.GLOBAL_BLUE_OFFSET = new Point(x, y, rot);
    }

    public InvertibleWaypoint(double x, double y, double rot) {
        redBackstagePoint = new Point(x, y, rot);
    }

    public InvertibleWaypoint(Point p) {
        this(p.x, p.y, p.heading);
    }

    /**
     * Apply for Red Audience coordinates.
     */
    public InvertibleWaypoint setAudienceOffset(double x, double y, double rot) {
        audienceOffset = new Point(x, y, rot);
        return this;
    }

    /**
     * Apply for Red Audience coordinates.
     */
    public InvertibleWaypoint setRedAudienceOffset(double x, double y, double rot) {
        offsets.set(0, new Point(x, y, rot));
        return this;
    }

    /**
     * Regular offset. Apply from FC coordinates.
     */
    public InvertibleWaypoint setBlueAudienceOffset(double x, double y, double rot) {
        offsets.set(2, new Point(x, y, rot));
        return this;
    }

    /**
     * Regular offset. Apply from FC coordinates.
     */
    public InvertibleWaypoint setBlueBackstageOffset(double x, double y, double rot) {
        offsets.set(1, new Point(x, y, rot));
        return this;
    }

    public InvertibleWaypoint setBlueHeadingReversed() {
        blueHeadingIsReversed = true;
        return this;
    }

    /**
     * Does not overwrite other configured offsets for blue audience/backstage.
     * Regular offset. Apply from FC coordinates.
     */
    public InvertibleWaypoint setBlueOffset(double x, double y, double rot) {
        blueOffset = new Point(x, y, rot);
        return this;
    }

    public Point getPoint() {
        return rawGetPoint();
    }

    private Point rawGetPoint() {
        Point out = new Point(redBackstagePoint.x, redBackstagePoint.y, redBackstagePoint.heading);
        if (!InvertibleWaypoint.isRed) {
            out = out.reverseY();
            if (blueHeadingIsReversed) {
                out = out.reverseHeading();
            }
            out.offset(blueOffset);
            out.offset(InvertibleWaypoint.GLOBAL_BLUE_OFFSET);
            if (InvertibleWaypoint.isBackstage) {
                out.offset(offsets.get(1));
            } else {
                out.offset(InvertibleWaypoint.GLOBAL_AUDIENCE_OFFSET.reverseY());
                out.offset(audienceOffset.reverseY());
                out.offset(offsets.get(2));
            }
        } else {
            if (!InvertibleWaypoint.isBackstage) {
                out.offset(audienceOffset);
                out.offset(InvertibleWaypoint.GLOBAL_AUDIENCE_OFFSET);
                out.offset(offsets.get(0));
            }
        }
        return out;
    }

    public AutonomousWaypoint toAutonomousWaypoint() {
        return AutonomousWaypoint.fromInvertibleWaypoint(this);
    }

    public AutonomousTrackingWaypoint toAutonomousTrackingWaypoint(Point trackingPoint) {
        return new AutonomousTrackingWaypoint(this.getPoint(), trackingPoint);
    }

    /**
     * Euclidean distance between two InvertibleWaypoints.
     */
    public static double distance(InvertibleWaypoint waypoint1, InvertibleWaypoint waypoint2) {
        return Point.distance(waypoint1.getPoint(), waypoint2.getPoint());
    }

    /**
     * Euclidean distance between the robot and an InvertibleWaypoint.
     */
    public static double distance(Pose2d robotPose, InvertibleWaypoint waypoint) {
        return Point.distance(new Point(robotPose.getX(), robotPose.getY(), 0), waypoint.getPoint());
    }
}