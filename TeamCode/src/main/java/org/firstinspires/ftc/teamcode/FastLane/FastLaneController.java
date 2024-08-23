package org.firstinspires.ftc.teamcode.FastLane;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import kotlin.NotImplementedError;

/**
 * Primary controller for the FastLane Path Following system.
 */
public class FastLaneController {

    private double robotRadius;
    private double obstacleBuffer;
    private Odometry odometry;
    private AutonomousWaypoint[] waypoints;
    /**
     * The point we are current tracking is index + 1.
     */
    private int index = 0;
    private FieldObstacle[] obstacles;

    private PIDFController headingController;

    private double maxDeceleration;

    private double[] movementVector = new double[]{0.0, 0.0, 0.0};

    private boolean done = false;

    /**
     * Prefix sum for path lengths to aid computation.
     */
    private double[] pathLengths;

    private enum colliding {
        STRICT,
        BOUNDARY,
        NONE
    }

    public FastLaneController(double robotRadius, double obstacleBuffer, double maxDeceleration, Odometry odometry, PIDFController headingController) {
        this.robotRadius = robotRadius;
        this.obstacleBuffer = obstacleBuffer;
        this.maxDeceleration = maxDeceleration;
        this.odometry = odometry;
        this.headingController = headingController;
    }

    public void setWaypoints(AutonomousWaypoint... waypoints) {
        this.waypoints = waypoints;
        this.pathLengths = new double[waypoints.length];
        this.done = false;

        pathLengths[0] = 0;
        for (int i = 1; i < pathLengths.length; i++) {
            pathLengths[i] = pathLengths[i-1] + Point.distance(waypoints[i].getPoint(), waypoints[i-1].getPoint());
        }
    }

    public void setObstacles(FieldObstacle... obstacles) {
        this.obstacles = obstacles;
    }

    public void updateHeadingPIDF(double kP, double kI, double kD, double kF) {
        headingController.setPIDF(kP, kI, kD, kF);
    }

    /**
     * @return A double[] of length 3, which contains x, y, and heading vectors.
     */
    public double[] getMovementVector() {
        return movementVector;
    }

    public boolean isDone() {
        return this.done;
    }

    public void update() {
        // prevent oob errors
        if (index + 1 < waypoints.length) {
            // if end of path, don't increment index, don't project pose
            if (index + 1 == waypoints.length - 1) {
                if (this.waypoints[index + 1].exit(odometry.getPose())) {
                   done = true;
                }
                // else, increment index and go next point
            } else {
                if (this.waypoints[index + 1].exit(odometry.getProjectedPose())) {
                    index++;
                }
            }
        }

        // find optimal t value
        Pose2d robotPose = odometry.getProjectedPose();
        Vector2d robotVector = new Vector2d(robotPose.getX(), robotPose.getY());
        double optimalt = 0;
        double optimalboundaryt = 0;
        boolean hasPath = false;
        boolean hasStrictPath = false;
        // iterate backwards checking collisions and breaking as soon as none occur
        // need to store: highest t with no collisions, highest t with only boundary collisions
        // if only strict collisions, move away from nearest obstacle at low power
        for (double t = 1; t >= 0; t -= 0.1) {
            colliding result = checkCollisions(robotVector,
                    FastLaneController.lerp(waypoints[index].getPoint().toVector2d(), waypoints[index+1].getPoint().toVector2d(), t));

            // if no collisions, break now
            if (result == colliding.NONE) {
                optimalt = t;
                hasPath = true;
                hasStrictPath = true;
                break;
                // else if only boundary collisions, save the value but do not break
            } else if (result == colliding.BOUNDARY) {
                optimalboundaryt = Math.max(optimalboundaryt, t);
                hasPath = true;
            }


        }
        // the robot has not found a path and we should stop (something is blocking t=0 so we shouldn't go that way either)
        // TODO: change this to a movement case
        if (!hasPath) {
            throw new RuntimeException("the robot can't find a path");w
        }


        if (hasStrictPath) {
            // search forwards to find the fastest path
            for (double t = optimalt + 0.1; t >= optimalt; t -= 0.01) {
                colliding result = checkCollisions(robotVector,
                        FastLaneController.lerp(waypoints[index].getPoint().toVector2d(), waypoints[index + 1].getPoint().toVector2d(), t));
                if (result == colliding.NONE) {
                    optimalt = t;
                    break;
                }
            }
        } else if (hasPath) {
            // search backwards to find the safest (lowest t value) path; if there is a strict path found that will be used instead
            for (double t = optimalboundaryt; t >= optimalboundaryt - 0.1; t -= 0.01) {
                colliding result = checkCollisions(robotVector,
                        FastLaneController.lerp(waypoints[index].getPoint().toVector2d(), waypoints[index + 1].getPoint().toVector2d(), t));
                if (result == colliding.NONE) {
                    optimalt = t;
                    hasStrictPath = true;
                    break;
                } else if (result == colliding.BOUNDARY) {
                    // useless math.min call that i don't want to get rid of for paranoia reasons
                    optimalboundaryt = Math.min(optimalboundaryt, t);
                }
            }
        }

        if (!hasStrictPath) {
            optimalt = optimalboundaryt;
        }

        // get requisite powers
        Vector2d targetVector = FastLaneController.lerp(waypoints[index].getPoint().toVector2d(), waypoints[index+1].getPoint().toVector2d(), optimalt);
        double targetHeading = waypoints[index+1].getPoint().heading;

        // normalized vector from robot to target
        Vector2d normalized = targetVector.minus(robotVector).normalize();

        // end of path logic
        // dist is the remaining distance to travel over all paths (robot -> path + remaining path lines)
        double dist = targetVector.minus(robotVector).magnitude()
                + waypoints[index+1].getPoint().toVector2d().minus(targetVector).magnitude() + pathLengths[pathLengths.length - 1] - pathLengths[index];
        /*
            Derivation of Math.sqrt(dist * maxDeceleration):
            v = at -> t = v/a
            d = vt = v * v/a = v^2/a
            therefore v^2 = ad -> v = sqrt(ad)
            thus sqrt(ad) is the maximum valid speed to stop in a distance d with max deceleration a (ignoring friction)
         */
        normalized.scale(Math.min(1, Math.sqrt(dist * maxDeceleration)));

        // add heading prio here if necessary
        movementVector = new double[]{normalized.getX(), normalized.getY(), headingController.calculate(robotPose.getRotation().getRadians(), targetHeading)};
    }

    /**
     * @return A boolean representing if the line between a and b collides with the obstacle map. True = collides.
     */
    private colliding checkCollisions(Vector2d a, Vector2d b) {
        boolean isBoundary = false;
        for (FieldObstacle obstacle : this.obstacles) {
            if (obstacle.isColliding(a, b, robotRadius)) {
                return colliding.STRICT;
            } else if (obstacle.isColliding(a, b, robotRadius + obstacleBuffer)) {
                isBoundary = true;
            }
        }
        return isBoundary ? colliding.BOUNDARY : colliding.NONE;
    }

    private static Vector2d lerp(Vector2d a, Vector2d b, double t) {
        return new Vector2d(a.scale(1-t).plus(b.scale(t)));
    }
}
