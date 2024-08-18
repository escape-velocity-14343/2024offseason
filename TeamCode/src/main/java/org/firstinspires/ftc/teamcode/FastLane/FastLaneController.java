package org.firstinspires.ftc.teamcode.FastLane;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import kotlin.NotImplementedError;

// TODO: collision detection

/**
 * Primary controller for the FastLane Path Following system.
 */
public class FastLaneController {

    private double robotRadius;
    private Odometry odometry;
    private AutonomousWaypoint[] waypoints;
    private int index = 0;
    private FieldObstacle[] obstacles;

    private PIDFController headingController;

    private double[] movementVector = new double[]{0.0, 0.0, 0.0};

    public FastLaneController(double robotRadius, Odometry odometry, PIDFController headingController) {
        this.robotRadius = robotRadius;
        this.odometry = odometry;
        this.headingController = headingController;
    }

    public void setWaypoints(AutonomousWaypoint... waypoints) {
        this.waypoints = waypoints;
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

    public void update() {
        if (index + 1 != waypoints.length) {
            if (this.waypoints[index + 1].exit(odometry.getProjectedPose())) {
                index++;
                if (index + 1 == waypoints.length) {
                    // TODO: add end state feedback
                }
            }
        }

        // find optimal t value
        Pose2d robotPose = odometry.getProjectedPose();
        double optimalt = 0;
        for (double t = 1; t >= 0; t -= 0.1) {
            if (!checkCollisions(new Vector2d(robotPose.getX(), robotPose.getY()),
                    FastLaneController.lerp(waypoints[index].getPoint().toVector2d(), waypoints[index+1].getPoint().toVector2d(), t))) {
                optimalt = t;
                break;
            }
            if (t < 0.05) {
                throw new RuntimeException("the robot can't find a path");
            }
        }
        for (double t = optimalt + 0.1; t >= optimalt; t -= 0.01) {
            if (!checkCollisions(new Vector2d(robotPose.getX(), robotPose.getY()),
                    FastLaneController.lerp(waypoints[index].getPoint().toVector2d(), waypoints[index+1].getPoint().toVector2d(), t))) {
                optimalt = t;
                break;
            }
        }

        // get requisite powers
        Vector2d targetVector = FastLaneController.lerp(waypoints[index].getPoint().toVector2d(), waypoints[index+1].getPoint().toVector2d(), optimalt);
        double targetHeading = waypoints[index+1].getPoint().heading;

        Vector2d normalized = targetVector.normalize();

        // add heading prio here if necessary
        movementVector = new double[]{normalized.getX(), normalized.getY(), headingController.calculate(robotPose.getRotation().getRadians(), targetHeading)};
    }

    /**
     * @return A boolean representing if the line between a and b collides with the obstacle map. True = collides.
     */
    private boolean checkCollisions(Vector2d a, Vector2d b) {
        for (FieldObstacle obstacle : this.obstacles) {
            if (obstacle.isColliding(a, b, robotRadius)) {
                return true;
            }
        }
        return false;
    }

    private static Vector2d lerp(Vector2d a, Vector2d b, double t) {
        return new Vector2d(a.scale(1-t).plus(b.scale(t)));
    }
}
