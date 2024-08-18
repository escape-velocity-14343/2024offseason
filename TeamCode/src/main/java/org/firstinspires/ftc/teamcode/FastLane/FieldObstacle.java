package org.firstinspires.ftc.teamcode.FastLane;

import com.arcrobotics.ftclib.geometry.Vector2d;

/**
 * A field obstacle. Hitbox is a circle.
 */
public class FieldObstacle {
    /**
     * X coordinate of the center.
     */
    private double x;
    /**
     * Y coordinate of the center.
     */
    private double y;
    /**
     * Radius.
     */
    private double r;

    public FieldObstacle(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    // thank you stackoverflow https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
    public boolean isColliding(Vector2d a, Vector2d b, double robotR) {
        Vector2d center = new Vector2d(x, y);
        // normalize everything
        center = center.minus(center);
        a = a.minus(center);
        b = b.minus(center);
        Vector2d ab = b.minus(a);
        Vector2d ac = center.minus(a);
        Vector2d d = ac.project(ab).plus(a);
        Vector2d ad = d.minus(a);

        double k = Math.abs(ab.getX()) > Math.abs(ab.getY()) ? ad.getX() / ab.getX() : ad.getY() / ad.getY();

        if (k >= 1) {
            return b.magnitude() < r + robotR;
        } else if (k <= 0) {
            return a.magnitude() < r + robotR;
        }

        return d.magnitude() < r + robotR;

    }
}
