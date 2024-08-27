package org.firstinspires.ftc.teamcode.FastLane;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AbstractLocalizer {

    public static double velocityLowPassGain = 0.1;

    // TODO: move these constants to the DT
    public static double xFricDeceleration = 40;
    public static double yFricDeceleration = 40;
    public static double hFricDeceleration = Math.toRadians(180);

    protected Pose2d lastPose;
    protected Pose2d velocity;
    protected ElapsedTime time;
    protected double deltaSeconds;
    protected boolean firstUpdate;
    protected Point offset;

    /**
     * Default constructor. You MUST create a public-facing constructor with the necessary localizer parameters.
     */
    protected AbstractLocalizer() {
        lastPose = new Pose2d();
        velocity = new Pose2d();
        firstUpdate = true;
        deltaSeconds = 0.05;
        offset = new Point(0, 0, 0);
        time = new ElapsedTime();
    }

    /**
     * Method to fetch the current robot pose from the localizer. Called in the <code>update()</code> cycle.
     * @return The current position, as an FTCLib Pose2d.
     */
    protected abstract Pose2d internalGetPose();

    /**
     * Method to fetch the current robot velocity from the localizer. Called in the <code>update()</code> cycle.
     * <br>If wishing to compute velocity from position, make this function return <code>getVelocityFromPose()</code>.
     * @return The current velocity, as an FTCLib Pose2d.
     */
    protected abstract Pose2d internalGetVelocity(Pose2d lastPose, Pose2d currentPose, double deltaSeconds);

    protected Pose2d getVelocityFromPose(Pose2d lastPose, Pose2d currentPose, double deltaSeconds) {
        Vector2d velvec = Point.fromPose2d(currentPose).toVector2d().minus(Point.fromPose2d(lastPose).toVector2d()).scale(1/deltaSeconds);
        return lowPassVelocity(new Pose2d(velvec.getX(), velvec.getY(), new Rotation2d((currentPose.getRotation().getRadians() - lastPose.getRotation().getRadians())/deltaSeconds)));
    }

    public void update() {
        if (!firstUpdate) {
            deltaSeconds = time.time();
        } else {
            firstUpdate = false;
        }

        Pose2d pos = internalGetPose();
        velocity = internalGetVelocity(lastPose, pos, deltaSeconds);
        lastPose = pos;

        time.reset();
    };

    public void reset() {
        reset(0, 0, 0);
    };

    /**
     * Default reset() function. Override this to modify all reset behavior.
     */
    public void reset(double x, double y, double heading) {
        offset = new Point(x, y, heading);
    };
    
    public void reset(Pose2d pose) {
        reset(pose.getX(), pose.getY(), pose.getRotation().getRadians());
    };

    public Pose2d getPose() {
        return Point.fromPose2d(lastPose).offset(offset).toPose2d();
    };
    
    public Pose2d getVelocity() {
        return velocity;
    };

    /**
     * @return The current pose, plus the forward velocity of the robot.
     */
    public Pose2d getProjectedPose() {
        Pose2d vel = new Pose2d(Math.signum(velocity.getX()) * velocity.getX() * velocity.getX() / 2 * xFricDeceleration,
                Math.signum(velocity.getY()) * velocity.getY() * velocity.getY() / 2 * yFricDeceleration,
                new Rotation2d());
        //Pose2d vel = velocity;
        //Vector2d vec = Point.fromPose2d(getPose()).toVector2d().plus(Point.fromPose2d(vel).toVector2d().scale(projectedScalar));
        return new Point(vel.getX() + getPose().getX(), vel.getY() + getPose().getY(),
                getPose().getRotation().getRadians()
                        + Math.signum(velocity.getRotation().getRadians()) * velocity.getRotation().getRadians() * velocity.getRotation().getRadians() / 2 * hFricDeceleration
        ).toPose2d();
    };

    /**
     * Runs an Exponentially-Weighted Moving Average low-pass filter on velocity.
     */
    // https://www.mcgurrin.info/robots/154/
    protected Pose2d lowPassVelocity(Pose2d deltaPos) {
        double x = velocity.getX() * velocityLowPassGain + deltaPos.getX() * (1-velocityLowPassGain);
        double y = velocity.getY() * velocityLowPassGain + deltaPos.getY() * (1-velocityLowPassGain);
        double h = velocity.getRotation().getRadians() * velocityLowPassGain + deltaPos.getRotation().getRadians() * (1-velocityLowPassGain);
        return new Pose2d(x, y, new Rotation2d(h));
    }

}
