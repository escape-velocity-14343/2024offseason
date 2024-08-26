package org.firstinspires.ftc.teamcode.FastLane;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.SparkFunOTOS.Pose2D;

@Config
public class OTOSLocalizer implements Odometry {
    public static double projectedScalar = 0.1;

    private SparkFunOTOS otos;
    private Pose2d lastPose;
    private Pose2d velocity;
    private ElapsedTime time;
    private double deltaSeconds;
    private boolean firstUpdate;
    private Point offset;

    private OTOSPoseUtil poseUtil = new OTOSPoseUtil();

    public OTOSLocalizer(HardwareMap hwm) {
        otos = hwm.get(SparkFunOTOS.class, "otos");
        otos.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        otos.setAngularUnit(SparkFunOTOS.AngularUnit.RADIANS);
        otos.setOffset(new SparkFunOTOS.Pose2D(4,1.5,Math.toRadians(90.5)));
        otos.setAngularScalar(360/364.0);
        otos.calibrateImu();
        otos.resetTracking();
        lastPose = new Pose2d();
        velocity = new Pose2d();
        firstUpdate = true;
        deltaSeconds = 0.05;
        offset = new Point(0, 0, 0);
        time = new ElapsedTime();
    }

    @Override
    public void update() {

        if (!firstUpdate) {
            deltaSeconds = time.time();
        } else {
            firstUpdate = false;
        }

        Pose2d pos = poseUtil.toPose2d(otos.getPosition());
        Vector2d velvec = Point.fromPose2d(pos).toVector2d().minus(Point.fromPose2d(lastPose).toVector2d()).scale(1/deltaSeconds);
        velocity = new Pose2d(velvec.getX(), velvec.getY(), new Rotation2d((pos.getRotation().getRadians() - lastPose.getRotation().getRadians())/deltaSeconds));
        lastPose = pos;

        time.reset();
    }

    @Override
    public void reset() {
        reset(0, 0, 0);
    }

    @Override
    public void reset(double x, double y, double heading) {
        otos.resetTracking();
        offset = new Point(x, y, heading);
    }

    @Override
    public void reset(Pose2d pose) {
        reset(pose.getX(), pose.getY(), pose.getRotation().getRadians());
    }

    @Override
    public Pose2d getPose() {
        return Point.fromPose2d(lastPose).offset(offset).toPose2d();
    }

    @Override
    public Pose2d getVelocity() {
        return velocity;
    }

    @Override
    public Pose2d getProjectedPose() {
        Pose2d vel = new Pose2d(velocity.getX() * velocity.getX(), velocity.getY() * velocity.getY(),
                new Rotation2d());
        Vector2d vec = Point.fromPose2d(getPose()).toVector2d().plus(Point.fromPose2d(vel).toVector2d().scale(projectedScalar));
        return new Point(vec.getX(), vec.getY(), getPose().getRotation().getRadians() + velocity.getRotation().getRadians() * velocity.getRotation().getRadians() * projectedScalar).toPose2d();
    }

    private class OTOSPoseUtil {
        private OTOSPoseUtil() {}

        private Pose2d toPose2d(Pose2D pose) {
            return new Pose2d(pose.x, pose.y, new Rotation2d(pose.h));
        }

        private Pose2D fromPose2d(Pose2d pose) {
            return new Pose2D(pose.getX(), pose.getY(), pose.getRotation().getRadians());
        }
    }

}
