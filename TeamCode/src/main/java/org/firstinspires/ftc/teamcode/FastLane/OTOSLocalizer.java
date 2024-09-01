package org.firstinspires.ftc.teamcode.FastLane;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.SparkFunOTOS.Pose2D;

@Config
public class OTOSLocalizer extends Localizer {

    private SparkFunOTOS otos;
    private OTOSPoseUtil poseUtil = new OTOSPoseUtil();

    public OTOSLocalizer(HardwareMap hwm) {
        super();
        otos = hwm.get(SparkFunOTOS.class, "otos");
        otos.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        otos.setAngularUnit(SparkFunOTOS.AngularUnit.RADIANS);
        otos.setOffset(new SparkFunOTOS.Pose2D(4,1.5,Math.toRadians(90.5)));
        otos.setAngularScalar(360/364.0);
        otos.calibrateImu();
        otos.resetTracking();
    }

    @Override
    /**
     * Method to fetch the current robot pose from the localizer. Called in the <code>update()</code> cycle.
     * @return The current position, as an FTCLib Pose2d.
     */
    protected Pose2d internalGetPose() {
        return poseUtil.toPose2d(otos.getPosition());
    }

    @Override
    /**
     * Method to fetch the current robot velocity from the localizer. Called in the <code>update()</code> cycle.
     * @return The current velocity, as an FTCLib Pose2d.
     */
    protected Pose2d internalGetVelocity(Pose2d lastPose, Pose2d currentPose, double deltaSeconds) {
        return getVelocityFromPose(lastPose, currentPose, deltaSeconds);
    }

    @Override
    public void reset(double x, double y, double heading) {
        otos.resetTracking();
        super.reset();
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
