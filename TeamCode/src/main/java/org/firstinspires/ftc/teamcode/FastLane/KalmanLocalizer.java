package org.firstinspires.ftc.teamcode.FastLane;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.stats.filtering.MultivariateKalman;

public abstract class KalmanLocalizer extends Localizer {

    private MultivariateKalman kalman;

    private SimpleMatrix R;
    private SimpleMatrix F;

    private boolean useVelocity;

    public void initKalman(MultivariateKalman kalman, SimpleMatrix R, boolean useVelocity) {
        this.kalman = kalman;
        this.R = R;
        this.useVelocity = useVelocity;
    }

    protected Pose2d getPosEstimate() {
        SimpleMatrix state = kalman.getMeans();
        return new Pose2d(state.get(0, 0), state.get(2, 0), new Rotation2d(state.get(4, 0)));
    }

    protected Pose2d getVelEstimate() {
        SimpleMatrix state = kalman.getMeans();
        return new Pose2d(state.get(1, 0), state.get(3, 0), new Rotation2d(state.get(5, 0)));
    }

    protected abstract Pose2d internalGetVelocity();

    @Override
    public void update() {
        computeF();
        SimpleMatrix z;
        if (!useVelocity) {
            Pose2d positionMeasurement = internalGetPose();
            z = new SimpleMatrix(new double[][]{{positionMeasurement.getX(), positionMeasurement.getY(), positionMeasurement.getRotation().getRadians()}});
        } else {
            Pose2d positionMeasurement = internalGetPose();
            Pose2d velocityMeasurement = internalGetVelocity();
            z = new SimpleMatrix(new double[][]{{positionMeasurement.getX(), velocityMeasurement.getX(), positionMeasurement.getY(), velocityMeasurement.getY()
                    ,positionMeasurement.getRotation().getRadians(), velocityMeasurement.getRotation().getRadians()}});
        }
        kalman.bulkUpdate(F, z, R);
        assignEstimates();
    }

    public void update(Pose2d drivetrainInput) {
        computeF();
        SimpleMatrix z;
        if (!useVelocity) {
            Pose2d positionMeasurement = internalGetPose();
            z = new SimpleMatrix(new double[][]{{positionMeasurement.getX(), positionMeasurement.getY(), positionMeasurement.getRotation().getRadians()}});
        } else {
            Pose2d positionMeasurement = internalGetPose();
            Pose2d velocityMeasurement = internalGetVelocity();
            z = new SimpleMatrix(new double[][]{{positionMeasurement.getX(), velocityMeasurement.getX(), positionMeasurement.getY(), velocityMeasurement.getY()
                    ,positionMeasurement.getRotation().getRadians(), velocityMeasurement.getRotation().getRadians()}});
        }
        SimpleMatrix u = new SimpleMatrix(new double[][]{{drivetrainInput.getX(), drivetrainInput.getY(), drivetrainInput.getRotation().getRadians()}});
        kalman.bulkUpdate(F, u, z, R);
        assignEstimates();
    }

    protected void computeF() {
        if (!firstUpdate) {
            deltaSeconds = time.time();
        } else {
            firstUpdate = false;
        }

        F = new SimpleMatrix(
                new double[][]{{1, deltaSeconds, 0, 0, 0, 0},
                        {0, 1, 0, 0, 0, 0},
                        {0, 0, 1, deltaSeconds, 0, 0},
                        {0, 0, 0, 1, 0, 0},
                        {0, 0, 0, 0, 1, deltaSeconds},
                        {0, 0, 0, 0, 0, 1}}
        );

        time.reset();
    }

    protected void assignEstimates() {
        Pose2d pos = getPosEstimate();
        velocity = getVelEstimate();
        lastPose = pos;
    }


}
