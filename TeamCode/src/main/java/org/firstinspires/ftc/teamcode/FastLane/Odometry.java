package org.firstinspires.ftc.teamcode.FastLane;

import com.arcrobotics.ftclib.geometry.Pose2d;

public interface Odometry {

    public void update();

    public void reset();

    public void reset(double x, double y, double heading);
    
    public void reset(Pose2d pose);

    public Pose2d getPose();
    
    public Pose2d getVelocity();

    public Pose2d getProjectedPose();

}
