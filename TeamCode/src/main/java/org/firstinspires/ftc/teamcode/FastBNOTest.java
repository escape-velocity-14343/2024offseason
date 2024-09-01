package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous
public class FastBNOTest extends LinearOpMode {

    public static boolean useBaseBno = false;

    private ElapsedTime elapsed = new ElapsedTime();
    private IMU imu;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        imu = hardwareMap.get(IMU.class, "imu");

        imu.resetYaw();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double time0 = elapsed.time(TimeUnit.MILLISECONDS);
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double time1 = elapsed.time(TimeUnit.MILLISECONDS);
            telemetry.addData("Heading", heading);
            telemetry.addData("Time taken for imu read", time1-time0);
            telemetry.update();
        }


    }


}
