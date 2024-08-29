package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous
public class FastBNOTest extends LinearOpMode {

    private ElapsedTime elapsed = new ElapsedTime();
    private FastBNO055 imu;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(FastBNO055.class, "imu");

        imu.resetYaw();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double time0 = elapsed.time(TimeUnit.MILLISECONDS);
            double heading = imu.getHeading();
            double time1 = elapsed.time(TimeUnit.MILLISECONDS);
            telemetry.addData("Heading", heading);
            telemetry.addData("Time taken for imu read", time1-time0);
            telemetry.update();
        }


    }


}
