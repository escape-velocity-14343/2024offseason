package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class AnalogTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput analog = hardwareMap.get(AnalogInput.class, "analog");
        DcMotor motor = hardwareMap.dcMotor.get("frontLeft");
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        while(!isStopRequested()) {
            telemetry.addData("voltage",analog.getVoltage());
            motor.setPower(0.2);
            telemetry.update();
        }
    }
}
