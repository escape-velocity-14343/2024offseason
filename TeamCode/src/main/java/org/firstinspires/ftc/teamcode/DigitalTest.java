package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
@TeleOp
public class DigitalTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DigitalChannel googoo = hardwareMap.digitalChannel.get("googoo");
        DigitalChannel gaga = hardwareMap.digitalChannel.get("gaga");
        while(!isStopRequested()) {
            telemetry.addData("googoo", googoo.getState());
            telemetry.addData("gaga",gaga.getState());
            telemetry.update();
        }
    }
}
