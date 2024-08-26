package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDriveSubsystem extends SubsystemBase {
    DcMotor fr, fl, br, bl;
    Telemetry t = FtcDashboard.getInstance().getTelemetry();
    public MecanumDriveSubsystem(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl) {
        this.fr = fr;
        this.fl = fl;
        this.br = br;
        this.bl = bl;
    }
    public MecanumDriveSubsystem(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl, Telemetry telemetry) {
        this.fr = fr;
        this.fl = fl;
        this.br = br;
        this.bl = bl;
        t=telemetry;
    }
    /**
     * @param x positive drives forward
     * @param y positive drives left
     * @param rx idk
     * @param heading in degrees
     * */
    public void drive(double x, double y, double rx, double heading) {
        t.addData("inputx", x);
        t.addData("inputy",y);
        t.addData("inputr", rx);
        double headingDegs = -Math.toRadians(heading);
        double rotX = y * Math.cos(headingDegs) + x * Math.sin(headingDegs);
        t.addData("rotX", rotX);
        double rotY = y * Math.sin(headingDegs) - x * Math.cos(headingDegs);
        t.addData("rotY", rotY);
        //rotY = -rotY;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);

    }
}
