package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class SwerveController {
    public static double blOffset = -50, brOffset = 35, flOffset = 14, frOffset = -30;
    SwerveModule bl,br,fl,fr;
    public SwerveController(HardwareMap h) {
        bl = new SwerveModule(h.dcMotor.get("blMotor"), h.crservo.get("blServo"), h.analogInput.get("blAnalog"));
        bl.setOffset(blOffset);
        br = new SwerveModule(h.dcMotor.get("brMotor"), h.crservo.get("brServo"), h.analogInput.get("brAnalog"));
        br.setOffset(brOffset);
        fl = new SwerveModule(h.dcMotor.get("flMotor"), h.crservo.get("flServo"), h.analogInput.get("flAnalog"));
        fl.setOffset(flOffset);
        fr = new SwerveModule(h.dcMotor.get("frMotor"), h.crservo.get("frServo"), h.analogInput.get("frAnalog"));
        fr.setOffset(frOffset);
    }
    public void crabTest(double angle) {
        bl.PID(0,angle);
        br.PID(0,angle);
        fl.PID(0,angle);
        fr.PID(0,angle);
    }
    public void drive(double x, double y, double turn) {
        bl.setOffset(blOffset);
        br.setOffset(brOffset);
        fl.setOffset(flOffset);
        fr.setOffset(frOffset);
        //288 for short side
        //352 for long side
        double ratio = 352.0/288.0;


        double a = x-turn*ratio;
        double b = x+turn*ratio;
        double c = y-turn;
        double d = y+turn;
        fr.driveXY(b,c);
        fl.driveXY(b,d);
        bl.driveXY(a,d);
        br.driveXY(a,c);

        getTelemetry();
    }
    public void getTelemetry() {
        FtcDashboard.getInstance().getTelemetry().addData("bl", bl.getAngle());
        FtcDashboard.getInstance().getTelemetry().addData("fl", fl.getAngle());
        FtcDashboard.getInstance().getTelemetry().addData("br", br.getAngle());
        FtcDashboard.getInstance().getTelemetry().addData("fr", fr.getAngle());
        FtcDashboard.getInstance().getTelemetry().update();

    }
}
