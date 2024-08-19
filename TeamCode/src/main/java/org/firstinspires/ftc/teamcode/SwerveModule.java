package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class SwerveModule {
    DcMotor m;
    CRServo s;
    AnalogInput a;
    double offset=0;
    public static double kP = 0.01;
    PIDController pid = new PIDController(0,0,0);
    public SwerveModule(DcMotor motor, CRServo servo, AnalogInput analog) {
        m = motor;
        s = servo;
        a = analog;
    }

    public void move(double drive, double turn) {
        m.setPower(drive);
        s.setPower(turn);
    }
    public void PID(double drive, double angle) {
        pid.setP(kP);
        double currentRot = AngleUnit.normalizeDegrees((a.getVoltage()*360/3.3)-180+offset);

        if(Math.abs(AngleUnit.normalizeDegrees(angle-currentRot))>90) {
            angle = AngleUnit.normalizeDegrees(angle+180);
            drive *= -1;
        }
        if (Math.abs(drive)<0.01) {
            move(0, 0);
        }
        else
            move(drive, kP*AngleUnit.normalizeDegrees(angle-currentRot));
    }
    public void setOffset(double offset) {
        this.offset = offset;
    }
    public void driveXY(double x, double y) {
        PID(Math.hypot(x,y),Math.toDegrees(Math.atan2(y,x)));
    }
    public double getAngle() {
        return AngleUnit.normalizeDegrees((a.getVoltage()*360/3.3)-180+offset);
    }
}
