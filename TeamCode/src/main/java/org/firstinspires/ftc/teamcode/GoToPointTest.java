package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class GoToPointTest extends CommandOpMode {
    DcMotor fl, fr, bl, br;
    @Override
    public void initialize() {
        OTOSSubsystem otos = new OTOSSubsystem(hardwareMap,"otos",telemetry);
        otos.reset();
        fl = hardwareMap.dcMotor.get("frontLeft");
        fr = hardwareMap.dcMotor.get("frontRight");
        bl = hardwareMap.dcMotor.get("backLeft");
        br = hardwareMap.dcMotor.get("backRight");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        MecanumDriveSubsystem mecanum = new MecanumDriveSubsystem(fr, fl, br, bl,telemetry);
        register(otos, mecanum);
        waitForStart();
        schedule(new GoToPointCommand(mecanum, otos, new Pose2d(10,0,new Rotation2d(0)),0.1));
        schedule(new GoToPointCommand(mecanum, otos, new Pose2d(20,20,new Rotation2d(0)),0.1));
    }
}