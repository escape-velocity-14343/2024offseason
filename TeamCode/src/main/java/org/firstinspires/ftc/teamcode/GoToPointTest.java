package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.linefollower.LineFollowerSubsystem;
import org.firstinspires.ftc.teamcode.linefollower.LineRelocalizeCommand;

@Autonomous
public class GoToPointTest extends CommandOpMode {
    DcMotor fl, fr, bl, br;
    @Override
    public void initialize() {
        OTOSSubsystem otos = new OTOSSubsystem(hardwareMap,"otos",telemetry);
        LineFollowerSubsystem lineFollower = new LineFollowerSubsystem(hardwareMap,"linefollower");
        otos.reset();
        fl = hardwareMap.dcMotor.get("frontLeft");
        fr = hardwareMap.dcMotor.get("frontRight");
        bl = hardwareMap.dcMotor.get("backLeft");
        br = hardwareMap.dcMotor.get("backRight");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        MecanumDriveSubsystem mecanum = new MecanumDriveSubsystem(fr, fl, br, bl,telemetry);
        register(otos, mecanum,lineFollower);
        waitForStart();
        schedule(new SequentialCommandGroup(
                new GoToPointCommand(mecanum, otos, new Pose2d(24,0,new Rotation2d(0)),0.5),
                new GoToPointCommand(mecanum, otos, new Pose2d(24,-36,new Rotation2d(Math.PI/2)),0.5),
                new GoToPointCommand(mecanum, otos, new Pose2d(50,-8,new Rotation2d(Math.PI/4)),2,10).setEndWhenPast(true),
                new GoToPointCommand(mecanum, otos, new Pose2d(50,62,new Rotation2d(Math.PI/2)),1).setEndWhenPast(true),
                new GoToPointCommand(mecanum, otos, new Pose2d(50,72,new Rotation2d(Math.PI/2)),0.5),
                new ParallelDeadlineGroup(new LineRelocalizeCommand(lineFollower,otos),new GoToPointCommand(mecanum, otos, new Pose2d(50,72,new Rotation2d(Math.PI/2)),0.0)),
                new GoToPointCommand(mecanum, otos, new Pose2d(50,-8,new Rotation2d(Math.PI/2)),2).setEndWhenPast(true),
                new GoToPointCommand(mecanum, otos, new Pose2d(24,-36,new Rotation2d(Math.PI/2)),0.5),
                new GoToPointCommand(mecanum, otos, new Pose2d(0,0,new Rotation2d(0)))
        ));

    }
}
