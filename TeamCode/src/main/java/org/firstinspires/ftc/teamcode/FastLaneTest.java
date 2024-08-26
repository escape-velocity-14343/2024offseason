package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.FastLane.FastLaneController;
import org.firstinspires.ftc.teamcode.FastLane.FieldObstacle;
import org.firstinspires.ftc.teamcode.FastLane.InvertibleWaypoint;
import org.firstinspires.ftc.teamcode.FastLane.OTOSLocalizer;

@Config
@Autonomous(name="Fast Lane Tester")
public class FastLaneTest extends LinearOpMode {

    public static double scalar = 1;
    public static double maxVelocity = 80;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor fr = hardwareMap.dcMotor.get("frontRight");
        DcMotor fl = hardwareMap.dcMotor.get("frontLeft");
        DcMotor br =  hardwareMap.dcMotor.get("backRight");
        DcMotor bl = hardwareMap.dcMotor.get("backLeft");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(
                fr,
                fl,
                br,
                bl,
                telemetry
        );

        OTOSLocalizer otos = new OTOSLocalizer(hardwareMap);

        FastLaneController controller = new FastLaneController(12, 2, 20,
                maxVelocity, otos,  new PIDFController(0.01 * 180/Math.PI,0,0, 0));

        controller.setObstacles();
        controller.setWaypoints(new InvertibleWaypoint(0, 0, 0).toAutonomousWaypoint(),
                new InvertibleWaypoint(24, 0, 0).toAutonomousWaypoint(),
            new InvertibleWaypoint(24, -30, Math.PI/2).toAutonomousWaypoint());
        InvertibleWaypoint.configAuto(true, true);

        telemetry.addData("Pose x", otos.getPose().getX());
        telemetry.addData("Pose y", otos.getPose().getY());
        telemetry.addData("Pose h", otos.getPose().getRotation().getRadians());
        telemetry.update();

        waitForStart();

        int counter = 0;

        while (!isStopRequested()) {
            controller.update();
            otos.update();
            double[] powers = controller.getMovementVector();

            if (controller.isDone() && counter == 0) {
                counter++;
                controller.setWaypoints(new InvertibleWaypoint(24, -30, Math.PI/2).toAutonomousWaypoint(),
                        new InvertibleWaypoint(50, -8, Math.PI/4).toAutonomousWaypoint().setTolerance(2),
                        new InvertibleWaypoint(50, 62, Math.PI/2).toAutonomousWaypoint());
            } else if (controller.isDone() && counter >= 1) {
                requestOpModeStop();
                break;
            }

            drive.drive(powers[0]*scalar, powers[1]*scalar, -powers[2], otos.getPose().getRotation().getDegrees());
            telemetry.addData("Pose x", otos.getPose().getX());
            telemetry.addData("Pose y", otos.getPose().getY());
            telemetry.addData("Pose h", otos.getPose().getRotation().getRadians());
            telemetry.update();
            //drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, otos.getPose().getRotation().getDegrees());
        }
    }
}
