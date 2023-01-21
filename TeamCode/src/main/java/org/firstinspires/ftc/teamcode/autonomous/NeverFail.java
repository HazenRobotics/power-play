package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.vision.AprilTagsUtil;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;

@Autonomous(name = "NeverFail", group = "Autonomous")
//@Disabled
public class NeverFail extends LinearOpMode {

    MiniBot robot;
    MecanumDrive drive;

    @Override
    public void runOpMode() {

        robot = new MiniBot(this);
        drive = new MecanumDrive(hardwareMap);
        AprilTagsUtil detector = new AprilTagsUtil(hardwareMap, "webcam1", telemetry);
        detector.init();

        DcMotorEx[] motors = {
                drive.frontLeft,
                drive.backLeft,
                drive.frontRight,
                drive.backRight,
        };

        AprilTagDetectionPipeline.SignalPosition park;
        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Element position", robot.signalUtil.getSignalPosition());
            telemetry.update();
        }
        waitForStart();
        park = robot.signalUtil.getSignalPosition();
        for (DcMotorEx d : motors) {
            d.setPower(0.25);
        }
        waitRobot(2000);

        while (motors[0].isBusy()) ;

        if (park != AprilTagDetectionPipeline.SignalPosition.MIDDLE) {
            if (park == AprilTagDetectionPipeline.SignalPosition.LEFT) {
                motors[0].setPower(-0.25);
                motors[1].setPower(0.25);
                motors[2].setPower(-0.25);
                motors[3].setPower(0.25);
            }
            if (park == AprilTagDetectionPipeline.SignalPosition.RIGHT) {
                motors[0].setPower(0.25);
                motors[1].setPower(-0.25);
                motors[2].setPower(0.25);
                motors[3].setPower(-0.25);
            }
            waitRobot(2000);
        }
    }

    public static void waitRobot(int mills) {
        long startTime = System.currentTimeMillis();
        while ((startTime + mills) > System.currentTimeMillis()) ;
    }

}


