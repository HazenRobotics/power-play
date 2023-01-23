package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

    @Override
    public void runOpMode() {

        robot = new MiniBot(this);

        robot.initSubsystems();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Element position", robot.signalUtil.getSignalPosition());
            telemetry.update();
        }

        waitForStart();

        AprilTagDetectionPipeline.SignalPosition signalPosition = robot.signalUtil.getSignalPosition();

        robot.claw.close();

        waitRobot( 250 );

        robot.drive.setWeightedDrivePower(new Pose2d( 0,0,0 ));

        if (signalPosition == AprilTagDetectionPipeline.SignalPosition.LEFT) {
            robot.drive.setWeightedDrivePower( new Pose2d( 0, 0.5, -0.025  ) );
            waitRobot( 1500 );
        } else if (signalPosition == AprilTagDetectionPipeline.SignalPosition.RIGHT) {
            robot.drive.setWeightedDrivePower( new Pose2d( 0, -0.5, 0.025  ) );
            waitRobot( 1500 );
        }

        robot.drive.setWeightedDrivePower( new Pose2d( 0,0,0 ) );

        robot.drive.setWeightedDrivePower(new Pose2d( 0.25,0,0 ) );

        waitRobot(2700);

        robot.drive.setWeightedDrivePower( new Pose2d( 0,0,0 ) );
    }

    public static void waitRobot(int mills) {
        long startTime = System.currentTimeMillis();
        while ((startTime + mills) > System.currentTimeMillis()) ;
    }

}


