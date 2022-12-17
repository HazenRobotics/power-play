package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.drives.roadrunner.MecanumDriveMini;
import org.firstinspires.ftc.teamcode.robots.MiniBot;

public class DetectIfOverJunction extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {
		MiniBot robot = new MiniBot( this );

		robot.drive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		waitForStart();

		while (!isStopRequested()) {
			robot.drive.setWeightedDrivePower(
					new Pose2d(
							-gamepad1.left_stick_y,
							-gamepad1.left_stick_x,
							-gamepad1.right_stick_x
					)
			);

			robot.drive.update();
			Pose2d poseEstimate = robot.drive.getPoseEstimate();

//			Vector3D clawPos = robot.getClawPos();

			telemetry.addData("heading", poseEstimate.getHeading());
			telemetry.update();
		}
	}
}
