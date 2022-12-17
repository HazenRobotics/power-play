package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.AprilTagsUtil;
import org.firstinspires.ftc.teamcode.vision.SignalUtil;

@Autonomous(group="vision")
public class SignalDetectionAprilTest extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {
		AprilTagsUtil detector = new AprilTagsUtil( hardwareMap, "webcam1", telemetry );
		detector.init();

		telemetry.addLine("camera is ready");
		telemetry.update();

		waitForStart();
	}

}
