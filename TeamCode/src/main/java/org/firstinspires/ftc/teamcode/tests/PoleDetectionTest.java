package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.PoleUtil;
import org.firstinspires.ftc.teamcode.vision.SignalUtil;
import org.firstinspires.ftc.teamcode.vision.pipelines.PoleDetector;

@Autonomous(group="vision")
public class PoleDetectionTest extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {
		PoleUtil detector = new PoleUtil( hardwareMap, "webcam1", telemetry );
		detector.init();

		telemetry.addLine("camera is ready");
		telemetry.update();

		waitForStart();

		while(!isStopRequested());
	}

}
