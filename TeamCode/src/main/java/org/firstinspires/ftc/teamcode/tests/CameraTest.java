package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.SignalDetector;
import org.firstinspires.ftc.teamcode.vision.SignalUtil;

@TeleOp(name = "CameraTest", group = "Autonomous")
public class CameraTest extends LinearOpMode {

	SignalUtil detector;

	@Override
	public void runOpMode( ) throws InterruptedException {
		detector = new SignalUtil( hardwareMap, "webcam1", telemetry );
		detector.init();

		telemetry.addLine("camera is ready");
		telemetry.update();

		waitForStart();

		telemetry.addLine( "Position: " + detector.getBarcodePosition( ) );

		while(true);
	}
}
