package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.SignalUtil;

@Autonomous(group="vision")
public class SignalDetectionTest extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {
		SignalUtil detector = new SignalUtil( hardwareMap, "webcam1", telemetry );
		detector.init();

		telemetry.addLine("camera is ready");
		telemetry.update();

		telemetry.addLine( "Position: " + detector.getSignalPosition( ) );

		waitForStart();

		while(!isStopRequested());
	}

}
