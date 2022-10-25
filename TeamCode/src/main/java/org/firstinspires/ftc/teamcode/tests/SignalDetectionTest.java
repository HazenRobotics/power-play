package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.SignalUtil;

@TeleOp(name = "SignalDetectionTest", group = "Autonomous")
@Disabled
public class SignalDetectionTest extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {
		SignalUtil detector = new SignalUtil( hardwareMap, "webcam1", telemetry );
		detector.init();

		telemetry.addLine("camera is ready");
		telemetry.update();

		waitForStart();

		telemetry.addLine( "Position: " + detector.getSignalPosition( ) );

		while(!isStopRequested());
	}

}
