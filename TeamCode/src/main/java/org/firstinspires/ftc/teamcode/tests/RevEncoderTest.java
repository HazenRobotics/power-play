package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class RevEncoderTest extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {
		DcMotorEx motor = hardwareMap.get( DcMotorEx.class, "motor" );
		waitForStart();

		while (opModeIsActive()) {
			telemetry.addData( "position", motor.getCurrentPosition() );
			telemetry.update();
		}
	}
}
