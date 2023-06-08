package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class EncoderMotorTest extends LinearOpMode {

	DcMotorEx motor;

	@Override
	public void runOpMode( ) throws InterruptedException {
		motor = hardwareMap.get( DcMotorEx.class, "motor" );
		waitForStart();
		while (opModeIsActive() && !isStopRequested()) {
			motor.setPower( gamepad1.right_trigger - gamepad1.left_trigger );
			telemetry.addData( "position", motor.getCurrentPosition( ) );
			telemetry.update( );
		}
	}
}
