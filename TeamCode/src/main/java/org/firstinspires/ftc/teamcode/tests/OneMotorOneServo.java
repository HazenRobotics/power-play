package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class OneMotorOneServo extends LinearOpMode {

	DcMotorEx motor;
	CRServo servo;

	@Override
	public void runOpMode( ) throws InterruptedException {
		motor = hardwareMap.get( DcMotorEx.class, "motor" );
		servo = hardwareMap.crservo.get( "servo" );

		waitForStart();

		while(opModeIsActive()) {
			motor.setPower( gamepad1.right_trigger - gamepad1.left_trigger );

			if( gamepad1.a ) {
				servo.setPower( 1 );
			} else if( gamepad1.b ) {
				servo.setPower( -1 );
			} else {
				servo.setPower( 0 );
			}
		}
	}
}
