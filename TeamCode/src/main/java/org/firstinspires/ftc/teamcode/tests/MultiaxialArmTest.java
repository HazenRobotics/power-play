package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MultiaxialArmTest", group = "Tests" )
public class MultiaxialArmTest extends OpMode {

	CRServo spinnyServo;
	Servo lowerServo;
	Servo upperServo;

	double spinnyServoPower = 0;
	double lowerServoPosition;
	double upperServoPosition;

	@Override
	public void init( ) {
		spinnyServo = hardwareMap.crservo.get( "spinnyServo" );
		lowerServo = hardwareMap.servo.get( "lowerServo" );
		upperServo = hardwareMap.servo.get( "upperServo" );

		lowerServoPosition = lowerServo.getPosition();
		upperServoPosition = upperServo.getPosition();
	}

	@Override
	public void loop( ) {
		if(gamepad1.a)
			spinnyServoPower = 0.25;
		else if ( gamepad1.b )
			spinnyServoPower = -0.25;
		else
			spinnyServoPower = 0;


		if(gamepad1.dpad_up)
			lowerServoPosition += 0.005;
		else if ( gamepad1.dpad_down )
			lowerServoPosition -= 0.005;

		if(lowerServoPosition > 1)
			lowerServoPosition = 1;
		else if(lowerServoPosition < 0)
			lowerServoPosition = 0;

		if(gamepad1.x)
			upperServoPosition += 0.005;
		else if ( gamepad1.y )
			upperServoPosition -= 0.005;

		if(upperServoPosition > 1)
			upperServoPosition = 1;
		else if(upperServoPosition < 0)
			upperServoPosition = 0;

		spinnyServo.setPower(spinnyServoPower);
		lowerServo.setPosition(lowerServoPosition);
		upperServo.setPosition(upperServoPosition);
		printTelemetry();
	}

	public void printTelemetry() {
		telemetry.addLine("Spinny Servo Power: " + spinnyServoPower);
		telemetry.addLine("Lower Servo Position: " + lowerServoPosition);
		telemetry.addLine("Upper Servo Position: " + upperServoPosition);
		telemetry.update();
	}
}
