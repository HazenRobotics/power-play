package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.MultiaxialArm;

@TeleOp(name = "MultiaxialArmTest", group = "Tests" )
public class MultiaxialArmTest extends OpMode {

	MultiaxialArm arm;

	@Override
	public void init( ) {
		arm = new MultiaxialArm( hardwareMap );
	}

	@Override
	public void loop( ) {

		double armRotatePower = gamepad1.left_stick_x;
		double lowerPower = gamepad1.left_stick_y;
		double upperPower = gamepad1.right_stick_y;

		arm.controlMotors( armRotatePower, lowerPower, upperPower );
		arm.printTelemetry( telemetry );
	}
}
