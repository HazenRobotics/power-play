package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp(group = "test")
//@Disabled
public class DoubleLiftTest extends OpMode {

	DcMotor left;
	DcMotor right;

	@Override
	public void init( ) {
		left = hardwareMap.dcMotor.get( "left" );
		right = hardwareMap.dcMotor.get( "right" );
		right.setDirection( DcMotorSimple.Direction.REVERSE );
	}

	@Override
	public void loop( ) {
		left.setPower( gamepad1.right_trigger - gamepad1.left_trigger );
		right.setPower( gamepad1.right_trigger - gamepad1.left_trigger );
	}
}
