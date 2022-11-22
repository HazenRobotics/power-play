package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "SwerveTeleOp", group = "Test")
@Disabled
public class SwerveTeleOp extends OpMode {

	DcMotor leftMotor, rightMotor;

	@Override
	public void init( ) {

		telemetry.addData( "Mode", "Initiating robot..." );
		telemetry.update( );

		leftMotor = hardwareMap.dcMotor.get( "left" );
		rightMotor = hardwareMap.dcMotor.get( "right" );

		telemetry.addData( "Mode", "waiting for start??" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		leftMotor.setPower( gamepad1.left_stick_y );
		rightMotor.setPower( gamepad1.right_stick_y );

		telemetry.addData( "l: ", gamepad1.left_stick_y );
		telemetry.addData( "r: ", gamepad1.right_stick_y );

		telemetry.update( );
	}

}