package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

@TeleOp(name = "MecanumTeleOp", group = "TeleOp")

public class MecanumTeleOp extends OpMode {

	MecanumDrive drive;

	@Override
	public void init( ) {
		drive = new MecanumDrive( hardwareMap );
		drive.setMotorDirections(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE , DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD);
//		drive.setRunMode( DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.RunMode.RUN_USING_ENCODER );
	}

	@Override
	public void loop( ) {
		drive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );
		displayTelemetry();
	}

	public void displayTelemetry( ) {
		telemetry.addLine( "ly: " + -gamepad1.left_stick_y );
		telemetry.addLine( "lx: " + gamepad1.left_stick_x );
		telemetry.addLine( "rx: " + gamepad1.right_stick_x );
		telemetry.addLine(  );
		telemetry.addLine( "fl: " + drive.frontLeft.getCurrentPosition( ) );
		telemetry.addLine( "bl: " + drive.backLeft.getCurrentPosition( ) );
		telemetry.addLine( "fr: " + drive.frontRight.getCurrentPosition( ) );
		telemetry.addLine( "br: " + drive.backRight.getCurrentPosition( ) );
		telemetry.update( );
	}
}