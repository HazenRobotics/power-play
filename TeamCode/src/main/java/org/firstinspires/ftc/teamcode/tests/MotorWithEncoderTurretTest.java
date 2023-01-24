package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(group = "Test")
public class MotorWithEncoderTurretTest extends OpMode {

	Turret turret;
	double desiredPos;
	double desiredPow;

	@Override
	public void init( ) {
		turret = new Turret( hardwareMap, "turr", true, AngleUnit.DEGREES, 384.5, 170.0 / 30.0, -255, 75, new PIDController( 0,0,0 ) );
	}

	@Override
	public void loop( ) {
		Vector2d joystick = new Vector2d( gamepad1.right_stick_x, -gamepad1.right_stick_y );

		turret.setLiveRotationPower( joystick, 0 );

		double moveX = joystick.getX( );
		double moveY = joystick.getY( );

		telemetry.addData( "desired pos (heading)", -(Math.toDegrees( joystick.angle( ) ) - 90) );
		telemetry.addData( "desired power", Math.sqrt( moveX * moveX + moveY * moveY - (moveX * moveX * moveY * moveY) ) * .25 );
		telemetry.addData( "actualHeading", turret.getTurretHeading( ) );
		telemetry.addData( "actualPower", turret.getPower( ) );
		telemetry.update( );
	}
}
