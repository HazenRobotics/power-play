package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.subsystems.ServoEncoderTurret;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(group = "Test")
public class EncoderTurretTest extends OpMode {

	GamepadEvents gamepad;
	ServoEncoderTurret turret;
	boolean async = false;

	@Override
	public void init( ) {

		Robot.writeToDefaultFile( "created file", false, true );

		gamepad = new GamepadEvents( gamepad1 );
		turret = new ServoEncoderTurret( hardwareMap, "turret", "turr", false, true, AngleUnit.RADIANS, 2000, 15.0 / 170 );
		turret.setLimit( -180, 0 );
	}

	@Override
	public void loop( ) {

		if( gamepad.dpad_up.onPress( ) ) {
			turret.setRotation( 0.4, 0 );
			async = true;
		} else if( gamepad.dpad_left.onPress( ) ) {
			turret.setRotation( 0.2, -Math.PI / 2 );
			async = true;
		} else if( gamepad.dpad_down.onPress( ) ) {
			turret.setRotation( 0.3, -Math.PI );
			async = true;
		} else if( gamepad.dpad_right.onPress( ) ) {
			turret.setRotation( 0.4, -3 * Math.PI / 2 );
			async = true;
		}

		if( async && Math.abs(gamepad.right_stick_x) > 0.01 )
			async = false;
		if( !async )
			turret.setLivePower( gamepad.right_stick_x / 2 );

		telemetry.addLine( "position: " + turret.getPosition( ) );
		telemetry.addLine( "heading: " + turret.getHeading( ) );
		telemetry.update( );
		gamepad.update( );
	}

	/*
	0, -6
	0.9, -5
	1.9, -3.6
	3.1, -2.2
	4.7,




	 */
}
