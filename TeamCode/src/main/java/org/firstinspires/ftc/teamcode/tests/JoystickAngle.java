package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "Test")
public class JoystickAngle extends OpMode {

	@Override
	public void init( ) {

	}

	@Override
	public void loop( ) {
		Vector2d input = new Vector2d( gamepad1.left_stick_x, -gamepad1.left_stick_y );

		telemetry.addData( "angle", -(Math.toDegrees( input.angle() ) - 90));
		telemetry.addData( "magnitude", input.distTo( new Vector2d( 0, 0 ) ) );
		telemetry.addData( "x", input.getX() );
		telemetry.addData( "y", input.getY() );
		telemetry.update();
	}
}
