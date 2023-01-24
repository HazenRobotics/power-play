package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "➰ Hertz Test", group = "Test")
public class HertzTest extends OpMode {

	double loopTime = 0;
	boolean testing = false;
	double normHertz = 0, testingHertz = 0;
	GamepadEvents controller;
	boolean prevPress = false;

	@Override
	public void init( ) {

		controller = new GamepadEvents( gamepad1 );

		telemetry.addLine( "ready" );
	}

	@Override
	public void loop( ) {
		double loop = System.nanoTime( );
		if( testing ) {
			testingHertz = 1000000000 / (loop - loopTime);

			// test between here
			if( controller.left_bumper.onPress( ) ) testing = true;
			if( controller.right_bumper.onPress( ) ) testing = true;
			if( controller.a.onPress( ) ) testing = true;
			if( controller.b.onPress( ) ) testing = true;
			if( controller.x.onPress( ) ) testing = true;
			if( controller.y.onPress( ) ) testing = true;
			if( controller.dpad_up.onPress( ) ) testing = true;
			if( controller.dpad_down.onPress( ) ) testing = true;
			if( controller.dpad_left.onPress( ) ) testing = true;
			if( controller.dpad_right.onPress( ) ) testing = true;
			if( controller.right_trigger.onPress( ) ) testing = true;
			if( controller.left_trigger.onPress( ) ) testing = true;
			if( controller.left_stick_x > 0 ) testing = true;
			if( controller.left_stick_y > 0 ) testing = true;
			if( controller.right_stick_x > 0 ) testing = true;
			if( controller.right_stick_y > 0 ) testing = true;
			if( controller.left_stick_button.onPress( ) ) testing = true;
			if( controller.right_stick_button.onPress( ) ) testing = true;
			if( controller.back.onPress( ) ) testing = true;
			if( controller.start.onPress( ) ) testing = true;
			if( controller.ps.onPress( ) ) testing = true;
			if( controller.guide.onPress( ) ) testing = true;
			if( controller.touchpad.onPress( ) ) testing = true;

			controller.update( );

			// and here
		} else
			normHertz = 1000000000 / (loop - loopTime);


		if( gamepad1.a && !prevPress )
			testing = !testing;
		prevPress = gamepad1.a;//


		telemetry.addLine( "press a to toggle testing: testing = " + testing );
		telemetry.addData( "norm hz ", normHertz );
		telemetry.addData( "testing hz ", testingHertz );
		loopTime = loop;
		telemetry.update( );
	}
}
