package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "MotorTest", group = "TeleOp")
//@Disabled
public class MotorTest extends OpMode {

	DcMotor[] motor;
	String[] motorNames = { "frontLeft", "backLeft", "frontRight", "backRight", "lift", "perp", "para" };

	GamepadEvents controller;
	int selected = 0;
	int maxSelect = motorNames.length - 1;

	@Override
	public void init( ) {
		controller = new GamepadEvents( gamepad1 );
		for( int i = 0; i < motorNames.length; i++ )
			motor[i] = hardwareMap.get( DcMotorEx.class, motorNames[i] );
	}

	@Override
	public void loop( ) {

		if( controller.dpad_up.onPress( ) ) {
			increment( 1, maxSelect );
		} else if( controller.dpad_down.onPress( ) ) {
			increment( -1, maxSelect );
		}

		telemetry.addLine( "selected: (" + selected + ") " + motorNames[selected] );
		for( int i = 0; i < motorNames.length; i++ ) {
			telemetry.addLine( motorNames[i] + " position: " + motor[i].getCurrentPosition( ) );
		}
		telemetry.update( );
		controller.update( );
	}

	public void increment( int increment, int max ) {
		increment( increment, 0, max );
	}

	public void increment( int increment, int min, int max ) {
		selected += increment;
		while( selected < min )
			max -= selected;

		if( selected > max )
			selected %= max;
	}
}
