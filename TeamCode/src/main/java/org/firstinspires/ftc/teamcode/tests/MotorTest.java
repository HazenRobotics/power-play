package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

import java.util.ArrayList;

@TeleOp(name = "MotorTest", group = "TeleOp")
//@Disabled
public class MotorTest extends OpMode {


	ArrayList<String> motorNames = new ArrayList<>( );
	ArrayList<DcMotorEx> motor = new ArrayList<>( );

	GamepadEvents controller;
	int selected = 0;
	int maxSelect;

	@Override
	public void init( ) {
		controller = new GamepadEvents( gamepad1 );
		String[] names = { "topLeft", "bottomLeft", "topRight", "bottomRight", "frontLeft", "backLeft", "frontRight", "backRight", "lift", "para", "perp" };

		for( int i = 0; i < names.length; i++ ) {
			try {
				motor.add( hardwareMap.get( DcMotorEx.class, names[i] ) );
				this.motorNames.add( names[i] );

			} catch( IllegalArgumentException e ) {
				telemetry.addLine( "couldn't find " + names[i] );
			}
		}

		maxSelect = motor.size( ) - 1;
		telemetry.addLine( "added " + (maxSelect) + "motor" + (maxSelect == 0 ? "" : "s") );
		telemetry.update( );

	}

	@Override
	public void loop( ) {

		motor.get( selected ).setPower( -controller.right_stick_y );

		if( controller.dpad_up.onPress( ) ) {
			increment( 1, maxSelect );
		} else if( controller.dpad_down.onPress( ) ) {
			increment( -1, maxSelect );
		}

		telemetry.addLine( "selected: (" + selected + ") " + motorNames.get( selected ) );
		for( int i = 0; i < motor.size( ); i++ ) {
			telemetry.addLine( motorNames.get( i ) + " position: " + motor.get( i ).getCurrentPosition( ) );
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
			max += selected;

		if( selected > max )
			selected %= max;
	}
}
