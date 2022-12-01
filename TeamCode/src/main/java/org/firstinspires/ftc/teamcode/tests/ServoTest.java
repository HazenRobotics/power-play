package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

import java.util.ArrayList;

@TeleOp(name = "ServoTest", group = "TeleOp")
//@Disabled
public class ServoTest extends OpMode {


	ArrayList<String> servoNames = new ArrayList<>( );
	ArrayList<Servo> servo = new ArrayList<>( );

	GamepadEvents controller;
	int selected = 0;
	int maxSelect;

	@Override
	public void init( ) {
		controller = new GamepadEvents( gamepad1 );
		String[] names = { "claw", "clawV", "clawH", "clawR", "left", "right" };

		for( int i = 0; i < names.length; i++ ) {
			try {
				servo.add( hardwareMap.servo.get( names[i] ) );
				this.servoNames.add( names[i] );

			} catch( IllegalArgumentException e ) {
				telemetry.addLine( "couldn't find " + names[i] );
			}
		}

		maxSelect = servo.size( ) - 1;
		telemetry.addLine( "added " + (maxSelect) + " servo" + (maxSelect == 0 ? "" : "s") );
		telemetry.update( );

	}

	@Override
	public void loop( ) {

		servo.get( selected ).setPosition( -controller.right_stick_y );

		if( controller.dpad_up.onPress( ) ) {
			increment( 1, maxSelect );
		} else if( controller.dpad_down.onPress( ) ) {
			increment( -1, maxSelect );
		}

		telemetry.addLine( "selected: (" + selected + ") " + servoNames.get( selected ) );
		for( int i = 0; i < servo.size( ); i++ ) {
			telemetry.addLine( servoNames.get( i ) + " position: " + servo.get( i ).getPosition( ) );
		}
		telemetry.update( );
		controller.update( );
	}

	public void increment( int increment, int max ) {
		increment( increment, 0, max );
	}

	// inclusive
	public void increment( int increment, int min, int max ) {
		selected += increment;
		int range = 1 + max - min;
		while( selected < min )
			selected += range ;

		if( selected > max )
			selected %= range;
	}
}
