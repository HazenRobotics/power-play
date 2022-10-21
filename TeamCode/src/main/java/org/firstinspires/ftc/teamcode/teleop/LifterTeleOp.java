package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.LifterBot;

@TeleOp(name = "LifterTeleOp2", group = "TeleOp")
//@Disabled
public class LifterTeleOp extends OpMode {

	LifterBot robot;
	boolean aWasPressed = false;
	double closedPosition = 0.5;
	double openPosition = 0.9;

	@Override
	public void init( ) {

		telemetry.addData( "Mode", "Initiating robot..." );
		telemetry.update( );

		robot = new LifterBot( this );

		telemetry.addData( "Mode", "waiting for start??" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		robot.mecanumDrive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

		//rotate le
		displayTelemetry( );


	}


//    public void claw( ) {
//        if( clawServo.getPosition( ) == 0 ) {
//            clawServo.setPosition( 0.5 );
//        } else {
//            clawServo.setPosition( 0 );
//        }
//    }

	public void waitRobot( int mills ) {
		long startTime = System.currentTimeMillis( );
		while( (startTime + mills) > System.currentTimeMillis( ) ) {
			telemetry.update( );
		}
	}

	public void displayTelemetry( ) {
		telemetry.addData( "ly: ", -gamepad1.left_stick_y );
		telemetry.addData( "lx: ", gamepad1.left_stick_x );
		telemetry.addData( "rx: ", gamepad1.right_stick_x );
		telemetry.addLine( "" );


		telemetry.update( );
	}
}