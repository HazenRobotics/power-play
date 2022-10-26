package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.LifterBot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "LifterTeleOp", group = "TeleOp")
//@Disabled
public class LifterTeleOp extends OpMode {

	LifterBot robot;
	GamepadEvents controller1;
	boolean opened = true;

	@Override
	public void init( ) {

		controller1 = new GamepadEvents( gamepad1 );

		telemetry.addData( "Mode", "Initiating robot..." );
		telemetry.update( );

		robot = new LifterBot( this );
		robot.verticalLift.setEncoder( Lift.EncoderState.WITHOUT_ENCODER );

		telemetry.addData( "Mode", "waiting for start??" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		robot.mecanumDrive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

		robot.verticalLift.setPower( gamepad1.right_trigger - gamepad1.left_trigger );

		if( controller1.a.onPress( ) ) {
			telemetry.addLine( "on a press" );
			if( opened )
				robot.claw.close( );
			else
				robot.claw.open( );
			opened = !opened;
		}

		displayTelemetry( );
		controller1.update( );

	}

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