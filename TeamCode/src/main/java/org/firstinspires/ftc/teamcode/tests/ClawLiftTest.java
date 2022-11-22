package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "ClawLiftTest", group = "Test")
@Disabled
public class ClawLiftTest extends OpMode {

	Claw claw;
	Lift lift;
	GamepadEvents controller1;
	boolean opened = true;

	@Override
	public void init( ) {

		controller1 = new GamepadEvents( gamepad1 );

		telemetry.addData( "Mode", "Initiating robot..." );
		telemetry.update( );

		claw = new Claw( hardwareMap, "lCLaw", "rClaw", new double[]{ 0.5, 0.75 }, new double[]{ 05, 0.57 } );
		lift = new Lift( hardwareMap, "vLift", false, 0, 39.25 / 25.4 / 2, 0, AngleUnit.DEGREES );

		telemetry.addData( "Mode", "waiting for start??" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		lift.setPower( gamepad1.right_trigger - gamepad1.left_trigger );

		if( controller1.a.onPress( ) ) {
			telemetry.addLine( "on a press" );
			if( opened )
				claw.close( );
			else
				claw.open( );
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

		telemetry.addData( "lt: ", gamepad1.left_trigger );
		telemetry.addData( "rt: ", gamepad1.right_trigger );
		telemetry.addData( "claw status: ", opened );
		telemetry.addLine( "" );

		telemetry.update( );
	}
}