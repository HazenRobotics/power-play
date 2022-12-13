package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TiltingClaw;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(group = "Test")
//@Disabled
public class MotorEquilibriumTest extends OpMode {

	Lift lift;
	TiltingClaw claw;
	String[] logFiles = new String[]{ "EmptyLift", "ConeLift", "ConeBeaconLift" };
	int logFile = 0;
	double power;
	double targetPosition;

	GamepadEvents gamepad;

	@Override
	public void init( ) {
		lift = new Lift( hardwareMap, "lift", true, 0, 39.25 / 25.4 / 2, 90, AngleUnit.DEGREES, 103.6, 1 );
		claw = new TiltingClaw( hardwareMap, "claw", "clawV", new double[]{ 0.61, 0.35 }, new double[]{ 0.73, 0.53, 0.3 } );
		gamepad = new GamepadEvents( gamepad1 );

		claw.setState( TiltingClaw.VerticalClawState.DEPLOYED );
		lift.setEncoder( Lift.EncoderState.WITH_ENCODER );
	}

	@Override
	public void loop( ) {
		if( gamepad.a.onPress( ) )
			claw.toggle( );

		if( gamepad.x.onPress( ) ) {
			Robot.writeAFile( logFiles[logFile] + ".csv", "TargetPos, ActualPos, Power", false, false );
			runTest( );
		}

		if( gamepad.y.onPress( ) )
			lift.setDefaultHeightPow( 0.7 );

		if( gamepad.dpad_up.onPress( ) )
			increment( 1, 0, 3 );

		if( gamepad.dpad_down.onPress( ) )
			increment( -1, 0, 3 );

		telemetry.addLine( "logFile: " + logFiles[logFile] );
		telemetry.addLine( );
		telemetry.addLine( "toggle claw: a" );
		telemetry.addLine( "reset lift height: y" );
		telemetry.addLine( "inc/dec logFile: dpad_up/down" );
		telemetry.addData( "setPower", power );
		telemetry.addData( "actualPower", lift.getPower() );
		telemetry.addData( "target pos", targetPosition );
		telemetry.addData( "velocity", lift.getVelocity() );
		telemetry.addData( "liftPos", lift.getMotorPositionInch() );
		telemetry.update( );
		gamepad.update( );
	}

	public void increment( int increment, int min, int max ) {
		logFile += increment;
		while( logFile < min )
			max += logFile;

		if( logFile > max )
			logFile %= max;
	}

	public boolean opModeIsActive( ) {
		return ((LinearOpMode) (OpMode) this).opModeIsActive( );
	}

	public void sleep( long millis ) {
		long startTime = System.currentTimeMillis( );
		while( System.currentTimeMillis( ) < startTime + millis /*&& opModeIsActive( )*/ ) ;
	}

	public void runTest( ) {
		new Thread( ( ) -> {
			power = 0.2;
			double heldTime = System.currentTimeMillis();

			for( targetPosition = 0; targetPosition < 50; targetPosition += 5 ) {
				lift.setHeightPower( power, targetPosition, false, false );
				while( Math.abs( lift.getVelocity( ) ) < 100 && heldTime + 2000 > System.currentTimeMillis()) {
					if (Math.abs(targetPosition - lift.getMotorPositionInch()) > 2) {
						lift.setHeightPower( power, targetPosition, false, false );
						heldTime = System.currentTimeMillis( );
					}
					lift.setEncoder( Lift.EncoderState.WITH_ENCODER );
					lift.setPower( power );
					power += 0.05 * (lift.getMotorPositionInch( ) <= targetPosition ? 1 : -1);
					sleep( 500 );
				}

				Robot.writeAFile( logFiles[logFile] + ".csv", targetPosition + ", " + lift.getMotorPosition() + ", " + power, true, false );
			}
		} ).start( );
	}

}
