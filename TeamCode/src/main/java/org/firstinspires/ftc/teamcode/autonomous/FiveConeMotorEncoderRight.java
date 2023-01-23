package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.subsystems.InternalIMU;
import org.firstinspires.ftc.teamcode.subsystems.SingleServoClaw;
import org.firstinspires.ftc.teamcode.subsystems.TiltingClaw;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;

@Autonomous(group = "")
//@Disabled
public class FiveConeMotorEncoderRight extends LinearOpMode {

	MiniBot robot;
	InternalIMU gyro;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new MiniBot( this );
		gyro = new InternalIMU( hardwareMap );

//		robot.mecanumDrive.setWheelDiameter( 3.77953 );

//		robot.mecanumDrive.setMotorDirections( DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD );

//		robot.claw.setState( TiltingClaw.VerticalClawState.DEPLOYED );
		robot.claw.setState( SingleServoClaw.ClawState.CLOSED );

		waitForStart( );

		deliverInitial();

		turnToDegrees( -90, 0.25 );

//		for( int i = 0; i < 5; i++ )
//			cycle( );

		while( !isStopRequested( ) ) {
			displayTelemetry( );
		}

	}

	public void displayTelemetry( ) {
//		telemetry.addData( "fl", robot.mecanumDrive.convertTicksDist( robot.mecanumDrive.frontLeft.getCurrentPosition( ) ) );
//		telemetry.addData( "bl", robot.mecanumDrive.convertTicksDist( robot.mecanumDrive.backLeft.getCurrentPosition( ) ) );
//		telemetry.addData( "fr", robot.mecanumDrive.convertTicksDist( robot.mecanumDrive.frontRight.getCurrentPosition( ) ) );
//		telemetry.addData( "br", robot.mecanumDrive.convertTicksDist( robot.mecanumDrive.backRight.getCurrentPosition( ) ) );
		telemetry.addData( "gyro", gyro.getOrientation( ) );
		telemetry.update( );
	}

	public void waitRobot( int mills ) {
		long startTime = System.currentTimeMillis( );
		while( (startTime + mills) > System.currentTimeMillis( ) ) ;
	}

	public void cycle( double coneHeight ) {
		driveDistance( 30, 0.25 );

//		while(robot.mecanumDrive.frontLeft.isBusy());

		robot.claw.setState( SingleServoClaw.ClawState.CLOSED );

		robot.junctionToLiftPos( PPField.Junction.HIGH );

		waitRobot( 500 );

		driveDistance( -30, 0.25 );

		robot.turret.setRotationPower( 0.5, -135 );
	}

	public void deliverInitial() {
		robot.junctionToLiftPos( PPField.Junction.HIGH );

		driveDistance( 48, 0.25 );

		waitRobot( 500 );

		robot.turret.setRotationPower( 0.5, -45 );

		robot.claw.setState( SingleServoClaw.ClawState.OPEN );

	}

	public void turnToDegrees( double angle, double power ) {
//		robot.mecanumDrive.setRunMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		if (angle > gyro.getOrientation().firstAngle)
			power *= -1;

//		robot.mecanumDrive.turn( power );

		new Thread( () -> {
			while( gyro.getOrientation( ).firstAngle > angle )
				displayTelemetry( );
//			robot.mecanumDrive.drive(0,0);
		}).start();

	}

	public void driveDistance( double distance, double power ) {
//		robot.mecanumDrive.setRunMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
//
//		robot.mecanumDrive.setTargetPosition( robot.mecanumDrive.convertDistTicks( 48 ) );
//
//		robot.mecanumDrive.setRunMode( DcMotor.RunMode.RUN_TO_POSITION );
//
//		robot.mecanumDrive.move( power );
	}
}
