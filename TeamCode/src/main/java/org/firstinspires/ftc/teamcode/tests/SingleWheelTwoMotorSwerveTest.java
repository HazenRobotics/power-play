package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp
public class SingleWheelTwoMotorSwerveTest extends LinearOpMode {

	double TICKS_PER_REV = 537.7;

	GamepadEvents controller1;
	DcMotorEx rotateMotor, driveMotor;
	Vector2d vector;
	double joystickAngle;
	double loopTime = 0;
	double difference;
	double maxPower = 0;

	PIDController controller = new PIDController( 0.06, 0, 0.001 );


	@Override
	public void runOpMode( ) throws InterruptedException {
		rotateMotor = hardwareMap.get( DcMotorEx.class, "rotate" );
		rotateMotor.setDirection( DcMotorSimple.Direction.REVERSE );
		driveMotor = hardwareMap.get( DcMotorEx.class, "drive" );
		controller1 = new GamepadEvents( gamepad1 );
		Robot.writeToDefaultFile( "start of file", false, false );

		waitForStart( );

		while( !isStopRequested( ) && opModeIsActive( ) ) {
			vector = new Vector2d( gamepad1.left_stick_x, -gamepad1.left_stick_y );

			if( vector.magnitude( ) > 0.1 )
				joystickAngle = Math.toDegrees( vector.angle( ) ) + 180;

//			driveMotor.setPower( vector.magnitude( ) * .25 );

			double motorAngle = convertTicksToAngle( rotateMotor.getCurrentPosition( ) );

			difference = -findShortestAngularTravel( motorAngle, joystickAngle );

			rotateMotor.setPower( controller.calculate( motorAngle, motorAngle + difference ) );
			maxPower = Math.max( maxPower, rotateMotor.getPower() );

			if( controller1.x.onPress( ) ) {
				rotateMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
				rotateMotor.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
			}

			displayTelemetry( );
			controller1.update( );
		}
	}

	public double convertTicksToAngle(double ticks) {
		double angle = (ticks % TICKS_PER_REV) * (360.0 / TICKS_PER_REV);
		return angle < 0 ? angle + 360 : angle;
	}
	
	public double convertAngleToTicks( double angle ) {
		return (angle / 360) * TICKS_PER_REV;
	}

	public double findShortestAngularTravel (double motAngle, double joyAngle ) {
		double temp = Math.abs( joyAngle - motAngle ) % 360;
		double distance = temp > 180 ? 360 - temp : temp;

		int sign = (motAngle - joyAngle >= 0 && motAngle - joyAngle <= 180) || (motAngle - joyAngle <= -180 && motAngle - joyAngle >= -360) ? 1 : -1;

		return distance * sign;
	}



	public void displayTelemetry( ) {
		telemetry.addData( "joystick_x", gamepad1.left_stick_x );
		telemetry.addData( "joystick_y", -gamepad1.left_stick_y );
		telemetry.addData( "joystick angle", joystickAngle );
		telemetry.addData( "joystick magnitude", vector.magnitude( ) );
		telemetry.addData( "drive motor power", driveMotor.getPower( ) );
		telemetry.addData( "rotate motor power", rotateMotor.getPower( ) );
		telemetry.addData( "rotate motor angle", convertTicksToAngle( rotateMotor.getCurrentPosition( ) ) );
		telemetry.addData( "rotate motor ticks", rotateMotor.getCurrentPosition( ) );
		telemetry.addData( "joy stick angle to ticks", convertAngleToTicks( joystickAngle ) );
		telemetry.addData( "difference", difference );
		telemetry.addData( "max power", maxPower );

		double loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;

		telemetry.update( );
	}
}
