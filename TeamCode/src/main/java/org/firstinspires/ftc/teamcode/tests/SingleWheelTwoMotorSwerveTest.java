package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SwervePod;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp
public class SingleWheelTwoMotorSwerveTest extends LinearOpMode {

	double TICKS_PER_REV = 537.7;

	GamepadEvents controller1;
	SwervePod pod;

	double loopTime = 0;
	double forwardDifference, reverseDifference, usedDifference;
	double reverseDirection;
	double joyAngle, joyMag, joyX, joyY;


	@Override
	public void runOpMode( ) throws InterruptedException {
		pod = new SwervePod( hardwareMap, "drive" , false, "rotate", true, new double[] {0.5, 0.04}, 537.7 );

		controller1 = new GamepadEvents( gamepad1 );
		Robot.writeToDefaultFile( "start of file", false, false );

		waitForStart( );

		while( !isStopRequested( ) && opModeIsActive( ) ) {

			joyX = gamepad1.left_stick_x;
			joyY = -gamepad1.left_stick_y;
			joyMag = Math.sqrt( joyY * joyY + joyX * joyX );

			if( joyMag > 0.1 ) joyAngle = Math.atan2( joyY, joyX );

			joyAngle += joyAngle < 0 ? 360 : 0;

			pod.setPodAngleTarget( joyAngle );

			pod.updatePD();

//			forwardDifference = findShortestAngularTravel( joyAngle, motorAngle );
//			reverseDifference = findShortestAngularTravel( joyAngle, motorAngle + 180 );

//			if( Math.abs( reverseDifference ) < Math.abs( forwardDifference ) ) {
//				usedDifference = reverseDifference;
//				reverseDirection = -1;
//			} else {
//				usedDifference = forwardDifference;
//				reverseDirection = 1;
//			}

//			driveMotor.setPower( joyMag * reverseDirection );

//			if( controller1.x.onPress( ) ) {
//				rotateMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
//				rotateMotor.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
//			}

			displayTelemetry( );
			controller1.update( );
		}
	}

	public void displayTelemetry( ) {
		telemetry.addData( "joystick_x", gamepad1.left_stick_x );
		telemetry.addData( "joystick_y", -gamepad1.left_stick_y );
		telemetry.addData( "joystick angle", joyAngle );
		telemetry.addData( "joystick magnitude", joyMag );
		telemetry.addData( "drive motor power", pod.driveMotor.getPower( ) );
		telemetry.addData( "rotate motor power", pod.rotateMotor.getPower( ) );
		telemetry.addData( "rotate motor angle", pod.getPodAngle() );
		telemetry.addData( "forwardDifference", forwardDifference );
		telemetry.addData( "reverseDifference", reverseDifference );
		telemetry.addData( "usedDifference", usedDifference );
		telemetry.addData( "reverseDirection", reverseDirection );

		double loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;

		telemetry.update( );
	}
}
