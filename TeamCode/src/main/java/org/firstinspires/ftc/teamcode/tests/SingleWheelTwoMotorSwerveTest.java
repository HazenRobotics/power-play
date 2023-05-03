package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp
public class SingleWheelTwoMotorSwerveTest extends LinearOpMode {

	GamepadEvents controller1;
	DcMotorEx rotateMotor, driveMotor;
	Vector2d vector;
	double angle;
	double previousAngle;
	double angleIncrement;
	double loopTime;
	PIDController controller = new PIDController( 0.005, 0, 0.0001 );

	enum RotationStates {
		HALF,
		FULL
	}

	RotationStates state = RotationStates.FULL;


	@Override
	public void runOpMode( ) throws InterruptedException {
		rotateMotor = hardwareMap.get( DcMotorEx.class, "rotate" );
		rotateMotor.setDirection( DcMotorSimple.Direction.REVERSE );
		driveMotor = hardwareMap.get( DcMotorEx.class, "drive" );
		angleIncrement = 0;
		loopTime = 0;
		controller1 = new GamepadEvents( gamepad1 );

		waitForStart();

		while(!isStopRequested() && opModeIsActive()) {
			vector = new Vector2d( gamepad1.left_stick_x, -gamepad1.left_stick_y );

			if (controller1.a.onPress())
				state = state == RotationStates.HALF ? RotationStates.FULL : RotationStates.HALF;

			if (state == RotationStates.HALF) {
				if ( vector.magnitude() > 0.1 )
					angle = Math.abs(Math.toDegrees( vector.angle() ));
//				driveMotor.setPower( vector.magnitude() * Math.signum( vector.angle() ) );

				rotateMotor.setPower( controller.calculate( rotateMotor.getCurrentPosition(), convertAngleToTicks( angle ) ) );
			} else if (state == RotationStates.FULL) {
				if ( vector.magnitude() > 0.1 )
					angle = Math.toDegrees( vector.angle() );

				if ( previousAngle < -150 && angle > 150)
					angleIncrement--;
				else if (previousAngle > 150 && angle < -150) {
					angleIncrement++;
				}

				driveMotor.setPower( vector.magnitude() * .25 );

				rotateMotor.setPower( controller.calculate( rotateMotor.getCurrentPosition(), convertAngleToTicks( angle + (angleIncrement * 360) ) ) );

			}

			if (controller1.x.onPress()) {
				rotateMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
				rotateMotor.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
			}


			displayTelemetry();
			previousAngle = angle;
			controller1.update();
		}
	}

	public double convertTicksToAngle( double ticks ) {
		return (ticks / 537.7) * 360;
	}

	public double convertAngleToTicks( double angle ) {
		return (angle / 360) * 537.7;
	}

	public void displayTelemetry() {
		telemetry.addData( "joystick_x", gamepad1.left_stick_x  );
		telemetry.addData( "joystick_y", -gamepad1.left_stick_y  );
		telemetry.addData( "joystick angle", angle );
		telemetry.addData( "previous joystick angle", previousAngle );
		telemetry.addData( "angle difference", angle-previousAngle );
		telemetry.addData( "joystick magnitude", vector.magnitude() );
		telemetry.addData( "drive motor power", driveMotor.getPower() );
		telemetry.addData( "rotate motor angle", convertTicksToAngle( rotateMotor.getCurrentPosition() - convertAngleToTicks(angleIncrement * 360) ) );
		telemetry.addData( "rotate motor ticks", rotateMotor.getCurrentPosition() );
		telemetry.addData( "joy stick angle to ticks", convertAngleToTicks( angle ) );
		telemetry.addData( "increment", angleIncrement );

		double loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;

		telemetry.addData( "state", state );

		telemetry.update();
	}
}
