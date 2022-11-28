package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "PR bot", group = "TeleOp")

public class PRBot extends OpMode {

	DcMotor left,right, launcherLeft, launcherRight,bell;
	Servo blocker;
	double blockedPose = 0.5;
	double openPose = 0.75;
	@Override
	public void init( ) {

		telemetry.addData( "Mode", "Initiating robot..." );
		telemetry.update( );

		left = hardwareMap.get( DcMotor.class, "left" );
		right = hardwareMap.get( DcMotor.class, "right" );
		blocker = hardwareMap.get( Servo.class,"blocker" );

		launcherLeft = hardwareMap.get( DcMotor.class, "launchLeft" );
		launcherRight = hardwareMap.get( DcMotor.class, "launchRight" );

		bell = hardwareMap.get( DcMotor.class, "bell" );


		right.setDirection( DcMotorSimple.Direction.REVERSE );
		launcherLeft.setDirection( DcMotorSimple.Direction.REVERSE );

		telemetry.addData( "Mode", "waiting for start" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		setMotorPower( gamepad1.left_stick_y,gamepad1.right_stick_y );

		double power = gamepad1.left_trigger + gamepad1.right_trigger;
		launcherLeft.setPower( power );
		launcherRight.setPower( power );

		if( gamepad1.a && blocker.getPosition() != blockedPose) {
			blocker.setPosition( blockedPose );
		}
		if(!gamepad1.a && blocker.getPosition() != openPose) {
			blocker.setPosition( openPose );
		}

		telemetry.update( );
	}

	/**
	 * set directional power
	 *
	 * @param drive  power
	 */

	/**
	 * set individual power
	 *
	 * @param leftPower  power
	 * @param rightPower   power
	 */
	public void setMotorPower( double leftPower , double rightPower ) {
		left.setPower( leftPower );
		right.setPower( rightPower );
	}



	public void waitRobot( int mills ) {
		long startTime = System.currentTimeMillis( );
		while( (startTime + mills) > System.currentTimeMillis( ) )
			telemetry.update( );
	}

//	public void spinEyes(double power) {
//		leftEye.setPower( power );
//		rightEye.setPower( power );
//	}
}