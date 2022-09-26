package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "PR bot", group = "TeleOp")
//@Disabled
public class PRBot extends OpMode {

	DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, launcherLeft, launcherRight;
	Servo launcherServo;
	boolean aWasPressed = false;

	@Override
	public void init( ) {

		telemetry.addData( "Mode", "Initiating robot..." );
		telemetry.update( );

		frontLeftMotor = hardwareMap.get( DcMotor.class, "frontLeft" );
		backLeftMotor = hardwareMap.get( DcMotor.class, "backLeft" );
		frontRightMotor = hardwareMap.get( DcMotor.class, "frontRight" );
		backRightMotor = hardwareMap.get( DcMotor.class, "backRight" );
		launcherLeft = hardwareMap.get( DcMotor.class, "launchLeft" );
		launcherRight = hardwareMap.get( DcMotor.class, "launchRight" );

		launcherServo = hardwareMap.servo.get( "launcher" );

		frontLeftMotor.setDirection( DcMotorSimple.Direction.REVERSE );
		backLeftMotor.setDirection( DcMotorSimple.Direction.REVERSE );
		launcherLeft.setDirection( DcMotorSimple.Direction.REVERSE );

		telemetry.addData( "Mode", "waiting for start" );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		move( gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

		if( aWasPressed && !gamepad1.a ) {
			ring( );
		}
		double power = gamepad1.left_trigger + gamepad1.right_trigger;
		if( power > 1 ) {
			power = 1;
		}
		launcherLeft.setPower( power );
		launcherRight.setPower( power * 0.95 );
		aWasPressed = gamepad1.a;

		telemetry.update( );
	}

	/**
	 * set directional power
	 *
	 * @param drive  power
	 * @param strafe strafe power
	 * @param rotate power
	 */
	public void move( double drive, double strafe, double rotate ) {
		double frontLeftPower = drive + strafe + rotate;
		double backLeftPower = drive - strafe + rotate;
		double frontRightPower = drive - strafe - rotate;
		double backRightPower = drive + strafe - rotate;

		setMotorPower( frontLeftPower, backLeftPower, frontRightPower, backRightPower );
	}

	/**
	 * set individual power
	 *
	 * @param frontLeftPower  power
	 * @param backLeftPower   power
	 * @param frontRightPower power
	 * @param backRightPower  power
	 */
	public void setMotorPower( double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower ) {
		frontLeftMotor.setPower( frontLeftPower );
		backLeftMotor.setPower( backLeftPower );
		frontRightMotor.setPower( frontRightPower );
		backRightMotor.setPower( backRightPower );
	}

	public void ring( ) {
		launcherServo.setPosition( 0.5 );
		waitRobot( 500 );
		launcherServo.setPosition( 1 );
	}

	public void waitRobot( int mills ) {
		long startTime = System.currentTimeMillis( );
		while( (startTime + mills) > System.currentTimeMillis( ) ) {
			telemetry.update( );
		}

	}
}