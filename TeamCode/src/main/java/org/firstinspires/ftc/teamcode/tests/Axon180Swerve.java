package org.firstinspires.ftc.teamcode.tests;

import static org.apache.commons.math3.util.MathUtils.TWO_PI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.util.FastMath;

@TeleOp
public class Axon180Swerve extends LinearOpMode {

	Servo rotate;
	DcMotorEx drive;
	double joyAngle, joyMag, joyX, joyY, reverseMotor = 1;

	@Override
	public void runOpMode( ) throws InterruptedException {
		rotate = hardwareMap.get( Servo.class, "rotate" );
		drive = hardwareMap.get( DcMotorEx.class, "drive" );

		waitForStart();

		while( opModeIsActive() && !isStopRequested() ) {
			joyX = gamepad1.left_stick_x;
			joyY = -gamepad1.left_stick_y;
			joyMag = Math.sqrt( joyY * joyY + joyX * joyX );
			if( joyMag > 0.1 ) joyAngle = -Math.atan2( joyY, joyX );

			if( joyAngle < 0 ) {
				joyAngle += FastMath.PI;
				reverseMotor = -1;
			} else {
				reverseMotor = 1;
			}

			drive.setPower( joyMag * reverseMotor );
			rotate.setPosition( joyAngle / FastMath.PI );
			updateTelemetry();
		}

	}

	public void updateTelemetry() {
		telemetry.addData( "joyAngle", joyAngle );
		telemetry.update();
	}
}
