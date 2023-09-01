package org.firstinspires.ftc.teamcode.tests;

import static org.apache.commons.math3.util.MathUtils.TWO_PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.AlternativeSwervePIDController;

@Config
@TeleOp(group = "Test")
public class AlternativePIDSwerveTuning extends OpMode {

	DcMotorEx rotate;
	AlternativeSwervePIDController controller;

	public static double p, i, d, v = 1 / 6.54 , s = 0.1;

	public static double angle = 0;

	public final double TICKS_PER_REV = 537.7;

	double loopTime;
	boolean usingJoystick = true;

	double joyX, joyY, joyMag, joyAngle;



	@Override
	public void init( ) {
		rotate = hardwareMap.get( DcMotorEx.class, "rotate" );
		rotate.setDirection( DcMotorSimple.Direction.REVERSE );

		controller = new AlternativeSwervePIDController(  );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );
	}

	@Override
	public void loop( ) {
		controller.setValues( p, i, d, v, s );

//		if(usingJoystick) {
			joyX = gamepad1.left_stick_x;
			joyY = -gamepad1.left_stick_y;
			joyMag = Math.sqrt( joyY * joyY + joyX * joyX );

			if( joyMag > 0.1 ) angle = Math.atan2( joyY, joyX );

			angle += angle < 0 ? TWO_PI : 0;
//		}

		rotate.setPower( -controller.update( angle, convertTicksToAngle( rotate.getCurrentPosition() ), rotate.getVelocity( AngleUnit.RADIANS ) ) );

		if( gamepad1.x ) {
			angle = 0;
			rotate.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
			rotate.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		}

//		if( gamepad1.a ) {
//			usingJoystick = !usingJoystick;
//		}

		updateTelemetry( );

	}

	public void updateTelemetry( ) {
		telemetry.addData( "rotate pos RAD", convertTicksToAngle( rotate.getCurrentPosition() ) );
		telemetry.addData( "target angle RAD", angle );
		telemetry.addData( "error RAD", controller.getError( ) );
		telemetry.addData( "rotate pos DEG", Math.toDegrees( convertTicksToAngle( rotate.getCurrentPosition() ) ) );
		telemetry.addData( "target angle DEG", Math.toDegrees( angle ) );
		telemetry.addData( "error DEG", Math.toDegrees( controller.getError( ) ) );
		telemetry.addData( "usingJoystick", usingJoystick );

		double loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;

		telemetry.update( );
	}

	public double convertTicksToAngle( double ticks ) {
		double angle = (ticks % TICKS_PER_REV) * (TWO_PI / TICKS_PER_REV);
		return angle < 0 ? angle + TWO_PI : angle;
	}
}
