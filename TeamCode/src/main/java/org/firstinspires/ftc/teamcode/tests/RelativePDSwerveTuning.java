package org.firstinspires.ftc.teamcode.tests;

import static org.apache.commons.math3.util.MathUtils.TWO_PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.SwervePod;
import org.firstinspires.ftc.teamcode.utils.SwervePDController;

@Config
@TeleOp(group = "Test")
public class RelativePDSwerveTuning extends OpMode {

	SwervePod pod;

	public static double p = 0, d = 0;

	public static double angle = 0;

	double loopTime;
	boolean usingJoystick = true;

	double joyX, joyY, joyMag, joyAngle;

	@Override
	public void init( ) {
		pod = new SwervePod( hardwareMap, "drive", false, "rotate", true, new double[]{ 0, 0 }, 537.7 );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );
	}

	@Override
	public void loop( ) {
		pod.setPD( p, d );

		if(usingJoystick) {
			joyX = gamepad1.left_stick_x;
			joyY = -gamepad1.left_stick_y;
			joyMag = Math.sqrt( joyY * joyY + joyX * joyX );

			if( joyMag > 0.1 ) angle = Math.atan2( joyY, joyX );

			angle += joyAngle < 0 ? TWO_PI : 0;
		}

		pod.setPodAngleTarget( angle );

		pod.updatePD( );

		if( gamepad1.x ) {
			angle = 0;
			pod.resetPodAngle( );
		}

		if( gamepad1.a ) {
			usingJoystick = !usingJoystick;
		}

			updateTelemetry( );

	}

	public void updateTelemetry( ) {
		telemetry.addData( "rotate pos RAD", pod.getPodAngle( ) );
		telemetry.addData( "target angle RAD", angle );
		telemetry.addData( "error RAD", pod.getError( ) );
		telemetry.addData( "rotate pos DEG", Math.toDegrees( pod.getPodAngle( ) ) );
		telemetry.addData( "target angle DEG", Math.toDegrees( angle ) );
		telemetry.addData( "error DEG", Math.toDegrees( pod.getError( ) ) );
		telemetry.addData( "usingJoystick", usingJoystick );

		double loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;

		telemetry.update( );
	}
}
