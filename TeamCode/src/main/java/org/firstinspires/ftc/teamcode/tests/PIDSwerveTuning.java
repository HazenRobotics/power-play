package org.firstinspires.ftc.teamcode.tests;

import static org.apache.commons.math3.util.MathUtils.TWO_PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SwervePodTwoMotor;

@Config
@TeleOp(group = "Test")
public class PIDSwerveTuning extends OpMode {

	SwervePodTwoMotor pod;

	public static double p = 0, i = 0, d = 0;

	public static double angle = 0;

	double loopTime;
	boolean usingJoystick = true;

	double joyX, joyY, joyMag, joyAngle;

	@Override
	public void init( ) {
		pod = new SwervePodTwoMotor( hardwareMap, "drive", false, "rotate", true, new double[]{ 0, 0, 0 }, 537.7 );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );
		Robot.writeAFile("SwervePID.csv", "begin", true, true );
	}

	@Override
	public void loop( ) {
		pod.setPID( p, i, d );

//		if(usingJoystick) {
			joyX = gamepad1.left_stick_x;
			joyY = -gamepad1.left_stick_y;
			joyMag = Math.sqrt( joyY * joyY + joyX * joyX );

			if( joyMag > 0.1 ) angle = Math.atan2( joyY, joyX );

			angle += angle < 0 ? TWO_PI : 0;
//		}

		pod.getError();

		pod.setPodAngleTarget( angle );

		pod.updateRotatePD( );

		if( gamepad1.x ) {
			angle = 0;
			pod.resetPodAngle( );
		}

//		if( gamepad1.a ) {
//			usingJoystick = !usingJoystick;
//		}

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

		String dataToWrite = pod.rotateMotor.getPower() + ", "
				+ pod.rotateMotor.getCurrent( CurrentUnit.AMPS ) + ", "
				+ pod.rotateMotor.getVelocity( AngleUnit.RADIANS ) + ", "
				+ pod.getPodAngle() + ", "
				+ pod.getPodAngleTarget( ) + ", "
				+ pod.getError();

		Robot.writeAFile("SwervePID.csv", dataToWrite, true, true );

		double loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;

		telemetry.update( );
	}
}
