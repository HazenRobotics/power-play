package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.math3.analysis.function.Min;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.Drive;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MotorType;

@Config
@TeleOp(group = "Test" )
public class PIDF_Tuning extends OpMode {

	DcMotorEx motor;

	public double TICKS_PER_REV = 537.7;

	public static double p = 0, i = 0, d = 0;

	public static double angle = 0;

	PIDController controller;

	@Override
	public void init( ) {
		controller = new PIDController( p, i, d );
		motor = hardwareMap.get( DcMotorEx.class, "rotate" );
		motor.setDirection( DcMotorSimple.Direction.REVERSE );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

	}

	@Override
	public void loop( ) {
		controller.setPID( p, i, d );

		double motorAngle = convertTicksToAngle( motor.getCurrentPosition( ) );

		double difference = -findShortestAngularTravel( motorAngle, angle );

		motor.setPower( controller.calculate( motorAngle, motorAngle + difference ) );

		if (gamepad1.x) {
			angle = 0;
			motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
			motor.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		}

		telemetry.addData( "rotate pos", convertTicksToAngle( motor.getCurrentPosition() ) );
		telemetry.addData( "target angle", angle );
		telemetry.update();

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
}
