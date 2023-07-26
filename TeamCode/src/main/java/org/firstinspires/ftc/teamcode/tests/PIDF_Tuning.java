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
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.MotorType;

@Config
@TeleOp(group = "Test" )
public class PIDF_Tuning extends OpMode {

	DcMotorEx motor;

	public double TICKS_PER_REV = 751.8;

	public static double p = 0, i = 0, d = 0, f = 0;

	public static double target = 0;

	boolean manual = false;

	public double ticks_in_degrees = 751.8 / 360;

	PIDController controller;

	GamepadEvents gamepadEvents;

	@Override
	public void init( ) {
		controller = new PIDController( p, i, d );

		motor = hardwareMap.get( DcMotorEx.class, "arm" );
//		motor.setDirection( DcMotorSimple.Direction.REVERSE );
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		motor.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

		gamepadEvents = new GamepadEvents( gamepad1 );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

	}

	@Override
	public void loop( ) {

		if (!manual) {

			controller.setPID( p, i, d );

			double ff = Math.cos( Math.toRadians( target / ticks_in_degrees ) ) * f;

			motor.setPower( controller.calculate( motor.getCurrentPosition( ), target ) + ff );
		} else {
			motor.setPower( gamepad1.right_trigger - gamepad1.left_trigger );
		}

		if (gamepadEvents.a.onPress())
			manual = !manual;



		telemetry.addData( "arm pos", motor.getCurrentPosition() );
		telemetry.addData( "target angle", target );
		telemetry.update();

	}
}
