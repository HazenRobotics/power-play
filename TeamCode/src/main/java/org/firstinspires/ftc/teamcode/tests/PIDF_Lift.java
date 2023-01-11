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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
@TeleOp
public class PIDF_Lift extends OpMode {

	private PIDController controller;

	public static double p = 0, i = 0, d = 0;
	public static double f = 0;

	public static int target = 0;

	private DcMotorEx motor;

	@Override
	public void init( ) {
		controller = new PIDController( p, i, d );
		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		motor = hardwareMap.get( DcMotorEx.class, "lift");
		motor.setDirection( DcMotorSimple.Direction.REVERSE );
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
	}

	@Override
	public void loop( ) {
		controller.setPID( p, i, d );
		int liftPos = motor.getCurrentPosition( );
		double pid = controller.calculate( liftPos, target);

		double ff = Drive.normalize( liftPos, 0, 900, 0, f );

		double power = pid + ff;

		motor.setPower( power );

		telemetry.addData( "pos", liftPos );
		telemetry.addData( "target", target );
		telemetry.addData( "pid", pid );
		telemetry.addData( "ff", ff );
		telemetry.addData( "power", power );
		telemetry.update();
	}
}
