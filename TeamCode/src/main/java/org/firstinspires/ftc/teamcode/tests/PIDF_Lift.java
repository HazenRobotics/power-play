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
@TeleOp(group = "Test" )
public class PIDF_Lift extends OpMode {

	private PIDController leftController;
	private PIDController rightController;

	public static double p = 0, i = 0, d = 0;
	public static double f = 0;

	public static int target = 0;

	private Lift left;
	private Lift right;

	@Override
	public void init( ) {
		leftController = new PIDController( p, i, d );
		rightController = new PIDController( p, i, d );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		left = new Lift( hardwareMap, "leftLift", false, /* clawOffSet.getZ( ) */ 0, 39.25 / 25.4 / 2, 90, AngleUnit.DEGREES, 384.5, 1, new PIDController(0.02, 0, 0.00012) );
		right = new Lift( hardwareMap, "rightLift", true, /* clawOffSet.getZ( ) */ 0, 39.25 / 25.4 / 2, 90, AngleUnit.DEGREES, 384.5, 1, new PIDController(0.02, 0, 0.00012) );

	}

	@Override
	public void loop( ) {
		leftController.setPID( p, i, d );
		rightController.setPID( p, i, d );

		int leftLiftPos = left.getPosition( );
		int rightLiftPos = right.getPosition( );

		double leftPID = leftController.calculate( leftLiftPos, target );
		double rightPID = rightController.calculate( rightLiftPos, target);

		double leftPower = leftPID + f;
		double rightPower = rightPID + f;

		left.setPower( leftPower );
		right.setPower( rightPower );

		telemetry.addData( "left pos", leftLiftPos );
		telemetry.addData( "right pos", rightLiftPos );
		telemetry.addData( "target", target );
		telemetry.addData( "left PID", leftPID );
		telemetry.addData( "right PID", rightPID );
		telemetry.update();
	}
}
