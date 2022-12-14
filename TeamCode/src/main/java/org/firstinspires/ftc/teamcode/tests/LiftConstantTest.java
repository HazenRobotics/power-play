package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
@TeleOp(group = "")
public class LiftConstantTest extends OpMode {

	public static double constant = 0;
	ElapsedTime time = new ElapsedTime(  );

	MecanumDrive drive;
	Lift lift;


	@Override
	public void init( ) {
		lift = new Lift( hardwareMap, "lift", true, /* clawOffSet.getZ( ) */ 0, 39.25 / 25.4 / 2, 90, AngleUnit.DEGREES, 103.6, 1 );
		drive = new MecanumDrive( hardwareMap );
	}

	@Override
	public void loop( ) {

		double power = gamepad1.right_trigger - gamepad1.left_trigger + (time.milliseconds() % 1000 > 100 ? constant : 0);

		lift.setPower( power );

		drive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

		telemetry.addData( "height", lift.getPositionInch() );
		telemetry.addData( "actual power", lift.getPower() );
		telemetry.addData( "what power should be", power );
		telemetry.addData( "lt", gamepad1.left_trigger );
		telemetry.addData( "rt", gamepad1.right_trigger );
		telemetry.addData( "time", time.milliseconds() );
	}
}
