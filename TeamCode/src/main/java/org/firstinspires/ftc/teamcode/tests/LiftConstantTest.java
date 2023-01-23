package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
@TeleOp(group = "")
public class LiftConstantTest extends OpMode {

	public static double constant = 0;
	ElapsedTime time = new ElapsedTime(  );

	MiniBot robot;


	@Override
	public void init( ) {
		robot = new MiniBot( this );
	}

	@Override
	public void loop( ) {

		double power = gamepad1.right_trigger - gamepad1.left_trigger;

		robot.liftPower( power );

		telemetry.addData( "left height", robot.leftLift.getMotorPosition() );
		telemetry.addData( "righ height", robot.leftLift.getMotorPosition() );
		telemetry.addData( "actual power", robot.leftLift.getPower() );
		telemetry.addData( "what power should be", power );
		telemetry.addData( "lt", gamepad1.left_trigger );
		telemetry.addData( "rt", gamepad1.right_trigger );
		telemetry.addData( "time", time.milliseconds() );
	}
}
