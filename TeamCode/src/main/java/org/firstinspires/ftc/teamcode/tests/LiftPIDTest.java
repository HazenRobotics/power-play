package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
@TeleOp(group = "")
public class LiftPIDTest extends OpMode {

	MiniBot robot;

	PIDController controllerL;
	PIDController controllerR;

	public static int target;
	public static int p, i, d;

	@Override
	public void init( ) {
		robot = new MiniBot( this );
		controllerL = new PIDController( 0,0,0 );
		controllerR = new PIDController( 0,0,0 );
	}

	@Override
	public void loop( ) {
		controllerL.setPID( p, i, d );
		controllerR.setPID( p, i, d );

		robot.leftLift.setPower( controllerL.calculate( robot.leftLift.getMotorPosition(), target ) );
		robot.rightLift.setPower( controllerR.calculate( robot.rightLift.getMotorPosition(), target ) );


		telemetry.addData( "left height", robot.leftLift.getMotorPosition() );
		telemetry.addData( "righ height", robot.leftLift.getMotorPosition() );
		telemetry.addData( "actual power", robot.leftLift.getPower() );
	}
}
