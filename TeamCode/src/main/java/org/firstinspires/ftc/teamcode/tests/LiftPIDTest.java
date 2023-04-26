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

	public static int target;

	@Override
	public void init( ) {
		robot = new MiniBot( this );
	}

	@Override
	public void loop( ) {
		robot.setLiftTargetInches( target );

		robot.leftLift.updatePID( 1 );
		robot.rightLift.updatePID( 1 );

		telemetry.addData( "left height", robot.leftLift.getMotorPosition() );
		telemetry.addData( "righ height", robot.rightLift.getMotorPosition() );
		telemetry.addData( "actual power", robot.leftLift.getPower() );
	}
}
