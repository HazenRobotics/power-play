package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.subsystems.TiltingClaw;
import org.firstinspires.ftc.teamcode.subsystems.TwoAxesClaw;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;

@Autonomous(group = "")
public class MovingTurretAndLiftWhileDriving extends LinearOpMode {

	MiniBot robot;

	final boolean red = true, right = true;

	@Override
	public void runOpMode( ) throws InterruptedException {

		robot = new MiniBot( this );

		robot.initSubsystems();

		waitForStart();
		robot.signalUtil.stopCamera( );

		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
		robot.drive.setPoseEstimate( robot.getStartPos( red, right ) );

		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )
				.forward( 20 )
				.turn( Math.toRadians( 90 ) )
				.forward( 20 )
				.turn( Math.toRadians( 90 ) )
				.forward( 20 )
				.turn( Math.toRadians( 90 ) )
				.forward( 20 )
				.turn( Math.toRadians( 90 ) )
				.build( );

		robot.drive.followTrajectorySequence( mainTrajectory );

	}
}
