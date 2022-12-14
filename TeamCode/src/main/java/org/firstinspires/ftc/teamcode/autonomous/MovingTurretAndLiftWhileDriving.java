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

		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )
				.addTemporalMarker( () -> {
					robot.lift.setHeightPower( 20, 0.75 );
				} )
				.forward( 20 )
				.turn( Math.toRadians( 90 ) )
				.addTemporalMarker( () -> {
					robot.turret.setRotationPower( 0.5, -90 );
				} )
				.forward( 20 )
				.turn( Math.toRadians( 90 ) )
				.addTemporalMarker( () -> {
					robot.turret.setRotationPower( 0.5, -180 );
				} )
				.forward( 20 )
				.turn( Math.toRadians( 90 ) )
				.addTemporalMarker( () -> {
					robot.turret.setRotationPower( 0.5, -90 );
				} )
				.forward( 20 )
				.turn( Math.toRadians( 90 ) )
				.addTemporalMarker( () -> {
					robot.turret.setRotationPower( 0.5, 90 );
				} )
				.build( );

		robot.drive.followTrajectorySequence( mainTrajectory );

	}
}
