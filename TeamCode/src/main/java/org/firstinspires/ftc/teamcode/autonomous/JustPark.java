package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;
import org.firstinspires.ftc.teamcode.vision.SignalDetector;

@Autonomous( group = "A" )
public class JustPark extends LinearOpMode {

	MiniBot robot;

	@Override
	public void runOpMode( ) throws InterruptedException {

		Robot.createMatchLogFile( getClass( ).getSimpleName( ) );

		robot = new MiniBot( this );

		robot.signalUtil.init( );

		telemetry.addLine( "Ready!" );
		telemetry.update( );

		while( !isStopRequested( ) && !isStarted( ) ) {
			telemetry.addData( "Element position", robot.signalUtil.getSignalPosition( ) );
			telemetry.update( );
		}

		waitForStart( );

		SignalDetector.SignalPosition signalPosition = robot.signalUtil.getSignalPosition( );

		double tilePos = 1;
		if( signalPosition == SignalDetector.SignalPosition.LEFT )
			tilePos = 0;
		else if( signalPosition == SignalDetector.SignalPosition.RIGHT )
			tilePos = 2;
		Vector2d parkPos = new Vector2d( PPField.TILE_SIZE * 0.5 + PPField.TILE_SIZE * tilePos, -3 * PPField.TILE_SIZE / 2 );

		robot.signalUtil.stopCamera( );

		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
		robot.drive.setPoseEstimate( new Pose2d( PPField.TILE_SIZE + MiniBot.ROBOT_MAX_WIDTH / 2, -(3 * PPField.TILE_SIZE - MiniBot.ROBOT_LENGTH / 2), Math.toRadians( 90 ) ) );

		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )
				.lineTo( new Vector2d( 3 / 2.0 * PPField.TILE_SIZE, -3 / 2.0 * PPField.TILE_SIZE ) )
				.lineTo( parkPos )
				.build( );

		robot.drive.followTrajectorySequence( mainTrajectory );

	}
}
