package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.utils.localization.PPField.TILE_SIZE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;

@Autonomous(group="Autos")
public class JustParkStates extends LinearOpMode {

	MiniBot robot;
	AprilTagDetectionPipeline.SignalPosition signalPos;

	boolean red = true, right = true;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new MiniBot( this );

		robot.initSubsystems();

		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
		robot.drive.setPoseEstimate( robot.getStartPos( red, right ) );

		TrajectorySequence leftParkTrajectory = robot.getTrajectorySequenceBuilder( )
				.lineToLinearHeading( new Pose2d( TILE_SIZE * 1.5, -TILE_SIZE * 1.7, Math.toRadians( 90 ) ) )
				.lineToLinearHeading( new Pose2d( TILE_SIZE / 2 + TILE_SIZE * 0, -TILE_SIZE * 1.5, Math.toRadians( 90 ) ) )
				.build( );

		TrajectorySequence middleParkTrajectory = robot.getTrajectorySequenceBuilder( )
				.lineToLinearHeading( new Pose2d( TILE_SIZE * 1.5, -TILE_SIZE * 1.7, Math.toRadians( 90 ) ) )
				.build( );

		TrajectorySequence rightParkTrajectory = robot.getTrajectorySequenceBuilder( )
				.lineToLinearHeading( new Pose2d( TILE_SIZE * 1.5, -TILE_SIZE * 1.67, Math.toRadians( 90 ) ) )
				.lineToLinearHeading( new Pose2d( TILE_SIZE / 2 + TILE_SIZE * 2, -TILE_SIZE * 1.5, Math.toRadians( 90 ) ) )
				.build( );

		telemetry.addLine( "done" );
		telemetry.update( );

		while( !opModeIsActive( ) && opModeInInit( ) ) {
			signalPos = robot.signalUtil.getSignalPosition( );
			telemetry.addData( "position", signalPos );
			telemetry.update( );
		}

		waitForStart();
		robot.claw.close();
		robot.waitSeconds( 0.5 );

		robot.liftPower( 0.4 );

		robot.waitSeconds( 1 );

		if( signalPos == AprilTagDetectionPipeline.SignalPosition.LEFT )
			robot.drive.followTrajectorySequence( leftParkTrajectory );
		else if( signalPos == AprilTagDetectionPipeline.SignalPosition.RIGHT )
			robot.drive.followTrajectorySequence( rightParkTrajectory );
		else
			robot.drive.followTrajectorySequence( middleParkTrajectory );
	}
}
