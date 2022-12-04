package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.example.meepmeeppaths.teamcodeRequirements.MiniBot;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class ConesRedRight implements MeepMeepPath {

	final boolean red = true, right = true;

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return getFlippedTrajectorySequence( drive, -1, 1 );
	}

	@Override
	public TrajectorySequence getFlippedTrajectorySequence( DriveShim drive, double xFlip, double yFlip ) {


//		Robot.createMatchLogFile( getClass( ).getSimpleName( ) );
//
//		robot = new MiniBot( this );
//
//		robot.signalUtil.init( );
//		robot.claw.close( );
//
//		telemetry.addLine( "Ready!" );
//		telemetry.update( );
//
//		while( !isStopRequested( ) && !isStarted( ) ) {
//			telemetry.addData( "Element position", robot.signalUtil.getSignalPosition( ) );
//			telemetry.update( );
//		}
//
//		waitForStart( );
//
//		robot.junctionToLiftPos( PPField.Junction.GROUND );
//
//		Vector2d parkPos = robot.parkPosInit( red, right );
//		Vector2d conePos = MiniBot.getSignalPos( red, right );
//		Pose2d medJunction = MiniBot.getJunctionOffsetPos( MiniBot.getAngleOnSide( red, right ), quadSign[0], quadSign[1]);
//
//		robot.signalUtil.stopCamera( );
//
//		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
//		robot.drive.setPoseEstimate( robot.getStartPos( red, right ) );

		Vector2d conePos = MiniBot.getSignalPos( red, right );
		Vector2d parkPos = MiniBot.justParkInit( red, right );

//		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )
		return drive.trajectorySequenceBuilder( MiniBot.getStartPos( red, right ) )
				.lineTo( conePos )
				.lineTo( parkPos )
				.build( );

//		robot.drive.followTrajectorySequence( mainTrajectory );

	}
}
