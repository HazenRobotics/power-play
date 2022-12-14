package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.example.meepmeeppaths.teamcodeRequirements.MiniBot;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class testingPath implements MeepMeepPath {

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

		return drive.trajectorySequenceBuilder( MiniBot.getStartPos( red, right ) )
//		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )
//				.addTemporalMarker( () -> {
//					robot.lift.setHeightPower( 20, 0.75 );
//				} )
				.forward( 20 )
				.turn( Math.toRadians( 90 ) )
//				.addTemporalMarker( () -> {
//					robot.turret.setRotationPower( 0.5, -90 );
//				} )
				.forward( 20 )
				.turn( Math.toRadians( 90 ) )
//				.addTemporalMarker( () -> {
//					robot.turret.setRotationPower( 0.5, -180 );
//				} )
				.forward( 20 )
				.turn( Math.toRadians( 90 ) )
//				.addTemporalMarker( () -> {
//					robot.turret.setRotationPower( 0.5, -90 );
//				} )
				.forward( 20 )
				.turn( Math.toRadians( 90 ) )
//				.addTemporalMarker( () -> {
//					robot.turret.setRotationPower( 0.5, 90 );
//				} )
				.build( );

//		robot.drive.followTrajectorySequence( mainTrajectory );

	}
}
