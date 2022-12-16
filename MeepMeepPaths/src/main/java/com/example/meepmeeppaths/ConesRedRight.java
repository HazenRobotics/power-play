package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.example.meepmeeppaths.teamcodeRequirements.MiniBot;
import com.example.meepmeeppaths.teamcodeRequirements.PPField;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class ConesRedRight implements MeepMeepPath {

	final boolean red = true, right = true;
	final int[] quadSign = MiniBot.getQuadrantSign( red, right );

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
//		robot.initSubsystems( );
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

		Vector2d parkPos = MiniBot.justParkInit( red, right);
		Vector2d conePos = MiniBot.getSignalPos( red, right );
		Pose2d medJunction = MiniBot.getJunctionOffsetPos( MiniBot.getAngleOnSide( red, right ), quadSign[0], quadSign[1] );
		Pose2d highJunction = MiniBot.getJunctionOffsetPos( MiniBot.getAngleOnSide( red, right ), 12, quadSign[1] == -1 ? 90 : 270 , quadSign[0], 0 );
		Pose2d pickUpPos = MiniBot.getJunctionOffsetPos( Math.toRadians( quadSign[0] == 1 ? 0 : 180 ), MiniBot.ROBOT_MAX_LENGTH - MiniBot.ROBOT_LENGTH / 2, new Vector2d( quadSign[0] * PPField.TILE_SIZE * 3, (red ? -1 : 1) * PPField.TILE_SIZE / 2 ) );

		System.out.println( MiniBot.getJunctionOffsetPos( MiniBot.getAngleOnSide( red, right ), 12, quadSign[1] == -1 ? 90 : 270 , quadSign[0], 0 ));

//		robot.signalUtil.stopCamera( );

//		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
//		robot.drive.setPoseEstimate( robot.getStartPos( red, right ) );

//		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )
////				.lineTo( conePos )
//				.lineToLinearHeading( new Pose2d( conePos.getX(), conePos.getY(), quadSign[0] == 1 ? 0 : 180 ) ) // straighten out and center
//				.lineToLinearHeading( highJunction ) // move to medium junction
////				.addTemporalMarker( ( ) -> {
////					// move lift up async
////					robot.junctionToLiftPos( PPField.Junction.HIGH );
////				} )
//				.waitSeconds( 1.5 ) // wait
////				.addTemporalMarker( ( ) -> {
////					// deploy & open claw
////					robot.claw.setState( TiltingClaw.VerticalClawState.DEPLOYED );
////					robot.claw.setState( TiltingClaw.ClawState.OPEN );
////					// return lift down async
////					robot.junctionToLiftPos( PPField.Junction.GROUND );
////					// stow claw
////					robot.claw.setState( TiltingClaw.VerticalClawState.STOWED );
////				} )
//				// grab new cone
//				/*.lineToLinearHeading( pickUpPos ) // straighten out and center
//				.addTemporalMarker( ( ) -> {
//					// deploy & open claw
//					robot.claw.setState( TwoAxesClaw.ClawState.OPEN );
//					robot.claw.setState( TwoAxesClaw.VerticalClawState.DEPLOYED );
//				} )
//				.waitSeconds( 0.5 ) // wait
//				.addTemporalMarker( ( ) -> {
//					// close claw
//					robot.claw.setState( TwoAxesClaw.ClawState.CLOSED );
//					// move lift up async
//					robot.junctionToLiftPos( PPField.Junction.HIGH );
//				} )*/
//				.lineTo( parkPos ) // go to park position
//				.build( );
//
//		robot.drive.followTrajectorySequence( mainTrajectory );


//		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )
		return drive.trajectorySequenceBuilder( MiniBot.getStartPos( red, right ) )
//				.lineTo( conePos )
				.lineToLinearHeading( new Pose2d( conePos.getX(), conePos.getY(), quadSign[0] == 1 ? 0 : 180 ) ) // straighten out and center
				.lineToLinearHeading( highJunction ) // move to medium junction
//				.addTemporalMarker( ( ) -> {
//					// move lift up async
//					robot.junctionToLiftPos( PPField.Junction.HIGH );
//				} )
				.waitSeconds( 1.5 ) // wait
//				.addTemporalMarker( ( ) -> {
//					// deploy & open claw
//					robot.claw.setState( TiltingClaw.VerticalClawState.DEPLOYED );
//					robot.claw.setState( TiltingClaw.ClawState.OPEN );
//					// return lift down async
//					robot.junctionToLiftPos( PPField.Junction.GROUND );
//					// stow claw
//					robot.claw.setState( TiltingClaw.VerticalClawState.STOWED );
//				} )
				// grab new cone
				.lineToLinearHeading( pickUpPos ) // straighten out and center
				/*.addTemporalMarker( ( ) -> {
					// deploy & open claw
					robot.claw.setState( TwoAxesClaw.ClawState.OPEN );
					robot.claw.setState( TwoAxesClaw.VerticalClawState.DEPLOYED );
				} )
				.waitSeconds( 0.5 ) // wait
				.addTemporalMarker( ( ) -> {
					// close claw
					robot.claw.setState( TwoAxesClaw.ClawState.CLOSED );
					// move lift up async
					robot.junctionToLiftPos( PPField.Junction.HIGH );
				} )*/
//				.lineTo( parkPos ) // go to park position
				.build( );

//		robot.drive.followTrajectorySequence( mainTrajectory );

	}
}
