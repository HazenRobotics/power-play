package com.example.meepmeeppaths;

import static com.example.meepmeeppaths.teamcodeRequirements.PPField.THREE_HALVES_TILE;
import static com.example.meepmeeppaths.teamcodeRequirements.PPField.TILE_SIZE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.example.meepmeeppaths.teamcodeRequirements.MiniBot;
import com.example.meepmeeppaths.teamcodeRequirements.PPField;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class LinkageCycle implements MeepMeepPath {

	final boolean red = true, right = true;
	final int[] quadSign = MiniBot.getQuadrantSign( red, right );

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return getFlippedTrajectorySequence( drive, -1, 1 );
	}

	@Override
	public TrajectorySequence getFlippedTrajectorySequence( DriveShim drive, double xFlip, double yFlip ) {


		Vector2d parkPos = MiniBot.justParkInit( red, right);
		Vector2d conePos = MiniBot.getSignalPos( red, right );
		Pose2d medJunction = MiniBot.getJunctionOffsetPos( MiniBot.getAngleOnSide( red, right ), quadSign[0], quadSign[1] );
		Pose2d highJunction = MiniBot.getJunctionOffsetPos( MiniBot.getAngleOnSide( red, right ), 12, quadSign[1] == -1 ? 90 : 270 , quadSign[0], 0 );
		Pose2d pickUpPos = MiniBot.getJunctionOffsetPos( Math.toRadians( quadSign[0] == 1 ? 0 : 180 ), MiniBot.ROBOT_MAX_LENGTH - MiniBot.ROBOT_LENGTH / 2, new Vector2d( quadSign[0] * PPField.TILE_SIZE * 3, (red ? -1 : 1) * PPField.TILE_SIZE / 2 ) );

		System.out.println( MiniBot.getJunctionOffsetPos( MiniBot.getAngleOnSide( red, right ), 12, quadSign[1] == -1 ? 90 : 270 , quadSign[0], 0 ));

		return drive.trajectorySequenceBuilder( MiniBot.getStartPos( red, right ) )
				.lineToLinearHeading( new Pose2d (36, -48, Math.toRadians( 0 ) ))
				.lineToLinearHeading( new Pose2d( PPField.TILE_SIZE * 1.5, -PPField.TILE_SIZE * .65 , Math.toRadians( 90 ) ))
				.setReversed( true )
				.lineToLinearHeading(new Pose2d(  PPField.TILE_SIZE * 2 + 1, -PPField.TILE_SIZE * .65 , Math.toRadians( 0 )) )
//				.addTemporalMarker( () -> {
//					autoState = AutoState.PARK;
//				} )
				.build();

//		return drive.trajectorySequenceBuilder( MiniBot.getStartPos( red, right ) )
//				.splineTo( new Vector2d( 34, -54 ), Math.toRadians( 0 ) )
//				.splineToLinearHeading( new Pose2d(PPField.TILE_SIZE * 2 + 1, -PPField.TILE_SIZE * .65, Math.toRadians( 0 ) ), Math.toRadians( 0 ) )
//				.build();

	}
}
