package com.example.meepmeeppaths;

import static com.example.meepmeeppaths.teamcodeRequirements.MiniBot.ROBOT_LENGTH;
import static com.example.meepmeeppaths.teamcodeRequirements.MiniBot.ROBOT_MAX_LENGTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.example.meepmeeppaths.teamcodeRequirements.MiniBot;
import com.example.meepmeeppaths.teamcodeRequirements.PPField;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MethodTest implements MeepMeepPath {

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return getFlippedTrajectorySequence( drive, 1, 1 );
	}

	public static boolean red = true, right = false;
	public static int[] quadSign = MiniBot.getQuadrantSign( red, right );

	@Override
	public TrajectorySequence getFlippedTrajectorySequence( DriveShim drive, double xFlip, double yFlip ) {
		double cycleDist = totalTitle * 1.5;
		float x = -(float) ((2 * totalTitle - MiniBot.ROBOT_WIDTH / 2 + 3 * PPField.TILE_CONNECTOR / 2) * xFlip);
		float y = (float) (-(PPField.HALF_FIELD - ROBOT_LENGTH / 2) * xFlip);
		double xFlipR = -((xFlip - 1) / 2);
		double yFlipR = -((xFlip - 1) / 2);
		Pose2d conePose = new Pose2d( (-totalTitle * 2.5) * xFlip, (-totalTitle / 2) * xFlip, Math.toRadians( 180 + (180 * xFlipR) ) );


		return drive.trajectorySequenceBuilder( MiniBot.getJunctionOffsetPos( MiniBot.getAngleOnSide( red, right ), ROBOT_MAX_LENGTH - ROBOT_LENGTH / 2, PPField.getJunctionPose( quadSign[0], quadSign[1], false ) )
				)
				// Duck spin
				.forward( 0.1 )
				.build( );

	}


}