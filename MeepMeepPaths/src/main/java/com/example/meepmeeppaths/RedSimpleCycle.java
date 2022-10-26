package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.example.meepmeeppaths.teamcodeRequirements.LifterBot;
import com.example.meepmeeppaths.teamcodeRequirements.PPField;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class RedSimpleCycle implements MeepMeepPath {

	float x = 2 * PPField.TILE_SIZE - LifterBot.ROBOT_WIDTH / 2 + 3 * PPField.TILE_CONNECTOR / 2;
	float y = PPField.HALF_FIELD - LifterBot.ROBOT_LENGTH / 2;

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( x, y, Math.toRadians( 270 ) ) )


				.lineTo( new Vector2d( 0, -16 ) )
				.build( );
	}

}