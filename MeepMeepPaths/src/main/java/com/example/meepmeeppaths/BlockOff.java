package com.example.meepmeeppaths;

import com.example.meepmeeppaths.teamcodeRequirements.PPField;
import com.example.meepmeeppaths.teamcodeRequirements.LifterBot;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class BlockOff implements MeepMeepPath {

	float x = -(float) (2 * tileSize - LifterBot.ROBOT_WIDTH / 2 + 3 * tileConnector / 2);
	float y = PPField.HALF_FIELD - LifterBot.ROBOT_LENGTH / 2;
	Pose2d conePose = new Pose2d( -tileSize / 2, -(tileSize * 3) + robotLength, Math.toRadians( 180 ) );


	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( x, y, Math.toRadians( 90 ) ) )
				.lineToConstantHeading( new Vector2d( -tileSize * 2, y ) )
				//drop cone
				.lineToLinearHeading( conePose )
				//grab cone
				.lineToLinearHeading( new Pose2d( -robotLength, 0, Math.toRadians( 0 ) ) )
				.lineToLinearHeading( conePose )
				.lineToLinearHeading( new Pose2d( -(tileSize * 2) + robotLength, 0, Math.toRadians( 0 ) ) )
				.build( );
	}

}