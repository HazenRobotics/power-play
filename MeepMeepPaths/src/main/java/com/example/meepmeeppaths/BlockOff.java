package com.example.meepmeeppaths;

import com.example.meepmeeppaths.teamcodeRequirements.PPField;
import com.example.meepmeeppaths.teamcodeRequirements.LifterBot;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class BlockOff implements MeepMeepPath {

	float x = -(float) (2 * totalTitle - LifterBot.ROBOT_WIDTH / 2 + 3 * tileConnector / 2);
	float y = -(PPField.HALF_FIELD - LifterBot.ROBOT_LENGTH / 2);
	Pose2d conePose = new Pose2d( -totalTitle *2.5, -totalTitle/2, Math.toRadians( 180 ) );


	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( x, y, Math.toRadians( 90 ) ) )
				.lineToConstantHeading( new Vector2d( x-(totalTitle/2), y ) )
				//drop cone
				.lineToConstantHeading( new Vector2d( x-totalTitle,y ) )
				.lineToLinearHeading( conePose )
				//grab cone
				.lineToLinearHeading( new Pose2d( 0, -totalTitle/2, Math.toRadians( 90 ) ) )
				.lineToLinearHeading( conePose )
				.lineToLinearHeading( new Pose2d( -totalTitle*2, -robotLength, Math.toRadians( 90 ) ) )
				.build( );
	}

}