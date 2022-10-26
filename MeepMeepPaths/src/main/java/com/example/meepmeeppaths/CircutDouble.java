package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.example.meepmeeppaths.teamcodeRequirements.LifterBot;
import com.example.meepmeeppaths.teamcodeRequirements.PPField;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class CircutDouble implements MeepMeepPath {
	float x = (float) (2 * tileSize - LifterBot.ROBOT_WIDTH / 2 + 3 * tileConnector / 2);
	float y = PPField.HALF_FIELD - LifterBot.ROBOT_LENGTH / 2;
	Pose2d conePose = new Pose2d( -tileSize *2.5, -tileSize/2, Math.toRadians( 0 ) );
	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( x, y, Math.toRadians( 90 ) ) )
				.lineToConstantHeading( new Vector2d( x-(tileSize/2), y ) )
				//drop cone
				.lineToConstantHeading( new Vector2d( x-tileSize,y ) )
				.lineToLinearHeading( conePose )
				//grab cone
				.turn( Math.toRadians( 0 ) )
				.forward( tileSize*3 )
				.turn( Math.toRadians( 270 ) )
				.forward( tileSize*2 )
				.turn( Math.toRadians( 0 ) )
				.forward( tileSize*1.5 )
				.turn( Math.toRadians( 90 ) )
				//drop cone
				.build( );
	}
}
