package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.example.meepmeeppaths.teamcodeRequirements.LifterBot;
import com.example.meepmeeppaths.teamcodeRequirements.PPField;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class CircutSetup implements MeepMeepPath {

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return getFlipedTrajectorySequence( drive, 1, 1 );
	}

	@Override
	public TrajectorySequence getFlipedTrajectorySequence( DriveShim drive, double xFlip, double yFlip ) {
		double cycleDist = totalTitle * 1.5;
		float x = -(float) ((2 * totalTitle - LifterBot.ROBOT_WIDTH / 2 + 3 * tileConnector / 2) * xFlip);
		float y = (float) (-(PPField.HALF_FIELD - LifterBot.ROBOT_LENGTH / 2) * xFlip);
		double xFlipR = -((xFlip - 1) / 2);
		double yFlipR = -((xFlip - 1) / 2);
		Pose2d conePose = new Pose2d( (-totalTitle * 2.5) * xFlip, (-totalTitle / 2) * xFlip, Math.toRadians( 180 + (180 * xFlipR) ) );
		return drive.trajectorySequenceBuilder( new Pose2d( x, y, Math.toRadians( 90 ) ) )
				.lineToConstantHeading( new Vector2d( x - (totalTitle / 2), y ) )

				.lineToConstantHeading( new Vector2d( x - totalTitle, y ) )
				.lineToLinearHeading( conePose )
				//grab cone
				.back( cycleDist )
				//drop cone
				.forward( cycleDist )
				//grab cone
				.back( totalTitle * 2 )
				//drop cone
				.build( );
	}
}
