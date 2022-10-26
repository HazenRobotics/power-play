package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class RedSimpleCycle implements MeepMeepPath {
	public static
	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( 0, 64, Math.toRadians( 270 ) ) )
				// Duck spin

				.lineTo( new Vector2d( 0, -16 ) )
				.build( );
	}

}