package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.example.meepmeeppaths.teamcodeRequirements.MiniBot;
import com.example.meepmeeppaths.teamcodeRequirements.PPField;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class BlockOff implements MeepMeepPath {


	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return getFlippedTrajectorySequence( drive, -1, 1 );
	}

	@Override
	public TrajectorySequence getFlippedTrajectorySequence( DriveShim drive, double xFlip, double yFlip ) {
		double cycleDist = totalTitle * 1.5;
		float x = -(float) ((2 * totalTitle - MiniBot.ROBOT_WIDTH / 2 + 3 * tileConnector / 2) * xFlip);
		float y = (float) (-(PPField.HALF_FIELD - MiniBot.ROBOT_LENGTH / 2) * yFlip);
		double xFlipR = -((xFlip - 1) / 2);
		double yFlipR = -((yFlip - 1) / 2);
//		SignalUtil detector = new SignalUtil( hardwareMap, "webcam1", telemetry );
//		detector.init( );
//		cone = detector.getParkPosition( xflip, yflip );
		Vector2d cone = new Vector2d( -37 * xFlip, -11 * yFlip );
		Pose2d conePose = new Pose2d( -60 * xFlip, -11 * yFlip, Math.toRadians( 180 + (180 * xFlipR) ) );
		return drive.trajectorySequenceBuilder( new Pose2d( x, y, Math.toRadians( 90 + (180 * yFlipR) ) ) )
				.lineToConstantHeading( new Vector2d( -60 * xFlip, y ) )
				//drop cone
				.lineToLinearHeading( conePose )
				//grab cone
				.lineToConstantHeading( new Vector2d( 0, -11 * yFlip ) )
				.lineToLinearHeading( conePose )
				//grab cone
				.lineToConstantHeading( new Vector2d( -totalTitle * 2 * xFlip, -11 * yFlip ) )
				.lineTo( cone )
				.build( );
	}

}