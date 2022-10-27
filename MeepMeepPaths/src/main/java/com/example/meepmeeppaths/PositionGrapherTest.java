package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.example.meepmeeppaths.teamcodeRequirements.LifterBot;
import com.example.meepmeeppaths.teamcodeRequirements.PPField;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class PositionGrapherTest implements MeepMeepPath {

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return getFlipedTrajectorySequence( drive, 1, 1 );
	}

	@Override
	public TrajectorySequence getFlipedTrajectorySequence(DriveShim drive, double xFlip, double yFlip) {
		double cycledist = totalTitle*1.5;
		float x = -(float) ((2 * totalTitle - LifterBot.ROBOT_WIDTH / 2 + 3 * tileConnector / 2)*xFlip);
		float y = (float) (-(PPField.HALF_FIELD - LifterBot.ROBOT_LENGTH / 2)*xFlip);
		double  xFlipR = -((xFlip-1)/2);
		double yFlipR = -((xFlip-1)/2);
		Pose2d conePose = new Pose2d( (-totalTitle *2.5)*xFlip, (-totalTitle/2)*xFlip, Math.toRadians( 180+(180*xFlipR) ) );
		return drive.trajectorySequenceBuilder( new Pose2d( 0, 64, Math.toRadians( 270 ) ) )
				// Duck spin
				.lineTo( new Vector2d( 0, -16 ) )
				.build( );
	}

}