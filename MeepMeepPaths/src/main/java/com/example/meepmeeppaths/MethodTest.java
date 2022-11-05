package com.example.meepmeeppaths;

import static com.example.meepmeeppaths.teamcodeRequirements.LifterBot.ROBOT_LENGTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.example.meepmeeppaths.teamcodeRequirements.LifterBot;
import com.example.meepmeeppaths.teamcodeRequirements.PPField;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MethodTest implements MeepMeepPath {

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return getFlippedTrajectorySequence( drive, 1, 1 );
	}

	@Override
	public TrajectorySequence getFlippedTrajectorySequence( DriveShim drive, double xFlip, double yFlip ) {
		double cycleDist = totalTitle * 1.5;
		float x = -(float) ((2 * totalTitle - LifterBot.ROBOT_WIDTH / 2 + 3 * PPField.TILE_CONNECTOR / 2) * xFlip);
		float y = (float) (-(PPField.HALF_FIELD - ROBOT_LENGTH / 2) * xFlip);
		double xFlipR = -((xFlip - 1) / 2);
		double yFlipR = -((xFlip - 1) / 2);
		Pose2d conePose = new Pose2d( (-totalTitle * 2.5) * xFlip, (-totalTitle / 2) * xFlip, Math.toRadians( 180 + (180 * xFlipR) ) );

		System.out.println( PPField.getJunctionPose( 0, -1, false ) );

		return drive.trajectorySequenceBuilder( getHubPosition( 0, 0, PPField.TILE_SIZE / 2, PPField.getJunctionPose( 0, -1, false ) ) )
				// Duck spin
				.forward( 0.1 )
				.build( );
	}

	/**
	 * @param angle            the heading to face the Junction (degrees)
	 * @param angleOffset      the starting angle of the robots
	 * @param junctionDistance the distance away from the shipping hub base to be
	 * @param junctionPos      the position of the junction to go to
	 * @return the position/heading (Pose2D) of where to go
	 */
	public Pose2d getHubPosition( double angle, double angleOffset, double junctionDistance, Vector2d junctionPos ) {
		double dist = (junctionDistance + ROBOT_LENGTH / 2);
		angle = Math.toRadians( angle );
		double x = junctionPos.getX( ) - Math.cos( angle ) * dist;
		double y = junctionPos.getY( ) - Math.sin( angle ) * dist;
		return new Pose2d( x, y, Math.toRadians( angleOffset ) + angle );
	}

}