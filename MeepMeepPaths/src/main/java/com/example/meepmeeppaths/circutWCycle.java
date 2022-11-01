package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.example.meepmeeppaths.teamcodeRequirements.LifterBot;
import com.example.meepmeeppaths.teamcodeRequirements.PPField;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class circutWCycle implements MeepMeepPath {

	float x = (float) (2 * totalTitle - LifterBot.ROBOT_WIDTH / 2 + 3 * tileConnector / 2);
	float y = PPField.HALF_FIELD - LifterBot.ROBOT_LENGTH / 2;
	Pose2d conePose = new Pose2d( -totalTitle *2.5, -totalTitle/2, Math.toRadians( 0 ) );
	double cycledist = totalTitle*1.5;


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
		//SignalUtil detector = new SignalUtil( hardwareMap, "webcam1", telemetry );
//		detector.init();
//		cone = detector.getParkPosition(xflip,yflip);
		Vector2d cone = new Vector2d(-37*xFlip,-11*yFlip  );
		Pose2d conePose = new Pose2d( -60*xFlip,-11*yFlip, Math.toRadians( 180+(180*xFlipR) ) );
		return drive.trajectorySequenceBuilder( new Pose2d( x, y, Math.toRadians( 90 ) ) )
				.lineToConstantHeading( new Vector2d( -50*xFlip, -60*yFlip ) )
				//drop cone
				.lineToLinearHeading( conePose )

				//cycle1
				.back( cycledist )
				//drop cone
				.forward( cycledist )
				//grab cone

				//cycle2
				.back( cycledist )
				//drop cone
				.forward( cycledist )
				//grab cone

				//cycle3
				.back( cycledist )
				//drop cone
				.forward( cycledist )
				//drop cone
				.lineTo( cone )
				.build( );
	}

}