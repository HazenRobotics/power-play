package com.example.meepmeeppaths;


import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public interface MeepMeepPath {

	double robotLength = 13.25;
	double robotWidth = 13.5; // with belts

	double tileSize = 23;
	double tileConnector = 0.75;
	double totalTitle = tileConnector + tileSize;
	double hubRadius = 9;

	double cameraRightIndent = 1.25;

	TrajectorySequence getTrajectorySequence( DriveShim drive );

	TrajectorySequence getFlippedTrajectorySequence( DriveShim drive, double xFlip, double yFlip );


}
