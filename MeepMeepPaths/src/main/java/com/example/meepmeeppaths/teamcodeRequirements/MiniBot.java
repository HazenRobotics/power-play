package com.example.meepmeeppaths.teamcodeRequirements;

import static com.example.meepmeeppaths.teamcodeRequirements.PPField.TILE_CONNECTOR;
import static com.example.meepmeeppaths.teamcodeRequirements.PPField.TILE_SIZE;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MiniBot {

	public static final float ROBOT_LENGTH = 12.25f;
	public static final float ROBOT_MAX_LENGTH = 15.75f; // with claw
	public static final float ROBOT_WIDTH = 14.125f;
	public static final float ROBOT_MAX_WIDTH = 14.5f;

	public enum LiftPosition {
		BOTTOM, JNCTN_GROUND, JNCTN_LOW, JNCTN_MEDIUM, JNCTN_HIGH
	}


	/**
	 * @param red   true if on the red side, false for blue
	 * @param right true if on the right side from the perspective of the driver
	 * @return the starting position of the robot in these circumstances
	 */
	public static Pose2d getStartPos( boolean red, boolean right ) {
		double x, y, heading;
		if( red ) { // red side
			x = right ? (TILE_SIZE + ROBOT_MAX_WIDTH / 2) : -(2 * TILE_SIZE - ROBOT_MAX_WIDTH / 2);
			y = -(3 * (TILE_CONNECTOR + TILE_SIZE) - ROBOT_LENGTH / 2);
			heading = Math.toRadians( 90 );
		} else { // blue side
			x = right ? -(TILE_SIZE + ROBOT_MAX_WIDTH / 2) : (2 * TILE_SIZE - ROBOT_MAX_WIDTH / 2);
			y = (3 * (TILE_CONNECTOR + TILE_SIZE) - ROBOT_LENGTH / 2);
			heading = Math.toRadians( 270 );
		}
		return new Pose2d( x, y, heading );
	}


}
