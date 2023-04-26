package com.example.meepmeeppaths.teamcodeRequirements;

import static com.example.meepmeeppaths.teamcodeRequirements.PPField.THREE_HALVES_TILE;
import static com.example.meepmeeppaths.teamcodeRequirements.PPField.TILE_CONNECTOR;
import static com.example.meepmeeppaths.teamcodeRequirements.PPField.TILE_SIZE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class MiniBot {

	public static final float ROBOT_LENGTH = 12.25f;
	public static final float ROBOT_MAX_LENGTH = 19.45f; // with claw
	public static final float ROBOT_WIDTH = 14.125f;
	public static final float ROBOT_MAX_WIDTH = 13.8125f;

	public static final float CONE_OFFSET = 12.0f;

	public enum LiftPosition {
		BOTTOM, JNCTN_GROUND, JNCTN_LOW, JNCTN_MEDIUM, JNCTN_HIGH
	}

	public enum SignalPosition {
		LEFT,
		MIDDLE,
		RIGHT,
		NOT_FOUND
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

	public static Vector2d getSignalPos( boolean red, boolean right ) {
		double x, y;
		if( red ) { // red side
			x = right ? THREE_HALVES_TILE : -THREE_HALVES_TILE;
			y = -THREE_HALVES_TILE;
		} else { // blue side
			x = right ? -THREE_HALVES_TILE : THREE_HALVES_TILE;
			y = THREE_HALVES_TILE;
		}
		return new Vector2d( x, y );
	}

	public static Vector2d justParkInit( boolean red, boolean right ) {

		double x, y;

		SignalPosition signalPosition = SignalPosition.LEFT;

		double tilePos = 0.05;
		if( signalPosition == SignalPosition.LEFT )
			tilePos = -1;
		else if( signalPosition == SignalPosition.RIGHT )
			tilePos = 1;

		if( red ) {
			x = right ? (THREE_HALVES_TILE + tilePos * TILE_SIZE) : -(THREE_HALVES_TILE - tilePos * TILE_SIZE);
			y = -TILE_SIZE / 2;
		} else {
			x = right ? -(THREE_HALVES_TILE + tilePos * TILE_SIZE) : (THREE_HALVES_TILE - tilePos * TILE_SIZE);
			y = TILE_SIZE / 2;
		}

		return new Vector2d( x, y );
	}


	/**
	 * @param angle            the heading to face the Junction (degrees)
	 * @param angleOffset      the starting angle of the robot but will do nothing
	 * @param junctionDistance the distance away from the shipping hub base to be
	 * @param junctionPos      the position of the junction to go to
	 * @return the position/heading (Pose2D) of where to go
	 */
	public static Pose2d getJunctionOffsetPos( double angle, double angleOffset, double junctionDistance, Vector2d junctionPos ) {
		double dist = (junctionDistance + ROBOT_LENGTH / 2);
		angle = Math.toRadians( angle );
		double x = junctionPos.getX( ) - Math.cos( angle ) * dist;
		double y = junctionPos.getY( ) - Math.sin( angle ) * dist;
		return new Pose2d( x, y, /*Math.toRadians( angleOffset ) + */angle );
	}

	public static double getAngleOnSide( boolean red, boolean right ) {

		if( red ) {
			if( right ) { // red right
				return 135;
			} else { // red left
				return 45;
			}
		} else {
			if( right ) { // blue right
				return 315;
			} else { // blue left
				return 225;
			}
		}
	}

	/**
	 * @param angle            the heading to face the Junction (degrees)
	 * @return the position/heading (Pose2D) of where to go
	 */
	public static Pose2d getJunctionOffsetPos( double angle, int junctionX, int junctionY ) {
		Vector2d junctionPos = PPField.getJunctionPose( junctionX, junctionY, false );
		angle = Math.toRadians( angle );
		double x = junctionPos.getX( ) - Math.cos( angle ) * ROBOT_MAX_LENGTH;
		double y = junctionPos.getY( ) - Math.sin( angle ) * ROBOT_MAX_LENGTH;
		return new Pose2d( x, y, angle );
	}

	public static Pose2d getJunctionOffsetPos( double angle, double distance, double heading, int junctionX, int junctionY ) {
		Vector2d junctionPos = PPField.getJunctionPose( junctionX, junctionY, false );
		double dist = (distance + ROBOT_LENGTH / 2);
		angle = Math.toRadians( angle );
		double x = junctionPos.getX( ) - Math.cos( angle ) * dist;
		double y = junctionPos.getY( ) - Math.sin( angle ) * dist;
		return new Pose2d( x, y, Math.toRadians( heading ) );
	}


	/**
	 * @param angle            the heading to face the Junction (degrees)
	 * @param junctionDistance the distance away from the shipping hub base to be
	 * @param junctionPos      the position of the junction to go to
	 * @return the position/heading (Pose2D) of where to go
	 */
	public static Pose2d getJunctionOffsetPos( double angle, double junctionDistance, Vector2d junctionPos ) {
		double dist = (junctionDistance + ROBOT_LENGTH / 2);
		angle = Math.toRadians( angle );
		double x = junctionPos.getX( ) - Math.cos( angle ) * dist;
		double y = junctionPos.getY( ) - Math.sin( angle ) * dist;
		return new Pose2d( x, y, angle );
	}

	/**
	 *
	 * @param red whether the bot is on the red side of the field
	 * @param right whether the bot is on the right side of the field from that side's perspective
	 * @return an int array of length 2 containing the sign (1 or -1) of the quadrant the robot must be in
	 */
	public static int[] getQuadrantSign( boolean red, boolean right ) {
		return new int[]{ (red ? 1 : -1) * ( right ? 1 : -1), (red ? -1 : 1) };
	}



}
