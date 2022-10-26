package com.example.meepmeeppaths.teamcodeRequirements;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Stores information about the field so that it can represent a digital "map" of the field.
 * All measurements are in mm so that it is easier to use with vuforia
 */
public class PPField {

	// All measurements are in mm
	private static final float MM_PER_INCH = 1f / 2.54f;
	private static final float INCH_PER_MM = 2.45f;

	// Field information
	public static final float TILE_SIZE = 22.75f;
	public static final float TILE_CONNECTOR = 0.75f;

	public static final float FIELD_SIZE = (TILE_SIZE * 6 + TILE_CONNECTOR * 5);
	//	private static final float MM_TARGET_HEIGHT = (6) * mmPerInch;
	public static final float HALF_FIELD = FIELD_SIZE / 2;
	public static final float QUAD_FIELD = FIELD_SIZE / 4;

	// Goals information
	public static float toMM( float n ) {
		return n * MM_PER_INCH;
	}

	public static double toMM( double n ) {
		return n * MM_PER_INCH;
	}

	public static Pose2d toMM( Pose2d n ) {
		return n.times( MM_PER_INCH );
	}

	public static Pose2d[] toMM( Pose2d... n ) {
		Pose2d[] temp = n.clone( );
		for( Pose2d pose : temp )
			pose.times( MM_PER_INCH );
		return temp;
	}

	/**
	 * a 5x5 character map of which junction is which
	 */
	public static final Junction[][] JUNCTION_MAP = {
			{ Junction.GROUND, Junction.LOW, Junction.GROUND, Junction.LOW, Junction.GROUND },
			{ Junction.LOW, Junction.MEDIUM, Junction.HIGH, Junction.MEDIUM, Junction.LOW },
			{ Junction.GROUND, Junction.HIGH, Junction.GROUND, Junction.HIGH, Junction.GROUND },
			{ Junction.LOW, Junction.MEDIUM, Junction.HIGH, Junction.MEDIUM, Junction.LOW },
			{ Junction.GROUND, Junction.LOW, Junction.GROUND, Junction.LOW, Junction.GROUND }
	};

	private static final float POLE_OUTER_RADIUS = 0.5f;
	private static final float GROUND_OUTER_RADIUS = 3f;
	private static final float GROUND_HEIGHT = 0.56f;
	private static final float LOW_HEIGHT = 13.5f;
	private static final float MEDIUM_HEIGHT = 23.5f;
	private static final float HIGH_HEIGHT = 33.5f;

	/**
	 * get the junction type at a junction's position
	 *
	 * @param x value of the junction, in the range [-2,2], using field positioning
	 * @param y value of the junction, in the range [-2,2], using field positioning
	 * @return the field value in inches
	 */
	public static Junction getJunction( int x, int y ) {
		return JUNCTION_MAP[x + 2][y + 2];
	}

	/**
	 * get the pose of a junction
	 *
	 * @param x value of the junction, in the range [-2,2], using field positioning
	 * @param y value of the junction, in the range [-2,2], using field positioning
	 * @return the field value in inches
	 */
	public static Pose2d getJunctionPose( int x, int y, boolean inMM ) {
		Pose2d temp = new Pose2d( (TILE_SIZE + TILE_CONNECTOR) * x, (TILE_SIZE + TILE_CONNECTOR) * y );
		return inMM ? toMM( temp ) : temp;
	}

	public enum Junction {

		GROUND( GROUND_HEIGHT, GROUND_OUTER_RADIUS ),
		LOW( LOW_HEIGHT, POLE_OUTER_RADIUS ),
		MEDIUM( MEDIUM_HEIGHT, POLE_OUTER_RADIUS ),
		HIGH( HIGH_HEIGHT, POLE_OUTER_RADIUS );

		private final float height;
		private final float radius;

		Junction( float height, float radius ) {
			this.height = height;
			this.radius = radius;
		}

		private float height( ) {
			return height;
		}

		private float radius( ) {
			return radius;
		}

		private Pose2d[] poses( boolean inMM ) {
			Pose2d[] poses = new Pose2d[this == Junction.GROUND ? 9 : (this == Junction.LOW ? 8 : 4)];

			int i = 0;
			for( int x = 2; x >= -2; x-- )
				for( int y = 2; y >= -2; y-- )
					if( JUNCTION_MAP[x][y] == this )
						poses[i++] = getJunctionPose( x, y, false );

			return inMM ? toMM( poses ) : poses;
		}
	}

	// 29 1/4" from the inside of the field to the edge of the paper
	// 1 7/8" from the bottom of the paper to the floor, outside of the field

}
