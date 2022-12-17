package org.firstinspires.ftc.teamcode.utils.localization;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Stores information about the field so that it can represent a digital "map" of the field.
 * All measurements are in mm so that it is easier to use with vuforia
 */
public class PPField {

	// All measurements are in mm
	private static final float MM_PER_INCH = (float) DistanceUnit.mmPerInch;
	private static final float INCH_PER_MM = (float) DistanceUnit.MM.toInches( 1 );

	// Field information
	public static final float TILE_SIZE = 22.75f;
	public static final float TILE_CONNECTOR = 0.75f;

	public static final float THREE_HALVES_TILE = 3f / 2f * TILE_SIZE;

	public static final float FIELD_SIZE = (TILE_SIZE * 6 + TILE_CONNECTOR * 5); // 140.25"
	//	private static final float MM_TARGET_HEIGHT = (6) * mmPerInch;
	public static final float HALF_FIELD = FIELD_SIZE / 2;
	public static final float QUAD_FIELD = FIELD_SIZE / 4;

	public static final float CONE_STACK_OFFSET = 12f;
	public static final float CONE_RADIUS = 2f;
	public static final float CONE_HEIGHT = 5f;
	public static final float CONE_BASE = 0.75f;

	// Goals information
	public static float toMM( float n ) {
		return n * MM_PER_INCH;
	}

	public static double toMM( double n ) {
		return n * MM_PER_INCH;
	}

	public static Vector2d toMM( Vector2d n ) {
		return n.times( MM_PER_INCH );
	}

	public static Vector2d[] toMM( Vector2d... n ) {
		Vector2d[] temp = n.clone( );
		for( Vector2d pose : temp )
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
	public static Vector2d getJunctionPose( int x, int y, boolean inMM ) {
		Vector2d temp = new Vector2d( (TILE_SIZE + TILE_CONNECTOR) * x, (TILE_SIZE + TILE_CONNECTOR) * y );
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

		public float height( ) {
			return height;
		}

		public float radius( ) {
			return radius;
		}

		private Vector2d[] poses( boolean inMM ) {
			Vector2d[] poses = new Vector2d[this == Junction.GROUND ? 9 : (this == Junction.LOW ? 8 : 4)];

			int i = 0;
			for( int x = 2; x >= -2; x-- )
				for( int y = 2; y >= -2; y-- )
					if( JUNCTION_MAP[x][y] == this )
						poses[i++] = getJunctionPose( x, y, false );

			return inMM ? toMM( poses ) : poses;
		}
	}

}
