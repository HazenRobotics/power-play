package org.firstinspires.ftc.teamcode.utils;

public class CylindricalCoord extends Coordinate {

	public CylindricalCoord( double r, double theta, double h ) {
		super( r, theta, h );
		super.dimensionLabel = new String[] {
				"R","Theta","H"
		};
	}

	@Override
	public CartesianCoord toCartesian( ) {
		double x = getC1( ) * Math.cos( getC2( ) );
		double y = getC1( ) * Math.sin( getC2( ) );
		double z = getC3( );
		return new CartesianCoord( x, y, z );
	}

	@Override
	public SphericalCoord toSpherical( ) {
		double r = Math.sqrt( Math.pow( getC1( ), 2 ) + Math.pow( getC3( ), 2 ));
		double phi = Math.atan( getC1( ) / getC3( ) );
		double theta = getC2( );
		return new SphericalCoord( r,phi,theta );
	}

	@Override
	public CylindricalCoord toCylindrical( ) {
		return this;
	}
}
