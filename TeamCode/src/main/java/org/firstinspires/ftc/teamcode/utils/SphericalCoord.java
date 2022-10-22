package org.firstinspires.ftc.teamcode.utils;

public class SphericalCoord extends Coordinate {

	public SphericalCoord( double r, double theta, double phi ) {
		super( r,theta,phi);
		super.dimensionLabel = new String[] {
				"R","Theta","Phi"
		};
	}

	@Override
	public CartesianCoord toCartesian( ) {
		double x = getC1()*Math.cos( getC2() );
		double y = getC1()*Math.sin(getC2());
		double z = getC3();
		return new CartesianCoord( x,y,z );
	}

	@Override
	public SphericalCoord toSpherical( ) {
		return this;
	}

	@Override
	public CylindricalCoord toCylindrical( ) {
		double p = getC1()*Math.sin( getC2() );
		double theta = getC2();
		double z = getC1()*Math.cos( getC3() );
		return new CylindricalCoord( p,theta,z );
	}
//R,Theta,phi
}
