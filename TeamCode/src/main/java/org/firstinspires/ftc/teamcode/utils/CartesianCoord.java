package org.firstinspires.ftc.teamcode.utils;

public class CartesianCoord extends Coordinate {
	public CartesianCoord( double x, double y, double z ) {
		super( x, y, z );
		super.dimensionLabel= new String[] {
				"X","Y","Z"
		};
	}

	@Override
	public CartesianCoord toCartesian( ) {
		return this;
	}

	@Override
	public SphericalCoord toSpherical( ) {
		double r = Math.sqrt( Math.pow( this.getC1(),2 )+Math.pow( this.getC2(),2 )+Math.pow( this.getC3(),2 ) );
		double theta = Math.atan( this.getC2()/this.getC1() );
		double phi =  Math.atan(Math.sqrt( Math.pow( this.getC1(),2 )+Math.pow( this.getC2(),2 ))/Math.pow( this.getC3(),2 ));
		return new SphericalCoord(r,theta,phi);
	}

	@Override
	public CylindricalCoord toCylindrical( ) {
		double r = Math.sqrt( Math.pow( this.getC1(),2 )+Math.pow( this.getC2(),2 ));
		double theta = Math.atan( this.getC2()/this.getC1() );
		double h = this.getC3();
		return new CylindricalCoord( r,theta,h);
	}
}
