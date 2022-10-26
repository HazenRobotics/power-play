package org.firstinspires.ftc.teamcode.utils.coordinates;

abstract class Coordinate {
	double[] coordinates;
	String[] dimensionLabel;


	public Coordinate( double coord1, double coord2, double coord3 ) {
		coordinates = new double[] {
				coord1,coord2,coord3
		};
		dimensionLabel = new String[]{
				"Coord1", "Coord2", "Coord3"
		};
	}

	public double[] getCoordinates( ) {
		return new double[] {
		};
	}
	public double distance( Coordinate other ) {
		other=other.toCartesian();
		CartesianCoord cartesian= this.toCartesian();
		double disX = Math.pow(cartesian.getC1()-other.getC1(),2);
		double disY = Math.pow(cartesian.getC2()-other.getC2(),2);
		double disZ = Math.pow(cartesian.getC3()-other.getC3(),2);

		return Math.sqrt( disX+disY+disZ );
	}
	public abstract CartesianCoord toCartesian();
	public abstract SphericalCoord toSpherical();
	public abstract CylindricalCoord toCylindrical();

	public double getC1() {
		return coordinates[0];
	}
	public  double getC2() {
		return coordinates[1];
	}
	public double getC3() {
		return coordinates[2];
	}
	public String toString() {
		return "(" + dimensionLabel[0] + ": "+getC1()
				+ ", " + dimensionLabel[1] + ": " + getC2()
				+ ", " + dimensionLabel[2] + ": " + getC3 () + ")";
	}
}
