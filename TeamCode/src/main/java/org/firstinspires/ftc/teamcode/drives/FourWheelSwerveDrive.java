package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.SwervePod;

public class FourWheelSwerveDrive implements Drive {

	SwervePod[] swervePods = new SwervePod[4];
	double wheelbase;
	double trackwidth;

	/**
	 * @param hardwareMap       robot's hardware map
	 * @param motorNames        motor names in order of FL, BL, FR, BR
	 * @param motorReverse      true if motor reversed in order of FL, BL, FR, BR
	 * @param servoPowerNames   servo names in order of FL, BL, FR, BR
	 * @param servoEncoderNames **servo encoder names in order of FL, BL, FR, BR**
	 * @param servoReversed     true if servo reversed in order of FL, BL, FR, BR
	 */
	public FourWheelSwerveDrive( HardwareMap hardwareMap, String[] motorNames, boolean[] motorReverse, String[] servoPowerNames, String[] servoEncoderNames, boolean[] servoReversed, double wheelbase, double trackwidth ) {
		for( int i = 0; i < swervePods.length; i++ )
			swervePods[i] = new SwervePod( hardwareMap, motorNames[i], motorReverse[i], servoPowerNames[i], servoReversed[0], new double[]{ 0, 0, 0 }, 537.7 );

		this.wheelbase = wheelbase;
		this.trackwidth = trackwidth;
	}

	/**
	 * field oriented swerve drive
	 *
	 * @param drivePower  power to move in the y direction (relative to the field)
	 * @param strafePower power to move in the x direction (relative to the field)
	 * @param rotatePower power to turn the robot (right)
	 * @param heading     robot's heading (RAD)
	 */
	public void move( double drivePower, double strafePower, double rotatePower, double heading ) {
		double temp = drivePower * Math.cos( heading ) + strafePower * Math.sin( heading );
		strafePower = -drivePower * Math.sin( heading ) + strafePower * Math.cos( heading );
		drivePower = temp;

		// distance between opposite wheels
		double R = Math.sqrt( wheelbase * wheelbase + trackwidth * trackwidth );

		double A = strafePower - rotatePower * (wheelbase / R);
		double B = strafePower + rotatePower * (wheelbase / R);
		double C = drivePower - rotatePower * (trackwidth / R);
		double D = drivePower + rotatePower * (trackwidth / R);

		double ws1 = Math.sqrt( B * B + C * C );
		double wa1 = Math.atan2( B, C );
		double ws2 = Math.sqrt( B * B + D * D );
		double wa2 = Math.atan2( B, D );
		double ws3 = Math.sqrt( A * A + D * D );
		double wa3 = Math.atan2( A, D );
		double ws4 = Math.sqrt( A * A + C * C );
		double wa4 = Math.atan2( A, C );

		double max = Math.max( Math.max( ws1, ws2 ), Math.max( ws3, ws4 ) );

		if( max > 1 ) {
			ws1 /= max;
			ws2 /= max;
			ws3 /= max;
			ws4 /= max;
		}

		double[] wheelSpeeds = new double[]{ ws1, ws2, ws3, ws4 };
		double[] wheelAngles = new double[]{ wa1, wa2, wa3, wa4 };

		for( int i = 0; i < swervePods.length; i++ ) {
			// if angle to turn to is greater than 180, negate power and reduce angle by 180
			double distanceToTurn = wheelAngles[i] - swervePods[i].getPodAngle( );
			if( distanceToTurn > Math.PI ) {
				wheelAngles[i] -= Math.PI;
				wheelSpeeds[i] *= -1;
			}

			swervePods[i].setWheelPower( wheelSpeeds[i] );
			swervePods[i].setPodAngleTarget( wheelAngles[i] );
			swervePods[i].updateRotatePD();
		}


	}

	@Override
	public void move( double power ) {

	}

	@Override
	public void turn( double power ) {

	}

	@Override
	public void stop( ) {

	}

	@Override
	public void drive( double move, double turn ) {

	}

	@Override
	public State getState( ) {
		return null;
	}
}
