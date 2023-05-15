package org.firstinspires.ftc.teamcode.drives;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.SwervePod;

public class FourWheelSwerveDrive implements Drive {

	SwervePod frontLeftPod, backLeftPod, frontRightPod, backRightPod;

	/**
	 * @param hardwareMap       robot's hardware map
	 * @param motorNames        motor names in order of FL, BL, FR, BR
	 * @param motorReverse      true if motor reversed in order of FL, BL, FR, BR
	 * @param servoPowerNames   servo names in order of FL, BL, FR, BR
	 * @param servoEncoderNames **servo encoder names in order of FL, BL, FR, BR**
	 * @param servoReversed     true if servo reversed in order of FL, BL, FR, BR
	 */
	public FourWheelSwerveDrive( HardwareMap hardwareMap, String[] motorNames, boolean[] motorReverse, String[] servoPowerNames, String[] servoEncoderNames, boolean[] servoReversed ) {
		frontLeftPod = new SwervePod( hardwareMap, motorNames[0], motorReverse[0], servoPowerNames[0], servoReversed[0], new double[]{ 3, 3 }, 537.7 );
		backLeftPod = new SwervePod( hardwareMap, motorNames[1], motorReverse[1], servoPowerNames[1], servoReversed[0], new double[]{ 3, 3 }, 537.7 );
		frontRightPod = new SwervePod( hardwareMap, motorNames[2], motorReverse[2], servoPowerNames[2], servoReversed[0], new double[]{ 3, 3 }, 537.7 );
		backRightPod = new SwervePod( hardwareMap, motorNames[3], motorReverse[3], servoPowerNames[3], servoReversed[0], new double[]{ 3, 3 }, 537.7 );

	}

	public void move( double driveInput, double strafeInput, double rotateInput ) {
		move( new Vector2d( driveInput, strafeInput ), rotateInput );
	}


	public void move( Vector2d driveStick, double rotatePower ) {
		/*
		lerp between angles
		 */
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
