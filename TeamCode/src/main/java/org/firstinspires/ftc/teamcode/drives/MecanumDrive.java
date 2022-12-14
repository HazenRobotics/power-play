package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class sets up and holds methods for running a mecanum drive
 */
public class MecanumDrive extends FourWheelDrive {

	/**
	 * Creates a MecanumDrive with default names for the wheels
	 *
	 * @param hardwareMap robot's hardware map
	 */
	public MecanumDrive( HardwareMap hardwareMap ) {
		super( hardwareMap );
	}

	/**
	 * Creates a MecanumDrive with specified names for the wheels
	 *
	 * @param hardwareMap         robot's hardware map
	 * @param frontLeftMotorName  name of front left motor in the hardware map
	 * @param backLeftMotorName   name of back left motor in the hardware map
	 * @param frontRightMotorName name of front right motor in the hardware map
	 * @param backRightMotorName  name of back right motor in the hardware map
	 */
	public MecanumDrive( HardwareMap hardwareMap, String frontLeftMotorName, String backLeftMotorName, String frontRightMotorName, String backRightMotorName ) {
		super( hardwareMap, frontLeftMotorName, backLeftMotorName, frontRightMotorName, backRightMotorName );
	}

	public int convertDistTicks( double distanceToTravel ) {
		return Drive.convertDistTicks( distanceToTravel,  Math.PI * wheelDiameter, pulsesPerRevolution, gearRatio );
	}

	public double convertTicksDist( int ticksToTravel ) {
		return Drive.convertTicksDist( ticksToTravel,  Math.PI * wheelDiameter, pulsesPerRevolution, gearRatio );
	}

	/**
	 * Sets specified power to the motors
	 *
	 * @param frontLeftPower  power at which to run the front left motor.
	 * @param backLeftPower   power at which to run the back left motor.
	 * @param frontRightPower power at which to run the front right motor.
	 * @param backRightPower  power at which to run the back right motor.
	 */
	public void setMotorPower( double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower ) {
		super.setMotorPower( frontLeftPower, backLeftPower, frontRightPower, backRightPower );
	}

	/**
	 * Sets power to the wheel motors
	 *
	 * @param drive  power for forward and back motion
	 * @param strafe power for left and right robot
	 * @param rotate power for rotating the robot
	 */
	public void drive( double drive, double strafe, double rotate ) {

		// You might have to play with the + or - depending on how your motors are installed
		double frontLeftPower = drive + strafe + rotate;
		double backLeftPower = drive - strafe + rotate;
		double frontRightPower = drive - strafe - rotate;
		double backRightPower = drive + strafe - rotate;

		setMotorPower( frontLeftPower, backLeftPower, frontRightPower, backRightPower );
	}

	/**
	 * Sets power to the wheel motors to drive in straight line left or right
	 *
	 * @param power  power for sideways motion
	 */
	public void strafe( double power ) {
		setMotorPower( power, -power, -power, power );
	}

	/**
	 * Sets the target position of each motor
	 *
	 * @param frontLeftPos target position of frontLeft motor
	 * @param backLeftPos target position of backLeft motor
	 * @param frontRightPos target position of frontRight motor
	 * @param backRightPos target position of backRight motor
	 */
	public void setMotorTargetPositions( int frontLeftPos, int backLeftPos, int frontRightPos, int backRightPos ) {
		frontLeft.setTargetPosition( frontLeftPos );
		backLeft.setTargetPosition( backLeftPos );
		frontRight.setTargetPosition( frontRightPos );
		backRight.setTargetPosition( backRightPos );
	}

	/**
	 * Sets the target position of each motor to the same value
	 *
	 * @param position position to run motors to
	 */
	public void setTargetPosition( int position ) {
		setMotorTargetPositions( position, position, position, position );
	}

}
