package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class FieldCentricMecanumDrive extends MecanumDrive{

	BNO055IMU imu;

	/**
	 * Creates a MecanumDrive with an IMU added on
	 * @param hardwareMap
	 */
	public FieldCentricMecanumDrive( HardwareMap hardwareMap) {
		super( hardwareMap );
		setUpIMU( hardwareMap );
	}

	/**
	 * Creates a FieldCentricMecanumDrive with specified names for the wheels
	 *
	 * @param hardwareMap         robot's hardware map
	 * @param frontLeftMotorName  name of front left motor in the hardware map
	 * @param backLeftMotorName   name of back left motor in the hardware map
	 * @param frontRightMotorName name of front right motor in the hardware map
	 * @param backRightMotorName  name of back right motor in the hardware map
	 */
	public FieldCentricMecanumDrive( HardwareMap hardwareMap, String frontLeftMotorName, String backLeftMotorName, String frontRightMotorName, String backRightMotorName ) {
		super( hardwareMap, frontLeftMotorName, backLeftMotorName, frontRightMotorName, backRightMotorName );
		setUpIMU( hardwareMap );
	}

	/**
	 * Initializes the IMU
	 * @param hardwareMap
	 */
	private void setUpIMU( HardwareMap hardwareMap ) {
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		imu.initialize(parameters);
	}

	/**
	 * Field centric drive method
	 * @param drive  power for forward and back motion
	 * @param strafe power for left and right robot
	 * @param rotate power for rotating the robot
	 */
	public void drive( double drive, double strafe, double rotate ) {

		// You might have to play with the + or - depending on how your motors are installed

		double botHeading = -imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;


		double rotX = (strafe * 1.1) * Math.cos(botHeading) - drive * Math.sin(botHeading);
		double rotY = (strafe * 1.1) * Math.sin(botHeading) + drive * Math.cos(botHeading);

		double denom = Math.max(Math.abs(drive) + Math.abs(strafe * 1.1) + Math.abs(rotate), 1);

		double frontLeftPower = (rotY + rotX + rotate) / denom;
		double backLeftPower = (rotY - rotX + rotate) / denom;
		double frontRightPower = (rotY - rotX - rotate) / denom;
		double backRightPower = (rotY + rotX - rotate) / denom;

		setMotorPower( frontLeftPower, backLeftPower, frontRightPower, backRightPower );
	}
}
