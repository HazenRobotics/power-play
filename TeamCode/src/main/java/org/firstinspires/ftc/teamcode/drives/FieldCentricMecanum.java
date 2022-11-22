package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class FieldCentricMecanum extends MecanumDrive{

	BNO055IMU imu;

	public FieldCentricMecanum( HardwareMap hardwareMap) {
		super( hardwareMap );
		setUpIMU( hardwareMap );
	}

	private void setUpIMU( HardwareMap hardwareMap ) {
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		imu.initialize(parameters);
	}

	public void drive( double drive, double strafe, double rotate ) {

		// You might have to play with the + or - depending on how your motors are installed

		double botHeading = -imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;


		double rotX = strafe * Math.cos(botHeading) - drive * Math.sin(botHeading);
		double rotY = strafe * Math.sin(botHeading) + drive * Math.cos(botHeading);

		double frontLeftPower = (rotY + rotX + rotate);
		double backLeftPower = (rotY - rotX + rotate);
		double frontRightPower = (rotY - rotX - rotate);
		double backRightPower = (rotY + rotX - rotate);

		setMotorPower( frontLeftPower, backLeftPower, frontRightPower, backRightPower );
	}
}
