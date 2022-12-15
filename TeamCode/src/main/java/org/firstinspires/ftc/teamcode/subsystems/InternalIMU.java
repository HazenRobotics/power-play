package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class InternalIMU {

	public BNO055IMU gyro;

	public InternalIMU( HardwareMap hw, BNO055IMU.AngleUnit unit ) {
		gyro = hw.get( BNO055IMU.class, "imu" );

		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters( );
		parameters.angleUnit = unit;
		gyro.initialize( parameters );
	}

	public InternalIMU( HardwareMap hw ) {
		this(hw, BNO055IMU.AngleUnit.DEGREES);
	}

	public BNO055IMU.AngleUnit getUnit() {
		return gyro.getParameters().angleUnit;
	}

	public Orientation getOrientation() {
		return gyro.getAngularOrientation();
	}

}
