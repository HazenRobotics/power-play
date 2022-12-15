package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.InternalIMU;

@TeleOp(group = "Test")
//@Disabled
public class InternalIMUTest extends OpMode {

	MecanumDrive drive;
	InternalIMU gyro;

	@Override
	public void init( ) {
		drive = new MecanumDrive( hardwareMap );
		gyro = new InternalIMU( hardwareMap );
	}

	@Override
	public void loop( ) {
		drive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
		telemetry.addData("angles:", gyro.getOrientation());
	}
}
