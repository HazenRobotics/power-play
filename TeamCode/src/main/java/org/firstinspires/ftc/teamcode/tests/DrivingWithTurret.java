package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(group = "Test")
public class DrivingWithTurret extends OpMode {

	MecanumDrive drive;
	Turret turret;

	@Override
	public void init( ) {
		drive = new MecanumDrive( hardwareMap );
		turret = new Turret( hardwareMap, "turr", true, AngleUnit.DEGREES, 384.5, 170.0 / 30.0, 0, 360 );
	}

	@Override
	public void loop( ) {
		drive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

	}
}
