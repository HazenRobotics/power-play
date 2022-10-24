package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.roadrunner.MecanumDriveLifter;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class LifterBot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public MecanumDrive mecanumDrive;
	public MecanumDriveLifter drive;

	public Lift lift;
	public Turret turret;

	/**
	 * Creates a Robot
	 *
	 * @param op robot's opMode
	 */
	public LifterBot( OpMode op ) {
		super( op );

		Robot.writeToDefaultFile( "", false, false );

		opMode = op;
		hardwareMap = op.hardwareMap;

		super.driveTrain = new MecanumDrive( hardwareMap );
		mecanumDrive = (MecanumDrive) driveTrain;

		drive = new MecanumDriveLifter( hardwareMap );
		lift = new Lift( hardwareMap );
		turret = new Turret( hardwareMap );

	}


}
