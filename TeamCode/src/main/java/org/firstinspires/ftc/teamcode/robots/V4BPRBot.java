package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.FieldCentricMecanumDrive;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.VirtualFourBar;

public class V4BPRBot extends Robot {

	public OpMode opMode;
	public HardwareMap hw;
	public MecanumDrive drive;
	public FieldCentricMecanumDrive fieldCentricMecanumDrive;
	public VirtualFourBar arm;


	public V4BPRBot( OpMode op ) {
		super( op );

		opMode = op;
		hw = op.hardwareMap;

		drive = new MecanumDrive( hw, "fl", "bl", "fr", "br" );
	}

}
