package org.firstinspires.ftc.teamcode.preSeason;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LearnRunner extends LinearOpMode {
	DcMotorEx frontLeft, backLeft, frontRight, backRight;

	@Override
	public void runOpMode( ) {

		frontLeft = hardwareMap.get( DcMotorEx.class, "frontLeft" );
		backLeft = hardwareMap.get( DcMotorEx.class, "backLeft" );
		frontRight = hardwareMap.get( DcMotorEx.class, "frontRight" );
		backRight = hardwareMap.get( DcMotorEx.class, "backRight" );
		frontRight.setDirection( DcMotorSimple.Direction.REVERSE );
		backRight.setDirection( DcMotorSimple.Direction.REVERSE );

		MecanumDrive drive = new MecanumDrive(hardwareMap);

		Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
				.strafeRight(10)
				.forward(5)
				.build();

		waitForStart();

		if(isStopRequested()) return;

		drive.followTrajectory(myTrajectory);
		}
	}
}


