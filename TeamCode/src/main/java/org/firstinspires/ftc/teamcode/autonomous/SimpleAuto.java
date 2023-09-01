package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.MiniBotSimple;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.vision.AprilTagsUtil;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;

@Autonomous(name = "SimpleAuto", group = "Autonomous")

public class SimpleAuto extends LinearOpMode {

	MiniBotSimple robot;

	@Override
	public void runOpMode() {
		robot = new MiniBotSimple(hardwareMap);



	}



}


