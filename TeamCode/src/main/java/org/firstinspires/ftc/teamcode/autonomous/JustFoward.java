package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.MiniBot;

@Autonomous(group = "tE")

public class JustFoward extends LinearOpMode {
	MiniBot robot;
	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new MiniBot( this );
		robot.drive.followTrajectorySequence( robot.drive.trajectorySequenceBuilder( MiniBot.getStartPos( true, true ) )
				.forward( 12 )
				.build( ) );
	}
}