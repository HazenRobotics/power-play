package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

@Autonomous(group = "test")

public class RampUpMotors extends LinearOpMode {

	MecanumDrive drive;

	@Override
	public void runOpMode( ) throws InterruptedException {
		drive = new MecanumDrive( hardwareMap );

		for( int i = 0; i < 20; i++ ) {
			drive.drive( i * 0.05,0 );
			double startTime = System.currentTimeMillis();
			while( startTime + 5000 > System.currentTimeMillis() );
			telemetry.addData( "loop", i );
			telemetry.update();
		}
	}

}
