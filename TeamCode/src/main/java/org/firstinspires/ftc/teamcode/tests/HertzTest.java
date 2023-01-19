package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class HertzTest extends LinearOpMode {
	OpMode target;
	long cycles = 0;
	long timeRan = 0;
	long hertz = 0;

	@Override
	public void runOpMode( ) throws InterruptedException {
		target.init( );
		long startTime = System.currentTimeMillis( );
		while( this.opModeIsActive() ) {
			target.loop( );
			cycles++;
			timeRan = System.currentTimeMillis( ) - startTime;
			hertz = cycles/(timeRan/1000);
			target.telemetry.addData( "HZ:",hertz );
			target.telemetry.addData( "Time eplased",timeRan );
		}
	}
}
