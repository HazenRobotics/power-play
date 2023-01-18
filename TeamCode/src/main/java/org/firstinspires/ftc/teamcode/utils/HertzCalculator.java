package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class HertzCalculator {

	OpMode target;
	long cycles = 0;
	long startTime;
	long timeRan;
	long hertz;
	public HertzCalculator( OpMode opMode ) {
		target = opMode;
	}

	public void runCalculator( long time ) {
		startTime = System.currentTimeMillis( );
		target.init( );
		while( time+startTime>System.currentTimeMillis() ) {
			target.loop( );
			cycles++;
			timeRan = System.currentTimeMillis( ) - startTime;
			hertz = cycles/(timeRan/1000);
			target.telemetry.addData( "HZ:",hertz );
			target.telemetry.addData( "Time elapsed",timeRan );
		}
	}
}