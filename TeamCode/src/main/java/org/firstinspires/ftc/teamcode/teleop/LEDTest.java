package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.RGBLights;

@Autonomous
@Disabled
public class LEDTest extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {
		RGBLights lights = new RGBLights( hardwareMap, "blinkin" );
		lights.showStatus( RGBLights.StatusLights.NORMAL );
		waitForStart();
		lights.showStatus( RGBLights.StatusLights.CELEBRATION );
		while( opModeIsActive() );

	}
}
