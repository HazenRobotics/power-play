package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class BlankOpMode extends OpMode {

	double loopTime = 0;

	@Override
	public void init( ) {
		telemetry.addLine("ready");
	}

	@Override
	public void loop( ) {
		double loop = System.nanoTime();
		telemetry.addData("hz ", 1000000000 / (loop - loopTime));
		loopTime = loop;
		telemetry.update();
	}
}
