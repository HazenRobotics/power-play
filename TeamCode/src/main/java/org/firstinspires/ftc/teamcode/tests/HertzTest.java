package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.teleop.MiniTeleOp;
import org.firstinspires.ftc.teamcode.utils.HertzCalculator;

public class HertzTest extends OpMode {
	HertzCalculator test = new HertzCalculator( new MiniTeleOp() );
	@Override
	public void init( ) {
		test.runCalculator( 100000 );
	}

	@Override
	public void loop( ) {

	}
}
