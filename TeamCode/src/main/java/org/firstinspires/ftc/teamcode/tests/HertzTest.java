package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.MiniTeleOp;
import org.firstinspires.ftc.teamcode.utils.HertzCalculator;

@TeleOp(group = "test")
//@Disabled
public class HertzTest extends OpMode {
	HertzCalculator test = new HertzCalculator( new MiniTeleOp() );
	@Override
	public void init( ) {
		test.runCalculator( 10000 );
	}

	@Override
	public void loop( ) {

	}
}
