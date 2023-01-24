package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MotorType;

@Config
@TeleOp(group = "Test" )
public class PIDF_Tuning extends OpMode {

	private PIDController controller;

	public static double p = 0, i = 0, d = 0;
	public static double f = 0;

	public static int target = 0;
	public static int heading = 0;

	private Turret turret;
	private Lift leftLift;
	private Lift rightLift;

	@Override
	public void init( ) {
		controller = new PIDController( p, i, d );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		leftLift = new Lift( hardwareMap, "leftLift", false, /* clawOffSet.getZ( ) */ 0, 39.25 / 25.4 / 2, 90, AngleUnit.DEGREES, 384.5, 1, new PIDController( 0.02, 0, 0.00012 ) );
		rightLift = new Lift( hardwareMap, "rightLift", true, /* clawOffSet.getZ( ) */ 0, 39.25 / 25.4 / 2, 90, AngleUnit.DEGREES, 384.5, 1, new PIDController( 0.02, 0, 0.00012 ) );


		turret = new Turret( hardwareMap, "turr", false, AngleUnit.DEGREES, MotorType.Gobilda137.TICKS_PER_ROTATION, 170.0 / 30.0, -230, 45, new PIDController( 0,0,0 ) );
	}

	@Override
	public void loop( ) {
		controller.setPID( p, i, d );

		int turretPos = turret.getPosition( );
		target = turret.convertHeadingToTicks( heading );

		double pid = controller.calculate( turretPos, target );

		double power = pid + f;

		turret.setTurretPower( power );

		telemetry.addData( "turret pos", turretPos );
		telemetry.addData( "target", target );
		telemetry.addData( "PID", pid );
		telemetry.update();
	}
}
