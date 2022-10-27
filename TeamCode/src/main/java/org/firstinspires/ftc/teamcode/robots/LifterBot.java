package org.firstinspires.ftc.teamcode.robots;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.roadrunner.MecanumDriveLifter;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class LifterBot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public MecanumDrive mecanumDrive;
	public MecanumDriveLifter drive;

	public Lift verticalLift;
	public Lift horizontalLift;
	public Turret turret;
	public Claw claw;

	public static final float ROBOT_LENGTH = 13.375f;
	public static final float ROBOT_WIDTH = 12.75f;
	public static final float ROBOT_MAX_WIDTH = 14.5f;

	public enum LiftPosition {
		BOTTOM, JNCTN_GROUND, JNCTN_LOW, JNCTN_MEDIUM, JNCTN_HIGH;
	}

	/**
	 * Creates a Robot
	 *
	 * @param op robot's opMode
	 */
	public LifterBot( OpMode op ) {
		super( op );

		Robot.writeToDefaultFile( "", false, false );

		opMode = op;
		hardwareMap = op.hardwareMap;

		super.driveTrain = new MecanumDrive( hardwareMap );
		mecanumDrive = (MecanumDrive) driveTrain;

		drive = new MecanumDriveLifter( hardwareMap );

		verticalLift = new Lift( hardwareMap, "vLift", false, 0, 39.25 / 25.4 / 2, 0, AngleUnit.DEGREES );
//		horizontalLift = new Lift( hardwareMap, "hLift", false, 0, 0.5, 0, AngleUnit.DEGREES );
		turret = new Turret( hardwareMap );

		claw = new Claw( hardwareMap, "lClaw", "rClaw" );
	}

	public void setClawPos( Vector3D clawPos, double... powers ) {

		Vector2d xyPos = new Vector2d( clawPos.getX( ), clawPos.getY( ) );

		setClawPos( xyPos, xyPos.angle( ), AngleUnit.RADIANS, powers );
	}

	public void setClawPos( Vector2d clawPos, double rotation, AngleUnit angleUnit, double... powers ) {
		if( powers.length == 0 )
			return;
		if( powers.length == 1 )
			powers = new double[]{ powers[0], powers[0], powers[0] };
		if( powers.length == 2 )
			powers = new double[]{ powers[0], powers[0], powers[1] };
		if( powers.length == 3 )
			powers = new double[]{ powers[0], powers[1], powers[2] };

		verticalLift.setHeightPower( powers[0], clawPos.getX( ) );
		horizontalLift.setHeightPower( powers[1], clawPos.getY( ) );
		turret.setRotationPower( powers[2], rotation, angleUnit );
	}

}
