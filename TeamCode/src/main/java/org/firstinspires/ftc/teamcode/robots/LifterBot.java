package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.roadrunner.MecanumDriveLifter;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MotorType;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;

public class LifterBot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public MecanumDrive mecanumDrive;
	public MecanumDriveLifter drive;

	public Lift verticalLift;
//	public Lift horizontalLift;
//	public Turret turret;
	public Claw claw;

	public static final float ROBOT_LENGTH = 13.375f;
	public static final float ROBOT_WIDTH = 12.75f;
	public static final float ROBOT_MAX_WIDTH = 14.5f;


	public enum LiftPosition {
		BOTTOM, JNCTN_GROUND, JNCTN_LOW, JNCTN_MEDIUM, JNCTN_HIGH;
	}

	public static final Vector3D clawOffSet = new Vector3D( 15.5, 1.075, 2 );
	/**
	 * Creates a Robot
	 *
	 * @param op robot's opMode
	 */
	public LifterBot( OpMode op ) {
		super( op );

		Robot.writeToDefaultFile( "written", false, true );

		opMode = op;
		hardwareMap = op.hardwareMap;

		super.driveTrain = new MecanumDrive( hardwareMap );
		mecanumDrive = (MecanumDrive) driveTrain;
		mecanumDrive.setMotorDirections( DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD );
		mecanumDrive.setWheelDiameter( 4 );
		mecanumDrive.setPulsesPerRevolution( MotorType.Gobilda192.TICKS_PER_ROTATION );

		drive = new MecanumDriveLifter( hardwareMap );

		verticalLift = new Lift( hardwareMap, "lift", true, 0, 39.25 / 25.4 / 2, 0, AngleUnit.DEGREES );
//		horizontalLift = new Lift( hardwareMap, "hLift", false, 0, 0.5, 0, AngleUnit.DEGREES );
//		turret = new Turret( hardwareMap, "turret", true, AngleUnit.DEGREES, MotorType.Gobilda192.TICKS_PER_ROTATION, 4.0 /* driven / driver */ );
//		turret.setLimit( -200, 200 );

		claw = new Claw( hardwareMap, "lClaw", "rClaw", new double[]{ 0.5, 0.75 }, new double[]{ 0.5, 0.25 } );
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
//		horizontalLift.setHeightPower( powers[1], clawPos.getY( ) );
//		turret.setRotationPower( powers[2], rotation, angleUnit );
	}

	public void junctionToLiftPos( PPField.Junction junction ) {
		verticalLift.moveDistancePower( 1, junction.height( ) + 1.5, true );
	}

	/**
	 * @param angle            the heading to face the Junction (degrees)
	 * @param angleOffset      the starting angle of the robots
	 * @param junctionDistance the distance away from the shipping hub base to be
	 * @param junctionPos      the position of the junction to go to
	 * @return the position/heading (Pose2D) of where to go
	 */
	public Pose2d stackJunctionPosition( double angle, double angleOffset, double junctionDistance, com.acmerobotics.roadrunner.geometry.Vector2d junctionPos ) {
		double dist = (junctionDistance + ROBOT_LENGTH / 2);
		angle = Math.toRadians( angle );
		double x = junctionPos.getX( ) - Math.cos( angle ) * dist;
		double y = junctionPos.getY( ) - Math.sin( angle ) * dist;
		return new Pose2d( x, y, Math.toRadians( angleOffset ) + angle );
	}

}
