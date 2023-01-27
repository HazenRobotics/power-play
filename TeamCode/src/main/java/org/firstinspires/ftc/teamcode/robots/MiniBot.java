package org.firstinspires.ftc.teamcode.robots;

import static org.firstinspires.ftc.teamcode.utils.localization.PPField.THREE_HALVES_TILE;
import static org.firstinspires.ftc.teamcode.utils.localization.PPField.TILE_CONNECTOR;
import static org.firstinspires.ftc.teamcode.utils.localization.PPField.TILE_SIZE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.roadrunner.MecanumDriveUnparalleled;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.CameraAngler;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.SingleServoClaw;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MotorType;
import org.firstinspires.ftc.teamcode.utils.RGBLights;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;
import org.firstinspires.ftc.teamcode.utils.localization.PPField.Junction;
import org.firstinspires.ftc.teamcode.vision.AprilTagsUtil;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline.SignalPosition;
import org.firstinspires.ftc.teamcode.vision.pipelines.SignalDetector;

import java.util.Scanner;

public class MiniBot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	//	public MecanumDrive mecanumDrive;
	public MecanumDriveUnparalleled drive;

	public Lift leftLift;
	public Lift rightLift;
	public Linkage linkage;
	//	public ServoTurret turret;
	public Turret turret;
	//	public Claw claw;
//	public RotatingClaw claw;
//	public TiltingClaw claw;
//	public TwoAxesClaw claw;
	public SingleServoClaw claw;
	//	public SignalUtil signalUtil;
	public AprilTagsUtil signalUtil;
	public BNO055IMU gyro;
//	public RGBLights lights;
	public CameraAngler angler;

	public boolean rightSide;

	public final float[] liftHeights = { 0, Junction.GROUND.height( ), Junction.LOW.height( ), Junction.MEDIUM.height( ), Junction.HIGH.height( ) };

//	Pose2d lastPose = drive.getPoseEstimate( );


	/*public enum RobotDimensions {
		FL( 7f, 6.625f ),
		BL( 7f, 6.625f ),
		FR( 6f, 7.5f ),
		BR( 6f, 6.875f );

		RobotDimensions( float x, float y ) {
			xOffset = x;
			yOffset = y;
		}

		public float xOffset;
		public float yOffset;

		public float getXOffset( ) {
			return xOffset;
		}

		public float getYOffset( ) {
			return yOffset;
		}
	}*/

	public static final float ROBOT_LENGTH = 14.75f;
	public static final float ROBOT_MAX_LENGTH = 15.75f; // with claw
	//	public static final float ROBOT_WIDTH = 14.125f;
	public static final float ROBOT_MAX_WIDTH = 13.75f;

	public static final float ROBOT_BACK_OFFSET = 6f;

	public static final float CLAW_OFFSET_X = 3.25f; // 3 1/4
	public static final float CLAW_OFFSET_Y = 1.375f; // 1 3/8
	public static final float CLAW_OFFSET = 12.0f;
	public static final float CLAW_CAMERA_OFFSET = 6.0f;

	public static boolean redSide = true;

	public static double autoEndHeading;

	public static Pose2d endAutoPos;

	public enum LiftPosition {
		BOTTOM, JNCTN_GROUND, JNCTN_LOW, JNCTN_MEDIUM, JNCTN_HIGH
	}

	/**
	 * Creates a Mini Robot
	 *
	 * @param op robot's opMode
	 */
	public MiniBot( OpMode op ) {
		this( op, true );
	}

	/**
	 * Creates a Robot
	 *
	 * @param op robot's opMode
	 */
	public MiniBot( OpMode op, boolean rightSide ) {
		super( op );

		Robot.writeToDefaultFile( "written", false, true );

		opMode = op;
		hardwareMap = op.hardwareMap;

		this.rightSide = rightSide;

//		mecanumDrive = new MecanumDrive( hardwareMap, "frontLeft", "backLeft/para", "frontRight/perp", "backRight" );
//		super.driveTrain = mecanumDrive;//REVERSE
		// note these must be the same as in MecanumDriveMini
//		mecanumDrive.setMotorDirections( DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD );
//		mecanumDrive.setWheelDiameter( 4 );
//		mecanumDrive.setPulsesPerRevolution( MotorType.Gobilda192.TICKS_PER_ROTATION );

		drive = new MecanumDriveUnparalleled( hardwareMap );

		leftLift = new Lift( hardwareMap, "leftLift", false, /* clawOffSet.getZ( ) */ 0, 39.25 / 25.4 / 2, 90, AngleUnit.DEGREES, 384.5, 1, new PIDController( 0.02, 0, 0.00012 ) );
		rightLift = new Lift( hardwareMap, "rightLift", true, /* clawOffSet.getZ( ) */ 0, 39.25 / 25.4 / 2, 90, AngleUnit.DEGREES, 384.5, 1, new PIDController( 0.02, 0, 0.00012 ) );

		leftLift.setEncoder( Lift.EncoderState.WITHOUT_ENCODER );
		rightLift.setEncoder( Lift.EncoderState.WITHOUT_ENCODER );

//		claw = new Claw( hardwareMap, "lClaw", "rClaw", new double[]{ 0.65, 0.75 }, new double[]{ 0.35, 0.25 } );

//		claw = new RotatingClaw( hardwareMap, "claw", "clawR", new double[]{ 0.35, 0.65 } );

//		claw = new TiltingClaw( hardwareMap, "claw", "clawV", new double[]{ 0.61, 0.45 }, new double[]{ 0.8, 0.43, 0.05 } );

//		claw = new TwoAxesClaw( hardwareMap, "claw", "clawH", "clawV", new double[]{ 0.61, 0.35 }, new double[]{ 1, 0.5, 0 }, new double[]{ 0.3, 0.53, 0.73 } );

		claw = new SingleServoClaw( hardwareMap, "claw", 1, 0.65 );

		turret = new Turret( hardwareMap, "turr", false, AngleUnit.DEGREES, MotorType.Gobilda137.TICKS_PER_ROTATION, 170.0 / 30.0, -230, 45, new PIDController( 0.005, 0, 0.0002 ) );

//		signalUtil = new SignalUtil( hardwareMap, "webcam1", telemetry );

		linkage = new Linkage( hardwareMap, "linkage", true,
				90, 10, 0,
				0.36, 0, 14, 7, 8.25 );

		signalUtil = new AprilTagsUtil( hardwareMap, "webcam1", telemetry );

		gyro = hardwareMap.get( BNO055IMU.class, "imu" );
		initGyro( );

//		lights = new RGBLights( hardwareMap, "blinkin" );

		angler = new CameraAngler( hardwareMap, "angler", 0, 0.3, 5, 85 );

		for( LynxModule module : hardwareMap.getAll( LynxModule.class ) )
			module.setBulkCachingMode( LynxModule.BulkCachingMode.AUTO );
	}

	public void waitSeconds( double seconds ) {
		double startTime = opMode.getRuntime( );
		while( opModeIsActive( ) && startTime + seconds > opMode.getRuntime( ) ) {
//			telemetry.addLine(  startTime + seconds + " > " + opMode.getRuntime( ) );
//			telemetry.update( );b
		}
	}

	public void initSubsystems( ) {
		signalUtil.init( );
//		claw.setState( SingleServoClaw.ClawState.CLOSED );
		claw.close( );
		linkage.moveToExtensionDistance( 0 );
//		claw.setState( TiltingClaw.VerticalClawState.STOWED );
	}

	private void initGyro( ) {
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters( );
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		gyro.initialize( parameters );
	}

//	public Vector3D getClawPos () {
//		final Pose2d robotPose = drive.getPoseEstimate();
//		Vector2d robotXyCoords = new Vector2d( robotPose.getX(), robotPose.getY() );
//
//		return new Vector3D(
//				robotXyCoords.getX() + Math.sin( turret.getTurretHeading() - (robotPose.getHeading() < 0 : robotPose.getHeading() ),
//				robotXyCoords.getX() + Math.cos( turret.getTurretHeading() ),
//				lift.getMotorPositionInch());
//	}

	public void updatePIDs( ) {
		leftLift.updatePID( 0.5 );
		rightLift.updatePID( 0.5 );
		turret.updatePID( 0.1 );
	}

	public void updatePIDs( double liftMult, double turretMult ) {
		leftLift.updatePID( liftMult );
		rightLift.updatePID( liftMult );
		turret.updatePID( turretMult );
	}

	public void setLiftTarget( int target ) {
		leftLift.setTarget( target );
		rightLift.setTarget( target );
	}

	public void setLiftTargetInches( double inches ) {
		leftLift.setTargetInches( inches );
		rightLift.setTargetInches( inches );
	}

	public boolean isOverJunction( ) {

		double play = 2;

		Pose2d poseEst = drive.getPoseEstimate( );
		double heading = poseEst.getHeading( ) + turret.getTurretHeading( AngleUnit.RADIANS );
		double x = poseEst.getX( ) + CLAW_OFFSET * Math.sin( heading );
		double y = poseEst.getY( ) + CLAW_OFFSET * Math.cos( heading );

		double dist = TILE_CONNECTOR + TILE_SIZE;

		return Math.abs( x % dist ) < play && Math.abs( y % dist ) < play;
	}


	/**
	 * @param right true if on the right side from the perspective of the driver
	 * @return the starting position of the robot in these circumstances
	 */
	public Pose2d getStartPos( boolean right ) {
		double x = right ? (TILE_SIZE + 3 * TILE_CONNECTOR / 2 + ROBOT_MAX_WIDTH / 2) : -(2 * TILE_SIZE + 3 * TILE_CONNECTOR / 2 - ROBOT_MAX_WIDTH / 2);
		return new Pose2d( x, -(3 * (TILE_CONNECTOR + TILE_SIZE) - ROBOT_BACK_OFFSET/*MiniBot.ROBOT_LENGTH / 2*/), Math.toRadians( 90 ) );
	}

	/**
	 * @param red   true if on the red side, false for blue
	 * @param right true if on the right side from the perspective of the driver
	 * @return the starting position of the robot in these circumstances
	 */
	public Pose2d getStartPos( boolean red, boolean right ) {
		double x, y, heading;
		if( red ) { // red side
			x = right ? (TILE_SIZE + 3 * TILE_CONNECTOR / 2 + ROBOT_MAX_WIDTH / 2) : -(2 * TILE_SIZE + 3 * TILE_CONNECTOR / 2 - ROBOT_MAX_WIDTH / 2);
			y = -(3 * (TILE_CONNECTOR + TILE_SIZE) - MiniBot.ROBOT_LENGTH / 2);
			heading = Math.toRadians( 90 );
		} else { // blue side
			x = right ? -(TILE_SIZE + 3 * TILE_CONNECTOR / 2 + ROBOT_MAX_WIDTH / 2) : (2 * TILE_SIZE + 3 * TILE_CONNECTOR / 2 - ROBOT_MAX_WIDTH / 2);
			y = (3 * (TILE_CONNECTOR + TILE_SIZE) - MiniBot.ROBOT_LENGTH / 2);
			heading = Math.toRadians( 270 );
		}
		return new Pose2d( x, y, heading );
	}

	/**
	 * signal needs to be initted!!
	 * robot.signalUtil.init( );
	 *
	 * @param right true if on right side of the field
	 * @return the position to park
	 */
	public Vector2d parkPosInit( boolean right, SignalPosition position ) {

		SignalPosition signalPosition = signalUtil.getSignalPosition( );
//		SignalDetector.SignalPosition signalPosition = SignalDetector.SignalPosition.RIGHT;
//		SignalDetector.SignalPosition signalPosition = SignalDetector.SignalPosition.MIDDLE;
//		SignalDetector.SignalPosition signalPosition = SignalDetector.SignalPosition.LEFT;
//		SignalDetector.SignalPosition signalPosition = null;

		double tilePos = 0.05;
		if( position == SignalPosition.LEFT )
			tilePos = -1;
		else if( position == SignalPosition.RIGHT )
			tilePos = 1;

		float THREE_HALVES = 3f / 2 * TILE_CONNECTOR + THREE_HALVES_TILE;

		return new Vector2d( (right ? 1 : -1) * THREE_HALVES + tilePos * (TILE_SIZE), -TILE_SIZE / 2 );
	}

	/**
	 * signal needs to be initted!!
	 * robot.signalUtil.init( );
	 *
	 * @param red   true if on the red side
	 * @param right true if on the right side from team/driver pov
	 * @return the Vector2d position for where to park
	 */
	public Vector2d parkPosInit( boolean red, boolean right ) {

		double x, y;

		SignalPosition signalPosition = signalUtil.getSignalPosition( );

		double tilePos = 0.05;
		if( signalPosition == SignalPosition.LEFT )
			tilePos = -1;
		else if( signalPosition == SignalPosition.RIGHT )
			tilePos = 1;

		if( red ) {
			x = right ? (THREE_HALVES_TILE + tilePos * (TILE_SIZE + 3)) : (-THREE_HALVES_TILE + tilePos * (TILE_SIZE + 3));
			y = -THREE_HALVES_TILE;
		} else {
			x = right ? (-THREE_HALVES_TILE + tilePos * (TILE_SIZE + 3)) : (THREE_HALVES_TILE - tilePos * (TILE_SIZE + 3));
			y = THREE_HALVES_TILE;
		}

		return new Vector2d( x, y );
	}

	public static Vector2d getSignalPos( boolean right ) {
		return new Vector2d( (right ? 1 : -1) * (THREE_HALVES_TILE + 3), -THREE_HALVES_TILE - 5 );
	}

	public static Vector2d getSignalPos( boolean red, boolean right ) {
		double x, y;
		if( red ) { // red side
			x = right ? THREE_HALVES_TILE : -THREE_HALVES_TILE;
			y = -THREE_HALVES_TILE;
		} else { // blue side
			x = right ? -THREE_HALVES_TILE : THREE_HALVES_TILE;
			y = THREE_HALVES_TILE;
		}
		return new Vector2d( x, y );
	}

	public void liftPower( double power ) {
		leftLift.setPower( power );
		rightLift.setPower( power );
	}

	public void liftToHeightPower( double power, double height ) {
		leftLift.setHeightPower( power, height );
		rightLift.setHeightPower( power, height );
	}

	public void liftToHeightPowerNotAsync( double power, double height ) {
		leftLift.setHeightPower( power, height, false, true );
		rightLift.setHeightPower( power, height, false, true );
	}

	public void junctionToLiftPos( PPField.Junction junction ) {
		junctionToLiftPos( junction, 3 );
	}

	public void junctionToLiftPos( PPField.Junction junction, double extension ) {
		leftLift.setHeightPower( 1, junction.height( ) + extension );
		rightLift.setHeightPower( 1, junction.height( ) + extension );
	}

	public void junctionToLiftPosNotAsync( PPField.Junction junction ) {
		leftLift.setHeightPower( 1, junction.height( ) + 3, false, false );
		rightLift.setHeightPower( 1, junction.height( ) + 3, false, false );
	}

	/**
	 * @param angle the heading to face the Junction (degrees)
	 * @return the position/heading (Pose2D) of where to go
	 */
	public static Pose2d getJunctionOffsetPos( double angle, int junctionX, int junctionY ) {
		Vector2d junctionPos = PPField.getJunctionPose( junctionX, junctionY, false );
		angle = Math.toRadians( angle );
		double x = junctionPos.getX( ) - Math.cos( angle ) * ROBOT_MAX_LENGTH;
		double y = junctionPos.getY( ) - Math.sin( angle ) * ROBOT_MAX_LENGTH;
		return new Pose2d( x, y, angle );
	}

	/**
	 * @param angle     angle of robot relative to pole
	 * @param distance  the distance away from the pole to center of robot
	 * @param heading   end heading of robot
	 * @param junctionX x pos of junction
	 * @param junctionY y pos of junction
	 * @return the position/heading (Pose2D) of where to go
	 */
	public static Pose2d getJunctionOffsetPos( double angle, double distance, double heading, int junctionX, int junctionY ) {
		Vector2d junctionPos = PPField.getJunctionPose( junctionX, junctionY, false );
		double dist = (distance + ROBOT_LENGTH / 2);
		angle = Math.toRadians( angle );
		double x = junctionPos.getX( ) - Math.cos( angle ) * distance;
		double y = junctionPos.getY( ) - Math.sin( angle ) * distance;
		return new Pose2d( x, y, Math.toRadians( heading ) );
	}

	/**
	 * @param angle            the heading to face the Junction (degrees)
	 * @param junctionDistance the distance away from the shipping hub base to be
	 * @param junctionPos      the position of the junction to go to
	 * @return the position/heading (Pose2D) of where to go
	 */
	public static Pose2d getJunctionOffsetPos( double angle, double junctionDistance, Vector2d junctionPos ) {
		double dist = (junctionDistance + ROBOT_LENGTH / 2);
		angle = Math.toRadians( angle );
		double x = junctionPos.getX( ) - Math.cos( angle ) * junctionDistance;
		double y = junctionPos.getY( ) - Math.sin( angle ) * junctionDistance;
		return new Pose2d( x, y, angle );
	}


	public static double getAngleOnSide( boolean red, boolean right ) {

		if( red ) {
			if( right ) { // red right
				return 135;
			} else { // red left
				return 45;
			}
		} else {
			if( right ) { // blue right
				return 315;
			} else { // blue left
				return 225;
			}
		}
	}

	/**
	 * @param red   whether the bot is on the red side of the field
	 * @param right whether the bot is on the right side of the field from that side's perspective
	 * @return an int array of length 2 containing the sign (1 or -1) of the quadrant the robot must be in
	 */
	public static int[] getQuadrantSign( boolean red, boolean right ) {
		return new int[]{ (red ? 1 : -1) * (right ? 1 : -1), (red ? -1 : 1) };
	}

	public TrajectorySequenceBuilder getTrajectorySequenceBuilder( ) {
		return drive.trajectorySequenceBuilder( drive.getPoseEstimate( ) );
	}

	public TrajectorySequenceBuilder getTrajectorySequenceBuilder( Pose2d pose ) {
		return drive.trajectorySequenceBuilder( pose );
	}

	public TrajectoryBuilder getTrajectoryBuilder( ) {
		return drive.trajectoryBuilder( drive.getPoseEstimate( ) );
	}

	public static void setRedSide( boolean redSide ) {
		MiniBot.redSide = redSide;
	}

	public boolean inSubstation( ) {
		Pose2d pose = drive.getPoseEstimate( );
		double maxX = 14 + ROBOT_MAX_LENGTH;
		double maxY = 58 + ROBOT_MAX_WIDTH;
		return (pose.getX( ) > -maxX && pose.getX( ) < maxX) &&
				(pose.getY( ) > maxY || pose.getY( ) < -maxY);
	}


	/**
	 * @return if the robot is on the red side of the field
	 */
	public static boolean getRedSide( ) {
		return redSide;
	}

	public double getJunctionTurretHeading( int junctionX, int junctionY ) {
		return getJunctionTurretHeading( drive.getPoseEstimate( ), junctionX, junctionY );
	}

	/**
	 *
	 * @param angle angle in degrees
	 */
	public static double normalizePM180( double angle ) {
		angle = ( angle % 360 + 360 ) % 360; // [0, 360]
		return angle < 180 ? angle % 180 : angle - 360;
	}

	public static final double TWO_PI = 2 * Math.PI;

	public static double getJunctionTurretHeading( Pose2d robotPos, int junctionX, int junctionY ) {
		Vector2d junctionPos = PPField.getJunctionPose( junctionX, junctionY, false );

		double robotAngleOffset = Math.atan2( robotPos.getY( ) - junctionPos.getY( ),
				robotPos.getX( ) - junctionPos.getX( ) ) + Math.PI;
		double angle = -( ( TWO_PI + robotAngleOffset - robotPos.getHeading( ) ) % TWO_PI );

		return normalizePM180( Math.toDegrees( angle ) );
	}

}
