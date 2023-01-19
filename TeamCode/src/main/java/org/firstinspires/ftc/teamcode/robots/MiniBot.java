package org.firstinspires.ftc.teamcode.robots;

import static org.firstinspires.ftc.teamcode.utils.localization.PPField.THREE_HALVES_TILE;
import static org.firstinspires.ftc.teamcode.utils.localization.PPField.TILE_CONNECTOR;
import static org.firstinspires.ftc.teamcode.utils.localization.PPField.TILE_SIZE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.roadrunner.MecanumDriveMini;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.SingleServoClaw;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.teleop.MiniTeleOp;
import org.firstinspires.ftc.teamcode.utils.MotorType;
import org.firstinspires.ftc.teamcode.utils.RGBLights;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;
import org.firstinspires.ftc.teamcode.vision.AprilTagsUtil;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline.SignalPosition;

public class MiniBot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public MecanumDrive mecanumDrive;
	public MecanumDriveMini drive;

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
	public RGBLights lights;

	public boolean rightSide;
//	Pose2d lastPose = drive.getPoseEstimate( );


	/*public enum89. RobotDimensions {
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

	public static final float ROBOT_LENGTH = 13.5f;
	public static final float ROBOT_MAX_LENGTH = 15.75f; // with claw
	//	public static final float ROBOT_WIDTH = 14.125f;
	public static final float ROBOT_MAX_WIDTH = 13.375f;

	public static final float ROBOT_BACK_OFFSET = 6f;

	public static final float CLAW_OFFSET = 12.0f;

	public static boolean redSide = true;

	public static double autoEndHeading;

	public static Pose2d endAutoPos;

	public enum LiftPosition {
		BOTTOM, JNCTN_GROUND, JNCTN_LOW, JNCTN_MEDIUM, JNCTN_HIGH
	}

//	public static final Vector3D clawOffSet = new Vector3D( 0, 12, 3 );

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

		mecanumDrive = new MecanumDrive( hardwareMap, "frontLeft", "backLeft/paraL", "frontRight/paraR", "backRight" );
//		super.driveTrain = mecanumDrive;//REVERSE
		// note these must be the same as in MecanumDriveMini
		mecanumDrive.setMotorDirections( DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD );
//		mecanumDrive.setWheelDiameter( 4 );
//		mecanumDrive.setPulsesPerRevolution( MotorType.Gobilda192.TICKS_PER_ROTATION );

		drive = new MecanumDriveMini( hardwareMap );

		leftLift = new Lift( hardwareMap, "leftLift", false, /* clawOffSet.getZ( ) */ 0, 39.25 / 25.4 / 2, 90, AngleUnit.DEGREES, 103.6, 1 );
		rightLift = new Lift( hardwareMap, "rightLift", true, /* clawOffSet.getZ( ) */ 0, 39.25 / 25.4 / 2, 90, AngleUnit.DEGREES, 103.6, 1 );

//		claw = new Claw( hardwareMap, "lClaw", "rClaw", new double[]{ 0.65, 0.75 }, new double[]{ 0.35, 0.25 } );

//		claw = new RotatingClaw( hardwareMap, "claw", "clawR", new double[]{ 0.35, 0.65 } );

//		claw = new TiltingClaw( hardwareMap, "claw", "clawV", new double[]{ 0.61, 0.45 }, new double[]{ 0.8, 0.43, 0.05 } );

//		claw = new TwoAxesClaw( hardwareMap, "claw", "clawH", "clawV", new double[]{ 0.61, 0.35 }, new double[]{ 1, 0.5, 0 }, new double[]{ 0.3, 0.53, 0.73 } );

		claw = new SingleServoClaw( hardwareMap, "claw", 0.48, 0.13 );

		turret = new Turret( hardwareMap, "turr", false, AngleUnit.DEGREES, MotorType.Gobilda137.TICKS_PER_ROTATION, 170.0 / 30.0, -230, 50 );

//		signalUtil = new SignalUtil( hardwareMap, "webcam1", telemetry );

		linkage = new Linkage( hardwareMap, "linkage", true,
				90, 0, 0,
				0.3, 0, 1, 1, 1 );

		signalUtil = new AprilTagsUtil( hardwareMap, "webcam1", telemetry );

		gyro = hardwareMap.get( BNO055IMU.class, "imu" );
		initGyro( );

		lights = new RGBLights( hardwareMap, "blinkin" );
	}

	public void waitSeconds( double seconds ) {
		double startTime = opMode.getRuntime( );
		while( /*opModeIsActive( ) && */startTime + seconds > opMode.getRuntime( ) ) {
//			telemetry.addLine(  startTime + seconds + " > " + opMode.getRuntime( ) );
//			telemetry.update( );b
		}
	}

	public void initSubsystems( ) {
		signalUtil.init( );
		claw.setState( SingleServoClaw.ClawState.CLOSED );
		waitSeconds( 0.25 );
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

	public boolean isOverJunction( ) {

		double play = 2;

		Pose2d poseEst = drive.getPoseEstimate( );
		double heading = poseEst.getHeading( ) + turret.getTurretHeading( AngleUnit.RADIANS );
		double x = poseEst.getX( ) + CLAW_OFFSET * Math.sin( heading );
		double y = poseEst.getY( ) + CLAW_OFFSET * Math.cos( heading );

		double dist = TILE_CONNECTOR + TILE_SIZE;

		return Math.abs( x % dist ) < play && Math.abs( y % dist ) < play;
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

//		lift.setHeightPower( powers[0], clawPos.getX( ) );
//		horizontalLift.setHeightPower( powers[1], clawPos.getY( ) );
//		turret.setRotationPower( powers[2], rotation, angleUnit );
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
	public Vector2d parkPosInit( boolean right ) {

		SignalPosition signalPosition = signalUtil.getSignalPosition( );
//		SignalDetector.SignalPosition signalPosition = SignalDetector.SignalPosition.RIGHT;
//		SignalDetector.SignalPosition signalPosition = SignalDetector.SignalPosition.MIDDLE;
//		SignalDetector.SignalPosition signalPosition = SignalDetector.SignalPosition.LEFT;
//		SignalDetector.SignalPosition signalPosition = null;

		double tilePos = 0.05;
		if( signalPosition == SignalPosition.LEFT )
			tilePos = -1;
		else if( signalPosition == SignalPosition.RIGHT )
			tilePos = 1;

		float THREE_HALVES = 3f / 2 * TILE_CONNECTOR + THREE_HALVES_TILE;

		return new Vector2d( (right ? 1 : -1) * THREE_HALVES + tilePos * (TILE_SIZE), -THREE_HALVES * 1 );
	}

	/**
	 * signal needs to be initted!!
	 * robot.signalUtil.init( );
	 *
	 * @param red   true if on the red side
	 * @param right
	 * @return
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

	public void liftToHeightPower( double power, double height) {
		leftLift.setHeightPower( power, height );
		rightLift.setHeightPower( power, height );
	}

	public void liftToHeightPowerNotAsync( double power, double height) {
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
		return ( pose.getX( ) > -maxX && pose.getX( ) < maxX ) &&
				( pose.getY( ) > maxY || pose.getY( ) < -maxY );
	}


	/**
	 * @return if the robot is on the red side of the field
	 */
	public static boolean getRedSide( ) {
		return redSide;
	}

}
