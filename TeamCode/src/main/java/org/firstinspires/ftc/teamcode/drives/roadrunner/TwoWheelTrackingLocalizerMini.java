package org.firstinspires.ftc.teamcode.drives.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelTrackingLocalizerMini extends TwoTrackingWheelLocalizer {

	public static double TICKS_PER_REV = 4000;
	public static double WHEEL_RADIUS = 0.728; // in
	public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

	// 210mm distance between deadwheels

//	1st x multi: 1.027789134213273097276815589505
//  2nd x multi: 1.019055199955614484624155458003
//  3rd x multi: 1.035457516903843963455252703407

//	1st y multi: 1.0067812309131058306057019614336
//  2nd y multi: 1.0180926373802892740547009861924
//  3rd y multi: 1.0111746028892628986555871890919
	public static double X_MULTIPLIER = 1.02743; // Multiplier in the X direction
	public static double Y_MULTIPLIER = 1.01201; // Multiplier in the Y direction

	public static double PARALLEL_X = 0.394; // X is the up and down direction
	public static double PARALLEL_Y = -4.331; // Y is the strafe direction

	public static double PERPENDICULAR_X = 0.787;
	public static double PERPENDICULAR_Y = 3.937;

	// Parallel/Perpendicular to the forward axis
	// Parallel wheel is parallel to the forward axis
	// Perpendicular is perpendicular to the forward axis
	private Encoder parallelEncoder, perpendicularEncoder;

	private MecanumDriveMini drive;

	public TwoWheelTrackingLocalizerMini( HardwareMap hardwareMap, MecanumDriveMini drive ) {
		super( Arrays.asList(
				new Pose2d( PARALLEL_X, PARALLEL_Y, 0 ),
				new Pose2d( PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians( 90 ) )
		) );

		this.drive = drive;

		parallelEncoder = new Encoder( hardwareMap.get( DcMotorEx.class, "para" ) );
		perpendicularEncoder = new Encoder( hardwareMap.get( DcMotorEx.class, "perp" ) );

		// TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
		parallelEncoder.setDirection( Encoder.Direction.REVERSE );
	}

	public static double encoderTicksToInches( double ticks ) {
		return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
	}

	@Override
	public double getHeading( ) {
		return drive.getRawExternalHeading( );
	}

	@Override
	public Double getHeadingVelocity( ) {
		return drive.getExternalHeadingVelocity( );
	}

	@NonNull
	@Override
	public List<Double> getWheelPositions( ) {
		return Arrays.asList(
				encoderTicksToInches( parallelEncoder.getCurrentPosition( ) ) * X_MULTIPLIER,
				encoderTicksToInches( perpendicularEncoder.getCurrentPosition( ) ) * Y_MULTIPLIER
		);
	}

	@NonNull
	@Override
	public List<Double> getWheelVelocities( ) {
		// TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
		//  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
		//  compensation method

		return Arrays.asList(
				encoderTicksToInches( parallelEncoder.getRawVelocity() ) * X_MULTIPLIER,
				encoderTicksToInches( perpendicularEncoder.getRawVelocity() ) * Y_MULTIPLIER
		);
	}
}