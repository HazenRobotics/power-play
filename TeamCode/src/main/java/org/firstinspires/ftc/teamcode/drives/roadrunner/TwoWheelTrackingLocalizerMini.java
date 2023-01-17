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
	public static double WHEEL_RADIUS = 0.75; // in
	public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

	// 210mm distance between deadwheels

//	0.98903375147251753550445135598407
//	0.98739220883145156590682343984545
//  0.98535061658064837698561555819895
	public static double X_MULTIPLIER = 0.98725885896153915946563011800949; // 1.02743 Multiplier in the X direction
//  0.9867006438220013926146611499961
//  0.98952878916097665534438197194642
//  0.9920634044666234639656413357742
	public static double Y_MULTIPLIER = 0.98943094581653383730822815257224; // 1.01201 Multiplier in the Y direction

	public static double PARALLEL_X = -0.787402; // 0.394 X is the up and down direction
	public static double PARALLEL_Y = 4.1417323; // 0.787 Y is the strafe direction

	public static double PERPENDICULAR_X = -0.6102362; //0.787
	public static double PERPENDICULAR_Y = -4.8228346; // 3.937

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

		parallelEncoder = new Encoder( hardwareMap.get( DcMotorEx.class, "backLeft/paraL" ) );
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