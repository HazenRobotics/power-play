package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.Robot;

public class Turret {

	public DcMotorEx motor;

	AngleUnit unit;

	double pulsesPerRevolution;
	double gearRatio;

	int turretPosition;

	public enum MovementState {
		REST, MOVING;
	}

	MovementState movementState = MovementState.REST;

	double leftLimit = Double.NEGATIVE_INFINITY;
	double rightLimit = Double.POSITIVE_INFINITY;

	public Turret( HardwareMap hw ) {
		this( hw, "turret", false, AngleUnit.RADIANS, 1, 1, 0, 360 );
	}

	public Turret( HardwareMap hw, String motorName, boolean reverseMotor, AngleUnit angleUnit, double ppr, double gearRatio, double lLimit, double rLimit ) {
		unit = angleUnit;

		motor = hw.get( DcMotorEx.class, motorName );

		motor.setDirection( reverseMotor ? Direction.REVERSE : Direction.FORWARD );
//		motor.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		motor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

		pulsesPerRevolution = ppr;
		this.gearRatio = gearRatio;

		turretPosition = 0;

		setLimit( lLimit, rLimit );

		resetTurret( );
	}

	public void setPower( double power ) {
		motor.setPower( power );
	}

	public void setLimit( double left, double right ) {
		double multiplyer = 1;
		if( unit.equals( AngleUnit.DEGREES ) )
			multiplyer = Math.PI / 180;
		leftLimit = left * multiplyer;
		rightLimit = right * multiplyer;
	}

	public void setRotationVelocity( double velocity, double position ) {
		motor.setTargetPosition( (int) convertTicksToHeading( position ) );
		motor.setVelocity( velocity, unit );
	}

	/**
	 * @param power    power to turn at
	 * @param position the rotation of the turret in the predetermined AngleUnit
	 */
	public void setRotationPower( double power, double position ) {
		setRotationPower( power, position, unit, true );
	}

	public void setLiveRotationPower( Vector2d move ) {


		double tolerance = 0.0;

		double moveX = move.getX( );
		double moveY = move.getY( );

		double target = move.angle( ); // target angle between
		double power = Math.sqrt( moveX * moveX + moveY * moveY - (moveX * moveX * moveY * moveY) );

		double currentHeading = getTurretHeading( );

		Robot.writeToDefaultFile( tolerance + ", " + move + ", " + target + ", " + currentHeading, true, false );


		if( target < leftLimit || target > rightLimit )
			power *= -1;

		if( target > currentHeading + tolerance )
			motor.setPower( -power );
		else if( target < currentHeading - tolerance )
			motor.setPower( power );
	}

	public void setRotationPower( double power, double position, AngleUnit angleUnit, boolean async ) {
		position = angleUnit == AngleUnit.DEGREES ? position : Math.toDegrees( position );

		motor.setTargetPosition( convertHeadingToTicks( position ) );

		motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		if (position < getTurretHeading())
			power *= -1;

		motor.setPower( power );

		if( async ) {
			// create a new thread so that it doesn't interfere with other mechanisms
			new Thread( ( ) -> {
				waitForMoveFinish( );
				setPower( 0 );
			} ).start( );
		} else {
			waitForMoveFinish( );
			setPower( 0 );
		}

	}

	public void waitForMoveFinish( ) {
		while( isBusy( ) ) {
			try {
				Thread.sleep( 50 );
			} catch( InterruptedException ignored ) {
			}
		}
	}

	public boolean isBusy( ) {
		return motor.isBusy( );
	}

	public void turnToPosPower( double power, double position, AngleUnit angleUnit ) {
		position = angleUnit == AngleUnit.DEGREES ? position : Math.toDegrees( position );

		if( position > getTurretHeading( ) ) {
			motor.setPower( power );
			while( position > getTurretHeading( ) ) {}
		} else if( position < getTurretHeading( ) ) {
			motor.setPower( -power );
			while( position < getTurretHeading( ) );
		}
		setPower( 0 );
	}

	public void turnToPosPower( double power, double position  ) {
		turnToPosPower( power, position, unit );
	}

	public double convertTicksToHeading( double ticks ) {
		return convertTicksToHeading( ticks, unit );
	}

	public double convertTicksToHeading( double ticks, AngleUnit angleUnit ) {
		double heading = (2 * Math.PI * ticks) / (gearRatio * pulsesPerRevolution);

		return angleUnit == AngleUnit.DEGREES ? Math.toDegrees( heading ) : heading;
	}

	public int convertHeadingToTicks( double heading ) {
		return convertHeadingToTicks( heading, unit );
	}

	public int convertHeadingToTicks( double heading, AngleUnit angleUnit ) {
		heading = unit == AngleUnit.RADIANS ? heading : Math.toRadians( heading );

		double ticks = (heading * gearRatio * pulsesPerRevolution) / (2 * Math.PI);

		return (int) ticks;
	}


	public double getTurretHeading( ) {
		return getTurretHeading( unit );
	}

	public double getTurretHeading( AngleUnit angleUnit ) {
		double heading = (2 * Math.PI * getPosition( )) / (gearRatio * pulsesPerRevolution);

		return angleUnit == AngleUnit.DEGREES ? Math.toDegrees( heading ) : heading;
	}

	public int getPosition( ) {
		return /*turretPosition +*/ motor.getCurrentPosition( );
	}

	/**
	 * stops and resets the physical motor and its encoder and sets turretHeading to 0
	 */
	public void resetTurret( ) {
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		movementState = MovementState.REST;
		turretPosition = 0;
		motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
	}

	/**
	 * adds the current motor position to liftPosition then stops and resets the encoder
	 */
	public void stopAndReset( ) {

		Log.d( "LOGGER", "motor position: " + motor.getCurrentPosition( ) );
		turretPosition = getPosition( );
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		// stop and reset encoder sets the encoder position to zero
	}

	public double getCurrent( ) {
		return getCurrent( CurrentUnit.AMPS );
	}

	public double getCurrent( CurrentUnit currentUnit ) {
		return motor.getCurrent( currentUnit );
	}

}
