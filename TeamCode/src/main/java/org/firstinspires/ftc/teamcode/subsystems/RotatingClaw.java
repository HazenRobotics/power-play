package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RotatingClaw {

	public enum ClawState {
		OPEN, CLOSED;
	}

	ClawState state = ClawState.CLOSED;

	Servo clawServo;
	Servo rotationServo;

	double[] clawPositions;

	public RotatingClaw( HardwareMap hw ) {
		this( hw, "claw", "clawR", new double[]{ 0, 0.5 } );
	}

	public RotatingClaw( HardwareMap hw, String claw, String rotation, double[] clawPositions ) {
		clawServo = hw.servo.get( claw );
		rotationServo = hw.servo.get( rotation );

		this.clawPositions = clawPositions;
	}

	public double[] getPositions( ) {
		return new double[]{ clawServo.getPosition( ), rotationServo.getPosition( ) };
	}

	public void rotateRight( double rotation ) {

	}

	public void rotateRight( ) {
		rotate( 0.05 );
	}

	public void rotateLeft( ) {
		rotate( 0.05 );
	}

	public void rotate( double rotation ) {
		setRotatePos( getRotatePos( ) + rotation );
	}

	public void setRotatePos( double pos ) {
		rotationServo.setPosition( pos );
	}

	public double getRotatePos( ) {
		return rotationServo.getPosition( );
	}

	public void close( ) {
		clawServo.setPosition( clawPositions[0] );
		state = ClawState.CLOSED;
	}

	public void open( ) {
		clawServo.setPosition( clawPositions[1] );
		state = ClawState.OPEN;
	}

	public void toggle( ) {
		if( state == ClawState.OPEN )
			close( );
		else
			open( );
	}

	public ClawState getState( ) {
		return state;
	}


}
