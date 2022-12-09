package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TiltingClaw {

	public enum ClawState {
		OPEN, CLOSED;

		public double pos = 0;
	}

	public enum VerticalClawState {
		PICKUP, DEPLOYED, STOWED;

		public double pos = 0;
	}


	ClawState clawState = ClawState.OPEN;
	VerticalClawState verticalState = VerticalClawState.DEPLOYED;

	Servo clawServo;
	Servo verticalServo;

	public TiltingClaw( HardwareMap hw ) {
		this( hw, "claw", "clawV", new double[]{ 0, 0.5 }, new double[]{ 0.25, 0.5, 0.75 } );
	}

	/**
	 * @param clawName            hwMap name of the opening/closing claw
	 * @param verticalName        hwMap name of the vertical rotating claw
	 * @param clawPositions       the double positions of the claw (open, closed)
	 * @param verticalPositions   the double positions of the claw (pickup, deployed, stowed)
	 */
	public TiltingClaw( HardwareMap hw, String clawName, String verticalName, double[] clawPositions, double[] verticalPositions ) {
		clawServo = hw.servo.get( clawName );
		verticalServo = hw.servo.get( verticalName );

		if( clawPositions.length >= 2 ) {
			ClawState.OPEN.pos = clawPositions[0];
			ClawState.CLOSED.pos = clawPositions[1];
		}

		if( verticalPositions.length >= 3 ) {
			VerticalClawState.PICKUP.pos = verticalPositions[0];
			VerticalClawState.DEPLOYED.pos = verticalPositions[1];
			VerticalClawState.STOWED.pos = verticalPositions[2];
		}
	}

	public void rotateVerticalSwerve( double rotation ) {
		setVerticalPosition( getVerticalPos( ) + rotation );
	}

	// set positions
	public void setClawPos( double pos ) {
		clawServo.setPosition( pos );
	}

	public void setVerticalPosition( double pos ) {
		verticalServo.setPosition( pos );
	}

	// get positions
	public double getPos( ) {
		return clawServo.getPosition( );
	}

	public double getVerticalPos( ) {
		return verticalServo.getPosition( );
	}

	// set positions
	public void setClawState( ClawState setState ) {
		clawState = setState;
		setClawPos( setState.pos );
	}


	public void setVerticalState( VerticalClawState setState ) {
		verticalState = setState;
		setVerticalPosition( setState.pos );
	}

	public void setState( ClawState setState ) {
		clawState = setState;
		setClawPos( setState.pos );
	}


	public void setState( VerticalClawState setState ) {
		verticalState = setState;
		setVerticalPosition( setState.pos );
	}

	// misc utilities
	public void toggle( ) {
		if( clawState == ClawState.OPEN )
			setClawState( ClawState.CLOSED );
		else
			setClawState( ClawState.OPEN );
	}

	public void init( ) {
		setState( ClawState.CLOSED );
		setState( VerticalClawState.DEPLOYED );
	}


	// get states
	public ClawState getState( ) {
		return clawState;
	}

	public VerticalClawState getVerticalState( ) {
		return verticalState;
	}


}
