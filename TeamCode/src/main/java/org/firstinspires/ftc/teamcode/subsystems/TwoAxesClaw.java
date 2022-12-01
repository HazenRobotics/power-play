package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TwoAxesClaw {

	public enum ClawState {
		OPEN, CLOSED;

		public double pos = 0;
	}

	public enum VerticalClawState {
		PICKUP, DEPLOYED, STOWED;

		public double pos = 0;
	}

	public enum HorizontalClawState {
		LEFT, CENTER, RIGHT;

		public double pos = 0;
	}

	ClawState clawState = ClawState.OPEN;
	VerticalClawState verticalState = VerticalClawState.DEPLOYED;
	HorizontalClawState horizontalState = HorizontalClawState.CENTER;

	Servo clawServo;
	Servo horizontalServo;
	Servo verticalServo;

	public TwoAxesClaw( HardwareMap hw ) {
		this( hw, "claw", "clawH", "clawV", new double[]{ 0, 0.5 }, new double[]{ 0, 0.5, 1 }, new double[]{ 0.25, 0.5, 0.75 } );
	}

	/**
	 * @param clawName            hwMap name of the opening/closing claw
	 * @param horizontalName      hwMap name of the sideways rotating claw
	 * @param verticalName        hwMap name of the vertical rotating claw
	 * @param clawPositions       the double positions of the claw (open, closed)
	 * @param horizontalPositions the double positions of the claw (left, center, right)
	 * @param verticalPositions   the double positions of the claw (pickup, deployed, stowed)
	 */
	public TwoAxesClaw( HardwareMap hw, String clawName, String horizontalName, String verticalName, double[] clawPositions, double[] horizontalPositions, double[] verticalPositions ) {
		clawServo = hw.servo.get( clawName );
		horizontalServo = hw.servo.get( horizontalName );
		verticalServo = hw.servo.get( verticalName );

		if( clawPositions.length >= 2 ) {
			ClawState.OPEN.pos = clawPositions[0];
			ClawState.CLOSED.pos = clawPositions[1];
		}

		if( horizontalPositions.length >= 3 ) {
			HorizontalClawState.LEFT.pos = horizontalPositions[0];
			HorizontalClawState.CENTER.pos = horizontalPositions[1];
			HorizontalClawState.RIGHT.pos = horizontalPositions[2];
		}

		if( verticalPositions.length >= 3 ) {
			VerticalClawState.PICKUP.pos = verticalPositions[0];
			VerticalClawState.DEPLOYED.pos = verticalPositions[1];
			VerticalClawState.STOWED.pos = verticalPositions[2];
		}
	}

	public void rotateHorizontalServo( double rotation ) {
		setHorizontalPosition( getHorizontalPos( ) + rotation );
	}

	public void rotateVerticalSwerve( double rotation ) {
		setVerticalPosition( getVerticalPos( ) + rotation );
	}

	// set positions
	public void setClawPos( double pos ) {
		clawServo.setPosition( pos );
	}

	public void setHorizontalPosition( double pos ) {
		horizontalServo.setPosition( pos );
	}

	public void setVerticalPosition( double pos ) {
		verticalServo.setPosition( pos );
	}

	// get positions
	public double getPos( ) {
		return clawServo.getPosition( );
	}

	public double getHorizontalPos( ) {
		return verticalServo.getPosition( );
	}

	public double getVerticalPos( ) {
		return verticalServo.getPosition( );
	}

	// set positions
	public void setClawState( ClawState setState ) {
		clawState = setState;
		setClawPos( setState.pos );
	}

	public void setHorizontalState( HorizontalClawState setState ) {
		horizontalState = setState;
		setHorizontalPosition( setState.pos );
	}

	public void setVerticalState( VerticalClawState setState ) {
		verticalState = setState;
		setVerticalPosition( setState.pos );
	}

	public void setState( ClawState setState ) {
		clawState = setState;
		setClawPos( setState.pos );
	}

	public void setState( HorizontalClawState setState ) {
		horizontalState = setState;
		setHorizontalPosition( setState.pos );
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
		setState( HorizontalClawState.CENTER );
		setState( VerticalClawState.DEPLOYED );
	}


	// get states
	public ClawState getState( ) {
		return clawState;
	}

	public VerticalClawState getVerticalState( ) {
		return verticalState;
	}

	public HorizontalClawState getHorizontalState( ) {
		return horizontalState;
	}


}
