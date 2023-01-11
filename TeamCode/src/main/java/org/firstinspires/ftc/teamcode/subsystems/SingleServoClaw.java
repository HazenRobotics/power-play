package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SingleServoClaw {

	public enum ClawState {
		OPEN, CLOSED;

		public double pos = 0;
	}

	ClawState clawState = ClawState.OPEN;

	Servo servo;

	public SingleServoClaw( HardwareMap hw ) {
		this( hw, "claw", 0, 1 );
	}

	public SingleServoClaw( HardwareMap hw, String name, double openPos, double closePos ) {
		servo = hw.servo.get( name );

		ClawState.OPEN.pos = openPos;
		ClawState.CLOSED.pos = closePos;
	}

	public void setState( ClawState state ) {
		clawState = state;
		servo.setPosition( clawState.pos );
	}

	public void close( ) {
		setState( ClawState.CLOSED );
	}

	public void open( ) {
		setState( ClawState.OPEN );
	}

	public void init( ) {
		setState( ClawState.CLOSED );
	}

	public void toggle( ) {
		if( clawState == ClawState.OPEN )
			setState( ClawState.CLOSED );
		else
			setState( ClawState.OPEN );
	}
}
