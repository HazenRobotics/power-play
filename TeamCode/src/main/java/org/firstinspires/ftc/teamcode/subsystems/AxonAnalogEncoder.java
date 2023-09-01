package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AxonAnalogEncoder {

	public final static double tau = 2 * Math.PI;
	AnalogInput encoder;
	double angularOffset, maxVoltage;
	boolean inverted;

	public AxonAnalogEncoder( HardwareMap hw ) {
		this(hw, "absoluteEncoder");
	}

	public AxonAnalogEncoder( HardwareMap hw, String encoderName ) {
		this(hw, encoderName, 0, 2.3, false);
	}

	public AxonAnalogEncoder( HardwareMap hw, String encoderName, double offset, double volt, boolean invert ) {
		encoder = hw.analogInput.get( encoderName );
		angularOffset = offset;
		maxVoltage = volt;
		inverted = invert;
	}

	public void invert() {
		inverted = !inverted;
	}

	public double getAngle() {
		return ((((encoder.getVoltage() / maxVoltage) * tau ) + angularOffset + tau) % tau);
	}

}
