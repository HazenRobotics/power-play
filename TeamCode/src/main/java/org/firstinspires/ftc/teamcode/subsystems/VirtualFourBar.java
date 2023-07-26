package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VirtualFourBar {

	double pulsePerRevolution;
	DcMotorEx motor;
	PIDController pid;

	public VirtualFourBar( HardwareMap hw, String motorName, double ppr , boolean reversed, double[] pidValues) {
		motor = hw.get( DcMotorEx.class, motorName );
		pulsePerRevolution = ppr;

		if (reversed)
			motor.setDirection( DcMotorSimple.Direction.REVERSE );

		pid.setPID( pidValues[0], pidValues[1], pidValues[2] );
	}

}
