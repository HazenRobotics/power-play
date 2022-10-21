package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

	DcMotorEx motor;
	public Turret( HardwareMap hardwareMap, String motorName, boolean reverseMotor )
	{
		setup( hardwareMap, motorName, reverseMotor);
	}
	public void setup(HardwareMap hardwareMap, String motorName, boolean reverseMotor)
	{
		motor = hardwareMap.get( DcMotorEx.class, motorName );

		if( reverseMotor )
			motor.setDirection( DcMotorSimple.Direction.REVERSE );
	}

}
