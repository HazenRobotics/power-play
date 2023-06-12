package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;


public abstract class SubSystem {
	HardwareMap hw;
	public SubSystem(HardwareMap h) {
		hw=h;
	}
	// Returns all important data from subsystem
	abstract Object[] getData();
	//test out system
	abstract void test();


}
