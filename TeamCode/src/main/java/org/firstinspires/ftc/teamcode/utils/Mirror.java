package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.TrajectorySegment;

public class Mirror {
	public static TrajectorySequence getMirroredTrajectorySequence(TrajectorySequence t,int xFlip,int yFlip) {
		for(int i=0; i<t.size(); i++) {
			getMirroredTrajectorySegment( t.get( i ),xFlip,yFlip );
		}
		return t;
	}
	public static SequenceSegment getMirroredTrajectorySegment( SequenceSegment t, int xFlip, int yFlip) {
		double startX = t.getStartPose().getX()*xFlip;
		double startY = t.getStartPose().getY()*yFlip;
		double startH = flippedAngle(t.getStartPose().getHeading(),xFlip,yFlip);

		double endX = t.getEndPose().getX()*xFlip;
		double endY = t.getEndPose().getY()*yFlip;
		double endH =flippedAngle(t.getStartPose().getHeading(),xFlip,yFlip);

		t.setStartPose( new Pose2d( startX,startY,startH  ) );
		t.setEndPose( new Pose2d( endX,endY,endH ) );
		return t;
	}
	public static double flippedAngle(double angle,int xFlip,int yFlip) {
		if(angle==0 || angle==180) {
			return angle+(180 * -((xFlip - 1) / 2));
		}
		else if(angle==270 || angle==90) {
			return angle+(180 * -((yFlip - 1) / 2));
		}
		return 0; //Change this I just need to change PR bot Code

	}
}
