package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.SequenceSegment;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySegment;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

public class Mirror {
	public static TrajectorySequence Flip(TrajectorySequence t,boolean xFlip,boolean yFlip) {
		ArrayList<Pose2d> start = new ArrayList<>();
		ArrayList<Pose2d> end = new ArrayList<>();
		for(int i=0; i<t.getSize(); i++) {
			double startX = t.get( i ).getStartPose().getX();
			double startY = t.get( i ).getStartPose().getY();
			double endX = t.get( i ).getEndPose().getX();
			double endY = t.get( i ).getEndPose().getY();
			//You do not need to change heading -Camden Harris

			if(xFlip) {

				startX=-startX;
				endX=-endX;
			}
			if(yFlip) {
				startY=-startY;
				endY=-endY;
			}
			start.add(  new Pose2d( startX,startY,t.get( i ).getStartPose().getHeading() ));
			end.add(  new Pose2d( startX,startY,t.get( i ).getStartPose().getHeading() ));
		}

		return new TrajectorySequence(  );
	}

}
