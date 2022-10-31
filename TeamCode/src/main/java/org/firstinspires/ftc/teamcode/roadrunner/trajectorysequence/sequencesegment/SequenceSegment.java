package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.List;

public abstract class SequenceSegment {
    double duration;
    Pose2d startPose;
    Pose2d endPose;
    List<TrajectoryMarker> markers;

    protected SequenceSegment(
            double duration,
            Pose2d startPose, Pose2d endPose,
            List<TrajectoryMarker> markers
    ) {
        this.duration = duration;
        this.startPose = startPose;
        this.endPose = endPose;
        this.markers = markers;
    }

    public double getDuration() {
        return this.duration;
    }

    public Pose2d getStartPose() {
        return startPose;
    }
    public Pose2d getEndPose() {
        return endPose;
    }

    public void setEndPose( Pose2d endPose ) {
        this.endPose = endPose;
    }
    public void setStartPose( Pose2d startPose ) {
        this.startPose = startPose;
    }


    public List<TrajectoryMarker> getMarkers() {
        return markers;
    }
}
