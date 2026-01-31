package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

@Autonomous
public class Tutorial extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        //START_POS, END_POS
        // Drive > Movement state
        // Shoot > attempt to score artifact
    }

    PathState pathstate;

    private final Pose startPose = new Pose(15.112183353437878,129.75633293124247,Math.toRadians(135));
    private final Pose shootPose = new Pose(40.29915560916767,102.31121833534378,Math.toRadians(135));

    private PathChain driveFromStartPosToShootPos;

    public void buildPaths(){
        // Put cords for starting pose and put cords in end pose
        driveFromStartPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
