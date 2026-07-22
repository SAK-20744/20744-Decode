package org.firstinspires.ftc.teamcode.config.paths;

import static org.firstinspires.ftc.teamcode.config.FieldPoses.mirror;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.FieldPoses;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Config
public class Just3 {
    private final Follower f;

    public Pose start = FieldPoses.redCloseStart;
    public Pose park = FieldPoses.redClosePark; //new Pose(36, 12, Math.toRadians(180));
    public Pose goal = FieldPoses.redHoop;

    private int index;

    public static double intakeBreakStrength = 0.7;
    public static double gateIntakeBreakStrength = 0.5;
    public static double gateIntakeTime = 1.5;

    public static boolean extraGateOpen = false;

    public Just3(Robot r) {
        this.f = r.f;

        if (r.a.equals(Alliance.BLUE)) {
            start = mirror(start);
            park = mirror(park);
            goal = mirror(goal);
        }

        index = 0;
    }

    public PathChain scoreP() {
        return f.pathBuilder()
                .addPath(
                        new BezierLine(
                                start,
                                park
                        )
                )
//                .setNoDeceleration()
                .setLinearHeadingInterpolation(start.getHeading(), park.getHeading())
                .build();
    }



    public PathChain next() {
        switch (index++) {
            case 0: return scoreP();
            default: return null;
        }
    }

    public boolean hasNext() {
        int PATH_COUNT = 1;
        return index < PATH_COUNT;
    }

    public void reset() {
        index = 0;
    }
}