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
public class FarCycler {
    private final Follower f;

    public Pose start = FieldPoses.redCyclerStart;
    public Pose score = FieldPoses.redCyclerScore;
    public Pose hpintake = FieldPoses.redHPPickupEnd; // intake\
    public Pose park = FieldPoses.redFarPark; //new Pose(36, 12, Math.toRadians(180));
    public Pose goal = FieldPoses.redHoop;

    private int index;
    public static double intakeBreakStrength = 5;

    public FarCycler(Robot r) {
        this.f = r.f;

        if (r.a.equals(Alliance.BLUE)) {
            start = mirror(start);
            score = mirror(score);
            park = mirror(park);
            goal = mirror(goal);
        }

        index = 0;
    }

    public PathChain start() { // intake 1
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                start,
                                score
                        )
                )
                .setBrakingStrength(intakeBreakStrength)
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake() { // intake 1
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                hpintake
                        )
                )
                .setBrakingStrength(intakeBreakStrength)
                .setLinearHeadingInterpolation(score.getHeading(), hpintake.getHeading(), 0.3)
                .build();
    }

    public PathChain score() {
        return f.pathBuilder()
                .addPath(new BezierCurve(hpintake, score))
//                .setNoDeceleration()
                .setLinearHeadingInterpolation(hpintake.getHeading(), score.getHeading())
                .build();
    }


    public PathChain park() {
        return f.pathBuilder()
                .addPath(new BezierLine(score, park))
                .setLinearHeadingInterpolation(score.getHeading(), park.getHeading())
                .build();
    }

    public PathChain next() {
        switch (index++) {
            case 0: return start();
            case 1: return intake();
            case 2: return score();
            case 3: return intake();
            case 4: return score();
            case 5: return intake();
            case 6: return score();
            case 7: return intake();
            case 8: return score();
            case 9: return intake();
            case 10: return score();
            case 11: return intake();
            case 12: return score();
            case 13: return park();
            default: return null;
        }
    }

    public void reset() {
        index = 0;
    }
}