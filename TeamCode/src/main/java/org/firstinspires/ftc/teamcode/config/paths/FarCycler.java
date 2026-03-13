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
    public Pose hpIntake = FieldPoses.redHPPickupEnd; // intake\
    public Pose park = FieldPoses.redFarPark; //new Pose(36, 12, Math.toRadians(180));
    public Pose goal = FieldPoses.redHoop;
    public Pose goalFar = FieldPoses.redHoopFar;

    public Pose intake1 = FieldPoses.redBall2End;
    public Pose intake1Ctrl = FieldPoses.redBall2Ctrl;

    public Pose cycleIntake = FieldPoses.redCyclerIntake;

    private int index;
    public static double intakeBreakStrength = 0.7;

    public FarCycler(Robot r) {
        this.f = r.f;

        if (r.a.equals(Alliance.BLUE)) {
            start = mirror(start);
            score = mirror(score);
            intake1 = mirror(intake1);
            intake1Ctrl = mirror(intake1Ctrl);
            hpIntake = mirror(hpIntake);
            cycleIntake = mirror(cycleIntake);
            park = mirror(park);
            goal = mirror(goal);
            goalFar = mirror(goalFar);
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

    public PathChain intake1() { // intake 1
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                hpIntake
                        )
                )
                .setBrakingStrength(intakeBreakStrength)
                .setLinearHeadingInterpolation(score.getHeading(), hpIntake.getHeading(), 0.3)
                .build();
    }

    public PathChain score1() {
        return f.pathBuilder()
                .addPath(new BezierCurve(hpIntake, score))
//                .setNoDeceleration()
                .setLinearHeadingInterpolation(hpIntake.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake2() { // intake 1
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                intake1Ctrl,
                                intake1
                        )
                )
                .setBrakingStrength(intakeBreakStrength)
                .setLinearHeadingInterpolation(score.getHeading(), intake1.getHeading(), 0.3)
                .build();
    }

    public PathChain score2() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake1, score))
//                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake1.getHeading(), score.getHeading())
                .build();
    }

    public PathChain cycleIntake() { // intake 1
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                cycleIntake
                        )
                )
                .setBrakingStrength(intakeBreakStrength)
                .setLinearHeadingInterpolation(score.getHeading(), cycleIntake.getHeading(), 0.3)
                .build();
    }

    public PathChain cycleScore() {
        return f.pathBuilder()
                .addPath(new BezierCurve(cycleIntake, score))
//                .setNoDeceleration()
                .setLinearHeadingInterpolation(cycleIntake.getHeading(), score.getHeading())
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
            case 1: return intake1();
            case 2: return score1();
            case 3: return intake2();
            case 4: return score2();
            case 5: return cycleIntake();
            case 6: return cycleScore();
            case 7: return cycleIntake();
            case 8: return cycleScore();
            case 9: return cycleIntake();
            case 10: return cycleScore();
            case 11: return cycleIntake();
            case 12: return cycleScore();
            case 13: return park();
            default: return null;
        }
    }

    public void reset() {
        index = 0;
    }
}