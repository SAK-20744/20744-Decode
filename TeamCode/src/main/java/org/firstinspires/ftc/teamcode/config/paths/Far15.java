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
public class Far15 {
    private final Follower f;

    public Pose start = FieldPoses.redFarStart;
    public Pose score = FieldPoses.redFarScore; // score
    public Pose closeScore = FieldPoses.redCloseScore;
    public Pose intake1 = FieldPoses.redPushFar;
    public Pose hpIntakePos = FieldPoses.redHPPickupEnd;
    public Pose intake2 = FieldPoses.redBall1End;
    public Pose intake2Control = FieldPoses.redBall1Ctrl;
    public Pose intake3 = FieldPoses.redBall2End; // intake
    public Pose intake3Control = FieldPoses.redBall2Ctrl;
    public Pose intake4 = FieldPoses.redBall0End; // intake
    public Pose intake4Control = FieldPoses.redBall0Ctrl;
    public Pose gate = FieldPoses.redGateOpen; //new Pose(144-132.781509, 61, Math.toRadians(28+90)); // gate
    public Pose gateControl = FieldPoses.redGateCtrl; //62);
    public Pose park = FieldPoses.redClosePark; //new Pose(36, 12, Math.toRadians(180));
    public Pose goal = FieldPoses.redHoop;

    private int index;

    public static double intakeBreakStrength = 1;
    public static double gateIntakeTime = 2;

    public static boolean intakeHP = false;

    public Far15(Robot r) {
        this.f = r.f;

        if (r.a.equals(Alliance.BLUE)) {
            start = mirror(start);
            score = mirror(score);
            closeScore = mirror(closeScore);
            intake1 = mirror(intake1);
            hpIntakePos = mirror(hpIntakePos);
            intake2 = mirror(intake2);
            intake2Control = mirror(intake2Control);
            intake3 = mirror(intake3);
            intake3Control = mirror(intake3Control);
            intake4 = mirror(intake4);
            intake4Control = mirror(intake4Control);
            gate = mirror(gate);
            gateControl = mirror(gateControl);
            park = mirror(park);
            goal = mirror(goal);
        }

        index = 0;
    }
    public void intakeHP(boolean ihp) {
        this.intakeHP = ihp;
        if (ihp) {
            intake1 = hpIntakePos;
        }
    }
    public PathChain intake1() { // intake 1
        return f.pathBuilder()
                .addPath(
                        new BezierLine(start, intake1)
                )
                .setBrakingStrength(intakeBreakStrength)
                .setLinearHeadingInterpolation(start.getHeading(), intake1.getHeading(), 0.3)
                .build();
    }

    public PathChain score1() {
        return f.pathBuilder()
                .addPath(new BezierLine(intake1, score))
                .setBrakingStrength(0.6)
//                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake1.getHeading(), score.getHeading())
                .build();
    }

    public PathChain gateIntake() { // go to gate from intake1
        return f.pathBuilder()
                .addPath(new BezierCurve(score, gateControl, gate))
                .setBrakingStrength(intakeBreakStrength)
                .setLinearHeadingInterpolation(score.getHeading(), gate.getHeading(), 0.3)
                .build();
    }

    public PathChain scoreG() {
        return f.pathBuilder()
                .addPath(new BezierCurve(gate, gateControl, score))
                .setBrakingStrength(0.6)
//                .setNoDeceleration()
                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake2() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score, intake2Control, intake2))
                .setBrakingStrength(intakeBreakStrength)
                .setLinearHeadingInterpolation(score.getHeading(), intake2.getHeading(), 0.3)
                .build();
    }
    public PathChain openGate() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake2, intake2Control, gate))
                .setBrakingStrength(intakeBreakStrength)
                .setLinearHeadingInterpolation(intake2.getHeading(), gate.getHeading())
                .build();
    }
    public PathChain score2() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake2, intake2Control, closeScore))
//                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake2.getHeading(), closeScore.getHeading())
                .build();
    }

    public PathChain intake3() {
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                closeScore,
                                intake3Control,
                                intake3
                        )
                )
                .setBrakingStrength(intakeBreakStrength)
                .setLinearHeadingInterpolation(closeScore.getHeading(), intake3.getHeading(), 0.3)
                .build();
    }

    public PathChain score3() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake3, intake3Control, closeScore))
//                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake3.getHeading(), closeScore.getHeading())
                .build();
    }
    public PathChain intake4() {
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                closeScore,
                                intake4Control,
                                intake4
                        )
                )
                .setBrakingStrength(intakeBreakStrength)
                .setLinearHeadingInterpolation(closeScore.getHeading(), intake4.getHeading(), 0.3)
                .build();
    }

    public PathChain score4() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake4, intake4Control, closeScore))
//                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake4.getHeading(), closeScore.getHeading())
                .build();
    }

    public PathChain park() {
        return f.pathBuilder()
                .addPath(new BezierLine(closeScore, park))
                .setLinearHeadingInterpolation(closeScore.getHeading(), park.getHeading())
                .build();
    }

    public PathChain next() {
        switch (index++) {
            case 0: return intake1();
            case 1: return score1();
            case 2: return intake2();
            case 3: return openGate();
            case 4: return score2();
            case 5: return intake3();
            case 6: return score3();
            case 7: return intake4();
            case 8: return score4();
            case 9: return park();
            default: return null;
        }
    }

    public boolean hasNext() {
        int PATH_COUNT = 8;
        return index < PATH_COUNT;
    }

    public void reset() {
        index = 0;
    }
}