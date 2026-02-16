package org.firstinspires.ftc.teamcode.config.paths;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.FieldPoses;
import static org.firstinspires.ftc.teamcode.config.FieldPoses.mirror;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Config
public class Fast15 {
    private final Follower f;

    public Pose start = FieldPoses.redCloseStart;
    public Pose score = FieldPoses.redCloseScore; // score
    public Pose intake1 = FieldPoses.redBall1End; // intake\
    public Pose intake1Control = FieldPoses.redBall1Ctrl;
    public Pose intake2 = FieldPoses.redBall0End; // intake
    public Pose intake2Control = FieldPoses.redBall0Ctrl;
    public Pose intake3 = FieldPoses.redBall2End;
    public Pose intake3Control = FieldPoses.redBall2Ctrl;
    public Pose gate = FieldPoses.redGatePickup; //new Pose(144-132.781509, 61, Math.toRadians(28+90)); // gate
    public Pose gateControl = FieldPoses.redBall1Ctrl; //62);
    public Pose park = FieldPoses.redClosePark; //new Pose(36, 12, Math.toRadians(180));
    public Pose goal = FieldPoses.redHoop;

    private int index;

    public static double intakeBreakStrength = 1;
    public static double gateIntakeTime = 2;

    public static boolean fullClassifier = false;

    public Fast15(Robot r) {
        this.f = r.f;

        if (r.a.equals(Alliance.BLUE)) {
            start = mirror(start);
            score = mirror(score);
            intake1 = mirror(intake1);
            intake1Control = mirror(intake1Control);
            intake2 = mirror(intake2);
            intake2Control = mirror(intake2Control);
            intake3 = mirror(intake3);
            intake3Control = mirror(intake3Control);
            gate = mirror(gate);
            gateControl = mirror(gateControl);
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
                                score
                        )
                )
//                .setNoDeceleration()
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake1() { // intake 1
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                intake1Control,
                                intake1
                        )
                )
                .setBrakingStrength(intakeBreakStrength)
                .setLinearHeadingInterpolation(score.getHeading(), intake1.getHeading(), 0.3)
                .build();
    }

    public PathChain score1() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake1, intake1Control, score))
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
//                .setNoDeceleration()
                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake2() {
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                intake2Control,
                                intake2
                        )
                )
                .setBrakingStrength(intakeBreakStrength)
                .setLinearHeadingInterpolation(score.getHeading(), intake2.getHeading(), 0.3)
                .build();
    }

    public PathChain score2() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake2, score))
//                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake2.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake3() {
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                intake3Control,
                                intake3
                        )
                )
                .setBrakingStrength(intakeBreakStrength)
                .setLinearHeadingInterpolation(score.getHeading(), intake3.getHeading(), 0.3)
                .build();
    }

    public PathChain score3() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake3, score))
//                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake3.getHeading(), score.getHeading())
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
            case 0: return scoreP();
            case 1: return intake1();
            case 2: return score1();
            case 3: return gateIntake();
            case 4: return scoreG();
            case 5: if (fullClassifier) return intake2(); else return gateIntake();
            case 6: if (fullClassifier) return score2(); else return scoreG();
            case 7: if (fullClassifier) return intake3(); else return intake2();
            case 8: if (fullClassifier) return score3(); else return score2();
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