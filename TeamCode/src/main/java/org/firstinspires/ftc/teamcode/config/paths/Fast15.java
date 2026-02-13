package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.FieldPoses;
import static org.firstinspires.ftc.teamcode.config.FieldPoses.mirror;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

public class Fast15 {
    private final Follower f;

    public Pose start = FieldPoses.redCloseStart;
    public Pose score = FieldPoses.redCloseScore; // score
    public Pose intake1 = FieldPoses.redBall1End; // intake\
    public Pose intake1Control = FieldPoses.redBall1Ctrl;
    public Pose intake2 = FieldPoses.redBall0End; // intake
    public Pose intake2Control = FieldPoses.redBall0Ctrl;
    public Pose gate = FieldPoses.redGatePickup; //new Pose(144-132.781509, 61, Math.toRadians(28+90)); // gate
    public Pose gateControl = FieldPoses.redBall0Ctrl; //62);
    public Pose park = FieldPoses.redPark; //new Pose(36, 12, Math.toRadians(180));

    private int index;

    public Fast15(Robot r) {
        this.f = r.f;

        if (r.a.equals(Alliance.BLUE)) {
            start = mirror(start);
            score = mirror(score);
            intake1 = mirror(intake1);
            intake1Control = mirror(intake1Control);
            intake2 = mirror(intake2);
            intake2Control = mirror(intake2Control);
            gate = mirror(gate);
            gateControl = mirror(gateControl);
            park = mirror(park);
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
                .setNoDeceleration()
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
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(score.getHeading(), intake1.getHeading(), 0.3)
                .build();
    }

    public PathChain score1() {
        return f.pathBuilder()
                .addPath(new BezierLine(intake1, score))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
                .build();
    }

    public PathChain gateIntake() { // go to gate from intake1
        return f.pathBuilder()
                .addPath(new BezierCurve(score, gateControl, gate))
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(score.getHeading(), gate.getHeading(), 0.3)
                .build();
    }

    public PathChain score2() {
        return f.pathBuilder()
                .addPath(new BezierLine(gate, score))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
                .build();
    }

    public PathChain score3() {
        return f.pathBuilder()
                .addPath(new BezierCurve(gate, score))
                .setNoDeceleration()
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
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(score.getHeading(), intake2.getHeading(), 0.3)
                .build();
    }

    public PathChain score4() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake2, score))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake2.getHeading(), score.getHeading())
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
            case 4: return score2();
            case 5: return gateIntake();
            case 6: return score3();
            case 7: return intake2();
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