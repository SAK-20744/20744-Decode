package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.paths.FarCycler;
import org.firstinspires.ftc.teamcode.config.paths.Fast15;
import org.firstinspires.ftc.teamcode.subsystems.BallSensors;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous
public class redFarCycler extends OpMode {
    Robot r;
    Limelight l;
    FarCycler p;
    BallSensors bs;
    int state = 0;
    int shootState = -1;
    ElapsedTime stateTimer = new ElapsedTime();
    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.RED);
        p = new FarCycler(r);
        r.f.setStartingPose(p.start);
        r.k.init();
        l = new Limelight(hardwareMap);
        l.switchToShoot();
        bs = new BallSensors(hardwareMap);
        r.k.slowed = true;
    }

    @Override
    public void init_loop() {
        if (gamepad1.x)
            r.t.resetTurret();
//        if (gamepad1.bWasPressed())
//            p.fullClassifier = !p.fullClassifier;

        bs.motif(l.motifDetection());
        ApolloConstants.motif = l.motifDetection();

        telemetry.addData("Detected ID: ", l.motifDetection());
        telemetry.addLine();
        telemetry.addData("Turret Angle:", r.t.getTurret());
        telemetry.addLine();
//        telemetry.addData("full classifier", p.fullClassifier);
        telemetry.update();
    }

    @Override
    public void start() {
        r.t.on();
    }

    @Override
    public void loop() {

        switch (state) {
            case 0: r.f.holdPoint(p.score); startShoot(); state++; break;
            case 1: if (shootState == -1) state++; break;

            // spike intake 1
            case 2: r.i.spinIn(); state++; break;
            case 3: r.f.followPath(p.next()); state++; break;
            case 4: if (!r.f.isBusy()) state++; break;
            case 5: r.f.followPath(p.next()); state++; break;
            case 6: if (!r.f.isBusy()) state++; break;
            case 7: r.i.spinOut(); state++; break;
            case 8: startShoot(); state++; break;
            case 9: if (shootState == -1) state++; break;

            case 10: r.i.spinIn(); state++; break;
            case 11: r.f.followPath(p.next()); state++; break;
            case 12: if (!r.f.isBusy()) state++; break;
            case 13: r.f.followPath(p.next()); state++; break;
            case 14: if (!r.f.isBusy()) state++; break;
            case 15: r.i.spinOut(); state++; break;
            case 16: startShoot(); state++; break;
            case 17: if (shootState == -1) state++; break;

            case 18: r.i.spinIn(); state++; break;
            case 19: r.f.followPath(p.next()); state++; break;
            case 20: if (!r.f.isBusy()) state++; break;
            case 21: r.f.followPath(p.next()); state++; break;
            case 22: if (!r.f.isBusy()) state++; break;
            case 23: r.i.spinOut(); state++; break;
            case 24: startShoot(); state++; break;
            case 25: if (shootState == -1) state++; break;

            case 26: r.i.spinIn(); state++; break;
            case 27: r.f.followPath(p.next()); state++; break;
            case 28: if (!r.f.isBusy()) state++; break;
            case 29: r.f.followPath(p.next()); state++; break;
            case 30: if (!r.f.isBusy()) state++; break;
            case 31: r.i.spinOut(); state++; break;
            case 32: startShoot(); state++; break;
            case 33: if (shootState == -1) state++; break;

            case 34: r.i.spinIn(); state++; break;
            case 35: r.f.followPath(p.next()); state++; break;
            case 36: if (!r.f.isBusy()) state++; break;
            case 37: r.f.followPath(p.next()); state++; break;
            case 38: if (!r.f.isBusy()) state++; break;
            case 39: r.i.spinOut(); state++; break;
            case 40: startShoot(); state++; break;
            case 41: if (shootState == -1) state++; break;

            case 42: r.i.spinIn(); state++; break;
            case 43: r.f.followPath(p.next()); state++; break;
            case 44: if (!r.f.isBusy()) state++; break;
            case 45: r.f.followPath(p.next()); state++; break;
            case 46: if (!r.f.isBusy()) state++; break;
            case 47: r.i.spinOut(); state++; break;
            case 48: startShoot(); state++; break;
            case 49: if (shootState == -1) state++; break;

            case 50: r.f.followPath(p.next()); state++; break;
            case 51: if (!r.f.isBusy()) state++; break;

        }
        Pose goal = p.goal;
        if (r.s.isFar) goal = p.goalFar;
        r.t.face(goal, r.f.getPose());
        r.s.adaptive(r.f.getPose().distanceFrom(p.goal));
        r.periodic();
        sortedShoot();

        telemetry.addData("state", state);
        telemetry.addData("shootState", shootState);
        telemetry.addLine();
        telemetry.addData("shooter at target", r.s.atTarget());
        telemetry.addData("shooter vel", r.s.getVelocity());
        telemetry.addData("shooter target", r.s.getTarget());
        telemetry.addLine("");
        telemetry.addData("motif", l.motifDetection());
        telemetry.addData("shoot order", bs.shootSequence().toString());
        telemetry.update();
    }
    public void startShoot() {
//        r.s.far();r.s.up();
        shootState = 0;
    }
    public void shoot() {
        switch (shootState) {
            case 0: if (r.s.atTarget()) shootState++; break;
            case 1: r.k.kickAll(); shootState++; break;
            case 2: if (!r.k.kickersActive()) shootState++;  break;
            case 3: shootState = -1; break;
        }
    }
    public void sortedShoot() {
        String[] shootSequence;
        switch (shootState) {
            case 0: if (r.s.atTarget()) shootState++; break;
            case 1: bs.read(); shootState++; break;
            case 2: r.k.kickSequenced(bs.shootSequence()); shootState++; break;
            case 3: if (!r.k.kickersActive()) shootState++;  break;
            case 4: shootState = -1; break;
        }
    }
    @Override
    public void stop() {
        r.saveEnd();
    }
}
