package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    }

    @Override
    public void init_loop() {
        if (gamepad1.x)
            r.t.resetTurret();
//        if (gamepad1.bWasPressed())
//            p.fullClassifier = !p.fullClassifier;

        bs.motif(l.motifDetection());

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
        r.s.close();r.s.down();
    }

    @Override
    public void loop() {

        switch (state) {
            case 0: r.f.followPath(p.next()); state++; break;
            case 1: if (!r.f.isBusy()) state++; break;
            case 2: startShoot(); state++; break;
            case 3: if (shootState == -1) state++; break;

            // spike intake 1
            case 4: r.i.spinIn(); state++; break;
            case 5: r.f.followPath(p.next()); state++; break;
            case 6: if (!r.f.isBusy()) state++; break;
            case 7: r.f.followPath(p.next()); state++; break;
            case 8: if (!r.f.isBusy()) state++; break;
            case 9: r.i.spinOut(); state++; break;
            case 10: startShoot(); state++; break;
            case 11: if (shootState == -1) state++; break;

            case 12: r.i.spinIn(); state++; break;
            case 13: r.f.followPath(p.next()); state++; break;
            case 14: if (!r.f.isBusy()) state++; break;
            case 15: r.f.followPath(p.next()); state++; break;
            case 16: if (!r.f.isBusy()) state++; break;
            case 17: r.i.spinOut(); state++; break;
            case 18: startShoot(); state++; break;
            case 19: if (shootState == -1) state++; break;

            case 20: r.i.spinIn(); state++; break;
            case 21: r.f.followPath(p.next()); state++; break;
            case 22: if (!r.f.isBusy()) state++; break;
            case 23: r.f.followPath(p.next()); state++; break;
            case 24: if (!r.f.isBusy()) state++; break;
            case 25: r.i.spinOut(); state++; break;
            case 26: startShoot(); state++; break;
            case 27: if (shootState == -1) state++; break;

            case 28: r.i.spinIn(); state++; break;
            case 29: r.f.followPath(p.next()); state++; break;
            case 30: if (!r.f.isBusy()) state++; break;
            case 31: r.f.followPath(p.next()); state++; break;
            case 32: if (!r.f.isBusy()) state++; break;
            case 33: r.i.spinOut(); state++; break;
            case 34: startShoot(); state++; break;
            case 35: if (shootState == -1) state++; break;

            case 36: r.i.spinIn(); state++; break;
            case 37: r.f.followPath(p.next()); state++; break;
            case 38: if (!r.f.isBusy()) state++; break;
            case 39: r.f.followPath(p.next()); state++; break;
            case 40: if (!r.f.isBusy()) state++; break;
            case 41: r.i.spinOut(); state++; break;
            case 42: startShoot(); state++; break;
            case 43: if (shootState == -1) state++; break;

            case 44: r.i.spinIn(); state++; break;
            case 45: r.f.followPath(p.next()); state++; break;
            case 46: if (!r.f.isBusy()) state++; break;
            case 47: r.f.followPath(p.next()); state++; break;
            case 48: if (!r.f.isBusy()) state++; break;
            case 49: r.i.spinOut(); state++; break;
            case 50: startShoot(); state++; break;
            case 51: if (shootState == -1) state++; break;

            case 52: r.f.followPath(p.next()); state++; break;
            case 53: if (!r.f.isBusy()) state++; break;

        }
        r.t.face(p.goal, r.f.getPose());
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
        r.s.close();r.s.down();
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
