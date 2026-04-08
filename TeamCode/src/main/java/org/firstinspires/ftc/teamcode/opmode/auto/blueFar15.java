package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.eject;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.config.FieldPoses;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.paths.Far15;
import org.firstinspires.ftc.teamcode.subsystems.BallSensors;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous
public class blueFar15 extends OpMode {
    Robot r;
    Limelight l;
    Far15 p;
    BallSensors bs;
    int state = 0;
    int shootState = -1;
    ElapsedTime stateTimer = new ElapsedTime();
    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        p = new Far15(r);
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
        if (gamepad1.bWasPressed())
            p.intakeHP(!p.intakeHP);

        bs.motif(l.motifDetection());
        ApolloConstants.motif = l.motifDetection();

        telemetry.addData("Detected ID: ", l.motifDetection());
        telemetry.addLine();
        telemetry.addData("Turret Angle:", r.t.getTurret());
        telemetry.addLine();
        telemetry.addData("intake hp", p.intakeHP);
        telemetry.update();
    }

    @Override
    public void start() {
        r.t.on();
        r.s.far();r.s.up();
    }

    @Override
    public void loop() {

        switch (state) {
            case 0: r.f.holdPoint(p.start); startShoot(); state++; break;
            case 1: if (shootState == -1) state++; break;

            // spike intake 1
            case 2: r.i.spinIn(); state++; break;
            case 3: r.f.followPath(p.next()); state++; break;
            case 4: if (!r.f.isBusy()) state++; break;
            case 5: r.f.followPath(p.next()); state++; break;
            case 6: if(r.f.getPathCompletion()>eject) { r.i.spinOut(); state++; } break;
            case 7: if (!r.f.isBusy()) state++; break;
            case 8: startShoot(); state++; break;
            case 9: if (shootState == -1) state++; break;

            //
            case 10: r.i.spinIn(); state++; break;
            case 11: r.f.followPath(p.next()); state++; break;
            case 12: if (!r.f.isBusy()) state++; break;
            case 13: r.f.followPath(p.next()); state++; break;
            case 14: if(r.f.getPathCompletion()>eject) { r.i.spinOut(); state++; } break;
            case 15: if (!r.f.isBusy()) state++; break;
            case 16: r.f.followPath(p.next()); state++; break;
            case 17: if (!r.f.isBusy()) state++; break;
            case 18: startCloseShoot(); state++; break;
            case 19: if (shootState == -1) state++; break;

            //
            case 20: r.i.spinIn(); state++; break;
            case 21: r.f.followPath(p.next()); state++; break;
            case 22: if (!r.f.isBusy()) state++; break;
            case 23: r.f.followPath(p.next()); state++; break;
            case 24: if(r.f.getPathCompletion()>eject) { r.i.spinOut(); state++; } break;
            case 25: if (!r.f.isBusy()) state++; break;
            case 26: startCloseShoot(); state++; break;
            case 27: if (shootState == -1) state++; break;

            // spike intake 2
            case 28: r.i.spinIn(); state++; break;
            case 29: r.f.followPath(p.next()); state++; break;
            case 30: if (!r.f.isBusy()) state++; break;
            case 31: r.f.followPath(p.next()); state++; break;
            case 32: if(r.f.getPathCompletion()>eject) { r.i.spinOut(); state++; } break;
            case 33: if (!r.f.isBusy()) state++; break;
            case 34: startCloseShoot(); state++; break;
            case 35: if (shootState == -1) state++; break;

            case 36: r.f.followPath(p.next()); state++; break;
            case 37: if (!r.f.isBusy()) state++; break;

        }
        Pose goal = p.goal;
        if (r.s.isFar) goal = p.goalFar;
        r.t.face(goal, r.f.getPose());
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
        r.s.far();r.s.up();
        shootState = 0;
    }
    public void startCloseShoot() {
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
