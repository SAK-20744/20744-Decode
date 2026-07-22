package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.eject;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.paths.Fast15;
import org.firstinspires.ftc.teamcode.config.paths.Sorted9;
import org.firstinspires.ftc.teamcode.subsystems.BallSensors2;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.Alliance;


@Config
public class close9 extends OpMode {
    Robot r;
    Limelight l;
    Sorted9 p;
    BallSensors2 bs;
    int state = 0;
    int shootState = -1;
    ElapsedTime stateTimer = new ElapsedTime();

    public static double shootPauseTime = 2;

    final Alliance a;
    public close9(Alliance alliance) {
        this.a = alliance;
    }
    @Override
    public void init() {
        r = new Robot(hardwareMap, a);
        p = new Sorted9(r);
        r.f.setStartingPose(p.start);
        r.k.init();
        l = new Limelight(hardwareMap);
        l.switchToShoot();
        bs = new BallSensors2(hardwareMap);
        r.k.slowed = true;
    }

    @Override
    public void init_loop() {
        if (gamepad1.x)
            r.t.resetTurret();
        if (gamepad1.bWasPressed())
            p.extraGateOpen = !p.extraGateOpen;

        bs.motif(l.motifDetection());
        ApolloConstants.motif = l.motifDetection();

        telemetry.addData("Detected ID: ", l.motifDetection());
        telemetry.addLine();
        telemetry.addData("Turret Angle:", r.t.getTurret());
        telemetry.update();
    }

    @Override
    public void start() {
        r.t.on();
//        r.s.close();r.s.downAuto();
    }

    @Override
    public void loop() {

        switch (state) {
            case 0: r.f.followPath(p.next()); state++; break;
            case 1: if (!r.f.isBusy()) { state++; stateTimer.reset(); } break;
            case 2: if (stateTimer.seconds() > shootPauseTime) {startShoot(); state++; } break;
            case 3: if (shootState == -1) state++; break;

            // spike intake 1
            case 4: r.i.spinIn(); state++; break;
            case 5: r.f.followPath(p.next()); state++; break;
            case 6: if (!r.f.isBusy()) state++; break;
            case 7: r.f.followPath(p.next()); state++; break;
            case 8: if(r.f.getPathCompletion()>eject) { r.i.spinOut(); state++; } break;
            case 9: if (!r.f.isBusy()) { state++; stateTimer.reset(); } break;
            case 10: if (stateTimer.seconds() > shootPauseTime) {startShoot(); state++; } break;
            case 11: if (shootState == -1) state++; break;

            // spike intake 2
            case 12: r.i.spinIn(); state++; break;
            case 13: r.f.followPath(p.next()); state++; break;
            case 14: if (!r.f.isBusy()) state++; break;
            case 15: r.f.followPath(p.next()); state++; break;
            case 16: if(r.f.getPathCompletion()>eject) { r.i.spinOut(); state++; } break;
            case 17: if (!r.f.isBusy()) { state++; stateTimer.reset(); } break;
            case 18: if (stateTimer.seconds() > shootPauseTime) {startShoot(); state++; } break;
            case 19: if (shootState == -1) state++; break;

            case 20: r.f.followPath(p.next()); state++; break;
            case 21: if (!r.f.isBusy()) state++; break;

//            case 40: r.f.followPath(p.next()); state++; break;
//            case 41: if (!r.f.isBusy()) state++; break;

        }
        r.t.face(p.goal, r.f.getPose());
        r.s.adaptive(r.f.getPose().distanceFrom(p.goal));
        r.periodic();
        sortedShoot();
        telemetry.addLine(" --Ball Colors");
        telemetry.addData("L",bs.leftC());
        telemetry.addData("M",bs.middleC());
        telemetry.addData("R",bs.rightC());
        telemetry.addData("heading", r.f.getHeading());
        telemetry.addData("pose", r.f.getPose());
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
