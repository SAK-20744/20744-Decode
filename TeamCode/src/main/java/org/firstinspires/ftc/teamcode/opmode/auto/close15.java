package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.eject;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.paths.Fast15;
import org.firstinspires.ftc.teamcode.subsystems.BallSensors2;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.Alliance;


@Config
public class close15 extends OpMode {
    Robot robot;
    Limelight limelight;
    Fast15 p;
    BallSensors2 ballSensor;
    int state = 0;
    int shootState = -1;
    ElapsedTime stateTimer = new ElapsedTime();

    public static double shootPauseTime = 0.5;

    final Alliance a;
    public close15(Alliance alliance) {
        this.a = alliance;
    }
    @Override
    public void init() {
        robot = new Robot(hardwareMap, a);
        p = new Fast15(robot);
        robot.f.setStartingPose(p.start);
        robot.k.init();
        limelight = new Limelight(hardwareMap);
        limelight.switchToShoot();
        ballSensor = new BallSensors2(hardwareMap);
        robot.k.slowed = true;
    }

    @Override
    public void init_loop() {
        if (gamepad1.x)
            robot.t.resetTurret();
        if (gamepad1.bWasPressed())
            p.fullClassifier = !p.fullClassifier;

        ballSensor.motif(limelight.motifDetection());

        telemetry.addData("Detected ID: ", limelight.motifDetection());
        telemetry.addLine();
        telemetry.addData("Turret Angle:", robot.t.getTurret());
        telemetry.addLine();
        telemetry.addData("full classifier", p.fullClassifier);
        telemetry.update();
    }

    @Override
    public void start() {
        robot.t.on();
        robot.s.close();
        robot.s.downAuto();
    }

    @Override
    public void loop() {

        switch (state) {
            case 0: robot.f.followPath(p.next()); state++; break;
            case 1: if (!robot.f.isBusy()) state++; stateTimer.reset(); break;
            case 2: if (stateTimer.seconds() > shootPauseTime) {startShoot(); state++; } break;
            case 3: if (shootState == -1) state++; break;

            // spike intake 1
            case 4: robot.i.spinIn(); state++; break;
            case 5: robot.f.followPath(p.next()); state++; break;
            case 6: if (!robot.f.isBusy()) state++; break;
            case 7: robot.f.followPath(p.next()); state++; break;
            case 8: if(robot.f.getPathCompletion()>eject) { robot.i.spinOut(); state++; } break;
            case 9: if (!robot.f.isBusy()) state++; break;
            case 10: startShoot(); state++; break;
            case 11: if (shootState == -1) state++; break;

            // gate intake 1
            case 12: robot.i.spinIn(); state++; break;
            case 13: robot.f.followPath(p.next()); state++; break;
            case 14: if (!robot.f.isBusy()) state++; break;

            case 15: stateTimer.reset(); state++; break;
            case 16: if (stateTimer.seconds() > p.gateIntakeTime) state++; break;

            case 17: robot.f.followPath(p.next()); state++; break;
            case 18: if(robot.f.getPathCompletion()>eject) { robot.i.spinOut(); state++; } break;
            case 19: if (!robot.f.isBusy()) state++; break;
            case 20: startShoot(); state++; break;
            case 21: if (shootState == -1) state++; break;

            // gate intake 2
            case 22: robot.i.spinIn(); state++; break;
            case 23: robot.f.followPath(p.next()); state++; break;
            case 24: if (!robot.f.isBusy()) state++; break;

            case 25: stateTimer.reset(); state++; break;
            case 26: if (stateTimer.seconds() > p.gateIntakeTime || p.fullClassifier) state++; break;

            case 27: robot.f.followPath(p.next()); state++; break;
            case 28: if(robot.f.getPathCompletion()>eject) { robot.i.spinOut(); state++; } break;
            case 29: if (!robot.f.isBusy()) state++; break;
            case 30: startShoot(); state++; break;
            case 31: if (shootState == -1) state++; break;

            // spike intake 2
            case 32: robot.i.spinIn(); state++; break;
            case 33: robot.f.followPath(p.next()); state++; break;
            case 34: if (!robot.f.isBusy()) state++; break;
            case 35: robot.f.followPath(p.next()); state++; break;
            case 36: if(robot.f.getPathCompletion()>eject) { robot.i.spinOut(); state++; } break;
            case 37: if (!robot.f.isBusy()) state++; break;
            case 38: startShoot(); state++; break;
            case 39: if (shootState == -1) state++; break;

//            case 40: robot.f.followPath(p.next()); state++; break;
//            case 41: if (!robot.f.isBusy()) state++; break;

        }
        robot.t.face(p.goal, robot.f.getPose());
        robot.periodic();
        sortedShoot();

        telemetry.addData("state", state);
        telemetry.addData("shootState", shootState);
        telemetry.addLine();
        telemetry.addData("shooter at target", robot.s.atTarget());
        telemetry.addData("shooter vel", robot.s.getVelocity());
        telemetry.addData("shooter target", robot.s.getTarget());
        telemetry.addLine("");
        telemetry.addData("motif", limelight.motifDetection());
        telemetry.addData("shoot order", ballSensor.shootSequence().toString());
        telemetry.update();
    }
    public void startShoot() {
        robot.s.close();
        robot.s.downAuto();
        shootState = 0;
    }
    public void shoot() {
        switch (shootState) {
            case 0: if (robot.s.atTarget()) shootState++; break;
            case 1: robot.k.kickAll(); shootState++; break;
            case 2: if (!robot.k.kickersActive()) shootState++;  break;
            case 3: shootState = -1; break;
        }
    }
    public void sortedShoot() {
        String[] shootSequence;
        switch (shootState) {
            case 0: if (robot.s.atTarget()) shootState++; break;
            case 1: ballSensor.read(); shootState++; break;
            case 2: robot.k.kickSequenced(ballSensor.shootSequence()); shootState++; break;
            case 3: if (!robot.k.kickersActive()) shootState++;  break;
            case 4: shootState = -1; break;
        }
    }
    @Override
    public void stop() {
        robot.saveEnd();
    }
}
