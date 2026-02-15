package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.FieldPoses;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.paths.Fast15;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous
public class redClose15 extends OpMode {
    Robot r;
    Fast15 p;
    int state = 0;
    int shootState = -1;
    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.RED);
        p = new Fast15(r);
        r.f.setStartingPose(p.start);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        r.s.close();r.s.down();
    }

    @Override
    public void loop() {

        switch (state) {
            case 0: r.f.followPath(p.next()); state++; break;
            case 1: if (!r.f.isBusy()) state++; break;
            case 2: startShoot(); state++; break;
            case 3: if (shootState != -1) shoot(); else state++; break;

            case 4: r.i.spinIn(); state++;
            case 5: r.f.followPath(p.next()); state++;
            case 6: if (!r.f.isBusy()) state++; break;
            case 7: r.f.followPath(p.next()); state++;
            case 8: if (!r.f.isBusy()) state++; break;
            case 9: r.i.spinOut(); state++; break;
            case 10: startShoot(); state++; break;
            case 11: if (shootState != -1) shoot(); else state++; break;

            case 12: r.i.spinIn(); state++;
            case 13: r.f.followPath(p.next()); state++;
            case 14: if (!r.f.isBusy()) state++; break;
            case 15: r.f.followPath(p.next()); state++;
            case 16: if (!r.f.isBusy()) state++; break;
            case 17: r.i.spinOut(); state++; break;
            case 18: startShoot(); state++; break;
            case 19: if (shootState != -1) shoot(); else state++; break;

            case 20: r.i.spinIn(); state++;
            case 21: r.f.followPath(p.next()); state++;
            case 22: if (!r.f.isBusy()) state++; break;
            case 23: r.f.followPath(p.next()); state++;
            case 24: if (!r.f.isBusy()) state++; break;
            case 25: r.i.spinOut(); state++; break;
            case 26: startShoot(); state++; break;
            case 27: if (shootState != -1) shoot(); else state++; break;

            case 28: r.i.spinIn(); state++;
            case 29: r.f.followPath(p.next()); state++;
            case 30: if (!r.f.isBusy()) state++; break;
            case 31: r.f.followPath(p.next()); state++;
            case 32: if (!r.f.isBusy()) state++; break;
            case 33: r.i.spinOut(); state++; break;
            case 34: startShoot(); state++; break;
            case 35: if (shootState != -1) shoot(); else state++; break;

            case 36: r.f.followPath(p.next()); state++;
            case 37: if (!r.f.isBusy()) state++; break;
            case 38: stop();

        }
        r.t.face(p.goal, r.f.getPose());
        r.periodic();
    }
    public void startShoot() {
        r.s.close();r.s.down();
        shootState = 0;
    }
    public void shoot() {
        switch (shootState) {
            case 0: if (r.s.atTarget()) state++; break;
            case 1: r.k.kickAll(); state++; break;
            case 2: if (!r.k.kickersActive()) state++;  break;
            case 3: shootState = -1; break;
        }
    }
    @Override
    public void stop() {
        r.saveEnd();
    }
}
