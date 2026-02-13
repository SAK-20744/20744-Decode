package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.paths.Fast15;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp
public class redClose15 extends OpMode {
    Robot r;
    Fast15 p;
    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.RED);
        p = new Fast15(r);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if (r.f.isBusy()) {

        } else {
            r.f.followPath(p.next());
        }
        r.periodic();
    }

    @Override
    public void stop() {
        r.saveEnd();
    }
}
