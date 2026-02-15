package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.paths.Fast15;
import org.firstinspires.ftc.teamcode.util.Alliance;
@Disabled
@Autonomous
public class redClose15Test extends LinearOpMode {
    Robot r;
    Fast15 p;
    @Override
    public void runOpMode() {
        r = new Robot(hardwareMap, Alliance.RED);
        p = new Fast15(r);
        r.f.setStartingPose(p.start);
        waitForStart();
        if (opModeIsActive()) {
            r.f.followPath(p.scoreP());
            while (r.f.isBusy()) {r.periodic();}
            r.f.followPath(p.intake1());
            while (r.f.isBusy()) {r.periodic();}
            r.f.followPath(p.score1());
            while (r.f.isBusy()) {r.periodic();}
            r.f.followPath(p.gateIntake());
            while (r.f.isBusy()) {r.periodic();}
            r.f.followPath(p.scoreG());
            while (r.f.isBusy()) {r.periodic();}
            r.f.followPath(p.gateIntake());
            while (r.f.isBusy()) {r.periodic();}
            r.f.followPath(p.scoreG());
            while (r.f.isBusy()) {r.periodic();}
            r.f.followPath(p.intake2());
            while (r.f.isBusy()) {r.periodic();}
            r.f.followPath(p.score4());
            while (r.f.isBusy()) {r.periodic();}
            r.f.followPath(p.park());
            while (r.f.isBusy()) {r.periodic();}
        }
    }
}
