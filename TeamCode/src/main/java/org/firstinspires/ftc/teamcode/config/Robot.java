package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.KDOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.KUP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.config.*;

import java.lang.reflect.Field;

public class Robot {
    public final Intake i;
    public final Limelight l;
    public final Shooter s;
    public final Turret t;
    public final Follower f;
    public Alliance a;

    private final LynxModule hub;
    private final Timer loop = new Timer();

    public static Pose endPose;
//    public static Pose defaultPose = new Pose(8+24,6.25+24,0);
    public static Pose defaultPose = new Pose(0,0,0);
    public static Pose shootTarget = new Pose(6, 144-6, 0);

    public Robot(HardwareMap h, Alliance a) {
        this.a = a;
        i = new Intake(h);
        l = new Limelight(h, a);
        s = new Shooter(h);
        t = new Turret(h);
        f = Constants.createFollower(h);

        hub = h.getAll(LynxModule.class).get(0);
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        loop.resetTimer();
        setShootTarget();

    }

    public void periodic() {
//        setShootTarget();

        if (loop.getElapsedTime() % 5 == 0) {
            hub.clearBulkCache();
        }

        f.update();
        t.periodic();
        s.periodic();
    }

    public void stop() {
        endPose = f.getPose();
    }


    public void setShootTarget() {
        if (a == Alliance.BLUE/* && shootTarget.getX() != 6*/)
//            shootTarget = new Pose(6, 144 - 6, 0);
//            shootTarget = FieldPoses.blueHoop;
            shootTarget = FieldPoses.teleBlueHoop;
        else if (a == Alliance.RED/* && shootTarget.getX() != (144 - 6)*/)
//            shootTarget = shootTarget.mirror();
//            shootTarget = FieldPoses.redHoop;
            shootTarget = FieldPoses.teleRedHoop;
    }

    public Pose getShootTarget() {
        return shootTarget;
    }

    public static class auto {
        Robot r;
        ElapsedTime kTimer = new ElapsedTime();
        cursedKicker kickers;
        public void init(Robot robot) {
            r = robot;
        }
        public void lKick (){
            kTimer.reset();
            kickers.lKickerUp(); kTimer.reset();
            while(kTimer.milliseconds()<KUP) {r.periodic();}
            kickers.lKickerDown();  kTimer.reset();
            while(kTimer.milliseconds()<KDOWN) {r.periodic();}
        }
        public void mKick (){
            kTimer.reset();
            kickers.mKickerUp(); kTimer.reset();
            while(kTimer.milliseconds()<KUP) {r.periodic();}
            kickers.mKickerDown(); kTimer.reset();
            while(kTimer.milliseconds()<KDOWN) {r.periodic();}
        }
        public void rKick (){
            kTimer.reset();
            kickers.rKickerUp(); kTimer.reset();
            while(kTimer.milliseconds()<KUP) {r.periodic();}
            kickers.rKickerDown(); kTimer.reset();
            while(kTimer.milliseconds()<KDOWN) {r.periodic();}
        }
    }
}