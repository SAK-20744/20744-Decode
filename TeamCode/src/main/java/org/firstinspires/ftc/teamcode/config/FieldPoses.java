package org.firstinspires.ftc.teamcode.config;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class FieldPoses {
    private static double TS = 24.0; // Tile Size
    private static double BOT_LENGTH = 17.5, BOT_WIDTH = 15;
    private static double R = Math.PI/2;

    public static Pose redFarStart = new Pose(TS*-3+BOT_WIDTH/2, -BOT_LENGTH/2, R);
//    public static Pose redCloseStart = new Pose(TS*2.1, TS*2.1, -Math.PI*3/4);
    public static Pose redShooting = new Pose(TS*-3+BOT_WIDTH*0.65,-BOT_LENGTH/2, Math.PI/2);
//    public static Pose redCloseShooting = new Pose(TS*0.5,TS*0.5, -Math.PI*3/4);
    public static Pose redBall0Start = new Pose(TS*0.5, -TS*0.75, Math.PI/2);
    public static Pose redBall0End = new Pose(TS*0.5, -TS*0.75, Math.PI/2);
    public static Pose redBall1Start = new Pose(-13, -TS*0.75, Math.PI/2);
    public static Pose redBall1End = new Pose(redBall1Start.getY(), -TS*3+BOT_LENGTH/2, Math.PI/2);
    public static Pose redBall2Start = new Pose(-TS*1.6, -TS*0.75, Math.PI/2);
    public static Pose redBall2End = new Pose(redBall2Start.getY(), -TS*3+BOT_LENGTH/2, Math.PI/2);
    public static Pose redHPPickupStart = new Pose(TS*-2.5, -TS*0.75, Math.PI/2);
    public static Pose redHPPickupEnd = new Pose(TS*-2.5, -TS*3+BOT_LENGTH/2, Math.PI/2);
    public static Pose redHoop = new Pose(TS*2.75,-TS*2.75);
    public static Pose redTele = new Pose(TS*2.75,-TS*2.75);
    public static Pose redPark = new Pose(redBall2Start.getX(), -BOT_LENGTH*0.75, Math.PI/2);

    public static Pose blueFarStart = mirror(redFarStart);
    public static Pose blueShooting = mirror(redShooting);
    public static Pose blueBall1Start = mirror(redBall1Start);
    public static Pose blueBall1End = mirror(redBall1End);
    public static Pose blueBall2Start = mirror(redBall2Start);
    public static Pose blueBall2End = mirror(redBall2End);
    public static Pose blueHPPickupEnd = mirror(redHPPickupEnd);
    public static Pose blueHPPickupStart = mirror(redHPPickupStart);
    public static Pose blueHoop = mirror(redHoop);
    public static Pose blueTele = mirror(redTele);
    public static Pose bluePark = mirror(redPark);
    public static Pose teleRedHoop = new Pose(60, -60, 0);
    public static Pose teleBlueHoop = mirror(teleRedHoop);

    // Baron's retard ass doesn't have (0,0) as the center of the field so his mirror function is useless
    public static Pose mirror(Pose pose) {
        return new Pose(pose.getX(), -pose.getY(), -pose.getHeading());
    }

}
