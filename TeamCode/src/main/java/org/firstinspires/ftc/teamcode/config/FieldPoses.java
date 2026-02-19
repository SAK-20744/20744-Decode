package org.firstinspires.ftc.teamcode.config;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class FieldPoses {
    private static double TS = 24.0; // Tile Size
    private static double BOT_LENGTH = 17.5, BOT_WIDTH = 18;
    private static double R = Math.PI/2;

    public static Pose redFarStart = new Pose(TS*-3+BOT_WIDTH/2, -BOT_LENGTH/2, -R);
    public static Pose redCyclerStart = new Pose(TS*-3+BOT_WIDTH/2,-16,-R);
    public static Pose redCyclerScore = new Pose(TS*-3+BOT_WIDTH/2 + 4,-18,-R);
    public static Pose redCloseStart = new Pose(49, -54.5, rads(-140));
    public static Pose redFarScore = new Pose(TS*-3+BOT_WIDTH*0.65,-BOT_LENGTH/2, -R);
    public static Pose redCloseScore = new Pose (6, -16, -R);

    public static Pose redPushFar = new Pose(redFarStart.getX(), -36, -R);

    public static Pose redBall0Ctrl = new Pose(TS*0.3, TS*-0.375, -R);
    public static Pose redBall0Start = new Pose(TS*0.3, -TS*0.75, -R);
    public static Pose redBall0End = new Pose(TS*0.3, -TS*2.4, -R);

    public static Pose redBall1Ctrl = new Pose(-13, TS*-0.375, -R);
    public static Pose redBall1Start = new Pose(-13, -TS*0.75, -R);
    public static Pose redBall1End = new Pose(redBall1Start.getX(), -TS*2.5, -R);

    public static Pose redBall2Ctrl = new Pose(-TS*1.7, TS*-0.25, -R);
    public static Pose redBall2Start = new Pose(-TS*1.7, -TS*0.75, -R);
    public static Pose redBall2End = new Pose(redBall2Start.getX(), -TS*2.5 , -R);

    public static Pose redGatePickup = new Pose(TS*-0.5, -TS*2.625, -R*0.75);

    public static Pose redGateOpen = new Pose(TS*-0.4, -TS*2.4, -R);

    public static Pose redHPPickupStart = new Pose(TS*-2.5, -TS*0.75, -R);
    public static Pose redHPPickupEnd = new Pose(TS*-3+BOT_WIDTH/2, -TS*3+BOT_LENGTH/2, -R);
    public static Pose redHoop = new Pose(TS*3,-TS*3);
    public static Pose redTele = new Pose(TS*2.75,-TS*2.75);
    public static Pose redClosePark = new Pose(-16, -16, -R);
    public static Pose redFarPark = new Pose(-36, -16, -R);
    public static Pose redPark = new Pose(redBall2Start.getX(), -BOT_LENGTH*0.75, -R);

    public static Pose blueFarStart = mirror(redFarStart);
    public static Pose blueShooting = mirror(redFarScore);
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
    public static double rads(double degrees) {return Math.toRadians(degrees);}
}
