package org.firstinspires.ftc.teamcode.config;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class FieldPoses {
    private static double TS = 24.0; // Tile Size
    private static double BOT_LENGTH = 17.5, BOT_WIDTH = 15;

    public static Pose redFarStart = new Pose(BOT_LENGTH/2, TS*-3+BOT_WIDTH/2, -Math.PI);
//    public static Pose redCloseStart = new Pose(TS*2.1, TS*2.1, -Math.PI*3/4);
    public static Pose redShooting = new Pose(BOT_LENGTH/2, TS*-3+BOT_WIDTH*0.65, -Math.PI);
//    public static Pose redCloseShooting = new Pose(TS*0.5,TS*0.5, -Math.PI*3/4);
    public static Pose redBall0Start = new Pose(TS*0.75, TS*0.5, -Math.PI);
    public static Pose redBall0End = new Pose(TS*0.75, TS*0.5, -Math.PI);
    public static Pose redBall1Start = new Pose(TS*0.75, -13, -Math.PI);
    public static Pose redBall1End = new Pose(TS*3-BOT_LENGTH/2, redBall1Start.getY(), -Math.PI);
    public static Pose redBall2Start = new Pose(TS*0.75, -TS*1.6, -Math.PI);
    public static Pose redBall2End = new Pose(TS*3-BOT_LENGTH/2, redBall2Start.getY(), -Math.PI);
    public static Pose redHPPickupStart = new Pose(TS*0.75, TS*-2.5, -Math.PI);
    public static Pose redHPPickup = new Pose(TS*3-BOT_LENGTH/2,TS*-2.5, -Math.PI);
    public static Pose redHoop = new Pose(TS*2.75,TS*2.75);
    public static Pose redTele = new Pose(TS*2.75,-TS*2.75);
    public static Pose redPark = new Pose(BOT_LENGTH*0.75, redBall2Start.getY(), -Math.PI);

    public static Pose blueFarStart = new Pose(-BOT_LENGTH/2, TS*-3+BOT_WIDTH/2, 0);
    public static Pose blueShooting = new Pose(-BOT_LENGTH/2, TS*-3+BOT_WIDTH*0.65, 0);
    public static Pose blueBall1Start = new Pose(-TS*0.75, -13, 0);
    public static Pose blueBall1End = new Pose(-TS*3+BOT_LENGTH/2, redBall1Start.getY(), 0);
    public static Pose blueBall2Start = new Pose(-TS*0.75, -TS*1.6, 0);
    public static Pose blueBall2End = new Pose(-TS*3+BOT_LENGTH/2, redBall2Start.getY(), 0);

    public static Pose blueHPPickup = new Pose(-TS*3+BOT_LENGTH/2,TS*-2.5, -Math.PI);
    public static Pose blueHPPickupStart = new Pose(blueHPPickup.getX()-15, blueHPPickup.getY(), blueHPPickup.getHeading());
    public static Pose blueHoop = new Pose(TS*2.75,TS*2.75);
    public static Pose blueTele = new Pose(TS*2.75, -TS*2.75);
    public static Pose bluePark = new Pose(-BOT_LENGTH*0.75, redBall2Start.getY(), 0);
//
    public static Pose teleRedHoop = new Pose(60, 60, 0);
    public static Pose teleBlueHoop = new Pose(-60, 60, 0);


}
