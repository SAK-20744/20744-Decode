package org.firstinspires.ftc.teamcode.config;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class FieldPoses {
    private static double TS = 24.0; // Tile Size
    private static double BOT_LENGTH = 17.5, BOT_WIDTH = 15;

    public static Pose redFarStart = new Pose(BOT_LENGTH/2, TS*-3+BOT_WIDTH/2, -Math.PI);
    public static Pose shooting = new Pose(BOT_LENGTH/2, TS*-3+BOT_WIDTH*0.65, -Math.PI);
    public static Pose redBall1Start = new Pose(TS*0.75, -13, -Math.PI);
    public static Pose redBall1End = new Pose(TS*3-BOT_LENGTH/2, redBall1Start.getY(), -Math.PI);
    public static Pose redBall2Start = new Pose(TS*0.75, -TS*1.6, -Math.PI);
    public static Pose redBall2End = new Pose(TS*3-BOT_LENGTH/2, redBall2Start.getY(), -Math.PI);
    public static Pose redHoop = new Pose(-TS*2.75,TS*2.75);
    public static Pose park = new Pose(BOT_LENGTH*0.75, redBall2Start.getY(), -Math.PI);


}
