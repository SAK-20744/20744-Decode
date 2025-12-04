package org.firstinspires.ftc.teamcode.config;


import com.pedropathing.geometry.Pose;

public class FieldPoses {
    private static double TS = 24.0; // Tile Size
    private static double BOT_LENGTH = 17.5, BOT_WIDTH = 15;

    public static Pose redFarStart = new Pose(TS*3-BOT_WIDTH/2,BOT_LENGTH/2,Math.PI/2);
    public static Pose blueStartPos = new Pose(redFarStart.getX(),-redFarStart.getY(),-redFarStart.getHeading());

    public static Pose redHPPickup = new Pose(redFarStart.getX(), TS*3-BOT_LENGTH/2);
    public static Pose blueHPPickup = new Pose(redHPPickup.getX(), -redHPPickup.getY());


    public static Pose redHoop = new Pose(-TS*2.75,TS*2.75);
    public static Pose blueHoop = new Pose(redHoop.getX(), -redHoop.getY());




}
