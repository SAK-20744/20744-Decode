package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.config.ApolloConstants.CS;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.FileConfig;
import org.firstinspires.ftc.teamcode.util.Motif;
import org.firstinspires.ftc.teamcode.util.Pattern;
import org.opencv.core.Scalar;

public class BallSensors2 {
    Motif motif = Motif.GPP ;
    public RevColorSensorV3 m1, l1,r1;
    BallColor lColor = BallColor.N, mColor = BallColor.N, rColor = BallColor.N;
    boolean left, mid, right = false;

    ApolloConstants.colorRanges colorRanges;

    public BallSensors2(HardwareMap h) {
        m1 = h.get(RevColorSensorV3.class, "m1");
        l1 = h.get(RevColorSensorV3.class, "l1");
        r1 = h.get(RevColorSensorV3.class, "r1");

        l1.setGain(75);
        m1.setGain(515);
        r1.setGain(500);

        colorRanges = FileConfig.loadJson("ColorSensorRanges.json",ApolloConstants.colorRanges.class);
        if (colorRanges == null) colorRanges = new ApolloConstants.colorRanges();
    }

    public void motif(Motif motif) {this.motif = motif;}

    public void motif(Pattern pattern) {
        switch (pattern) {
            case GPP21: motif = Motif.GPP;break;
            case PGP22: motif = Motif.PGP;break;
            case PPG23: motif = Motif.PPG;break;
        }
    }

    public void read() {
        if(l1.getDistance(DistanceUnit.MM) < 150)
            left=true;
        else left = false;

        if(r1.getDistance(DistanceUnit.MM) < 100)
            right=true;
        else right = false;

        if(m1.getDistance(DistanceUnit.MM) < 37)
            mid=true;
        else mid = false;

        lColor = sense("l");
        mColor = sense("m");
        rColor = sense("r");
    }
    public BallColor leftC() {return lColor;}
    public BallColor middleC() {return mColor;}
    public BallColor rightC() {return rColor;}

    public boolean leftD() {return left;}
    public boolean middleD() {return mid;}
    public boolean rightD() {return right;}
    public BallColor sense(String m) {
        RevColorSensorV3 s1, s2;
        double s1R,s1G,s1B,s2R,s2G,s2B;
        BallColor s1Color = BallColor.N, s2Color = BallColor.N;
        Scalar min, max;
        if (m == "l") {
            s1 = l1;
            min = colorRanges.lGMin;
            max = colorRanges.lGMax;
        } else if (m == "m") {
            s1 = m1;
            min = colorRanges.mGMin;
            max = colorRanges.mGMax;
//
        } else if (m == "r") {
            s1 = r1;
            min = colorRanges.rGMin;
            max = colorRanges.rGMax;
//
        } else {
            s1 = l1;
            min = colorRanges.lGMin;
            max = colorRanges.lGMax;
            // Default Case to Prevent Crashes and as Backup
        }
        s1R = s1.getNormalizedColors().red;
        s1G = s1.getNormalizedColors().green;
        s1B = s1.getNormalizedColors().blue;

//        s2R = s2.getNormalizedColors().red;
//        s2G = s2.getNormalizedColors().green;
//        s2B = s2.getNormalizedColors().blue;

        if (inCRange(s1R,s1G,s1B, min,max))
            s1Color = BallColor.G;
//        else if (inCRange(s1R,s1G,s1B,p1R,p1G,p1B, range))
//            s1Color = BallColor.P;
        else
            s1Color = BallColor.N;

//        if (inCRange(s2R,s2G,s2B, CS.G.l2R,CS.G.l2G,CS.G.l2B))
//            s2Color = BallColor.G;
//        else if (inCRange(s2R,s2G,s2B,CS.P.l2R,CS.P.l2G,CS.P.l2B))
//            s2Color = BallColor.P;
//        else
//            s2Color = BallColor.N;

//        if (s2Color != BallColor.N) return s2Color;
//        if (s1Color != BallColor.N) return s1Color;
//        return BallColor.N;
        return s1Color;
    }

//    boolean inCRange(double r,double g,double b, double tr, double tg, double tb, double range) {
//        return (Math.abs(tr-r) < range && Math.abs(tg-g) < range && Math.abs(tb-b) < range);
//    }
    boolean inCRange(double r,double g,double b, Scalar min, Scalar max) {
        return (r>min.val[0] && r<max.val[0]  &&  g>min.val[1] && g<max.val[1]  &&  b>min.val[2] && b<max.val[2]);
    }

    public String[] shootSequence() {
        if (lColor == BallColor.G) {
            switch (motif) {
                case GPP: return new String[]{"l","m","r"};
                case PGP: return new String[]{"r","l","m"};
                case PPG: return new String[]{"r","m","l"};
            }
        }
        else if (rColor == BallColor.G) {
            switch (motif) {
                case GPP: return new String[]{"r","m","l"};
                case PGP: return new String[]{"l","r","m"};
                case PPG: return new String[]{"l","m","r"};
            }
        }
        else{
            switch (motif) {
                case GPP: return new String[]{"m","r","l"};
                case PGP: return new String[]{"l","m","r"};
                case PPG: return new String[]{"l","r","m"};
            }
        }
        return new String[]{"l","m","r"};
    }
}
