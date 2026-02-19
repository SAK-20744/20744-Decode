package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.config.ApolloConstants.CS;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.Motif;
import org.firstinspires.ftc.teamcode.util.Pattern;

public class BallSensorsBroke {
    Motif motif = Motif.GPP ;
    RevColorSensorV3 m1, l1,r1;
    BallColor lColor = BallColor.N, mColor = BallColor.N, rColor = BallColor.N;
    boolean left, mid, right = false;

    public BallSensorsBroke(HardwareMap h) {
        m1 = h.get(RevColorSensorV3.class, "m1");
        l1 = h.get(RevColorSensorV3.class, "l1");
        r1 = h.get(RevColorSensorV3.class, "r1");

        l1.setGain(75);
        m1.setGain(515);
        r1.setGain(500);
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
        if(l1.getDistance(DistanceUnit.MM) < 30)
            left=true;
        else left = false;

        if(r1.getDistance(DistanceUnit.MM) < 30)
            right=true;
        else right = false;

        if(m1.getDistance(DistanceUnit.MM) < 34)
            mid=true;
        else mid = false;
        
//        lColor = sense("l");
//        mColor = sense("m");
//        rColor = sense("r");
        double l1G = l1.getNormalizedColors().green / ApolloConstants.CS.G.l1G;
        double m1G = m1.getNormalizedColors().green / ApolloConstants.CS.G.m1G;
        double r1G = r1.getNormalizedColors().green / ApolloConstants.CS.G.r1G;
        double gMax = Math.max(Math.max(l1G, m1G), r1G);
        if (gMax == r1G) { rColor = BallColor.G; lColor = BallColor.N; mColor = BallColor.N; }
        else if (gMax == m1G) { mColor = BallColor.G; lColor = BallColor.N; rColor = BallColor.N; }
        else { lColor = BallColor.G; mColor = BallColor.N; rColor = BallColor.N; }

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
        double p1R, p1G, p1B;
        double g1R, g1G, g1B;
        double range;
        if (m == "l") {
            s1 = l1;
            p1R = CS.P.l1R; p1G = CS.P.l1G; p1B = CS.P.l1B;
            g1R = CS.G.l1R; g1G = CS.G.l1G; g1B = CS.G.l1B;
            range = CS.l1Range;
        } else if (m == "m") {
            s1 = m1;
            p1R = CS.P.m1R; p1G = CS.P.m1G; p1B = CS.P.m1B;
            g1R = CS.G.m1R; g1G = CS.G.m1G; g1B = CS.G.m1B;
            range = CS.m1Range;
        } else if (m == "r") {
            s1 = r1;
            p1R = CS.P.r1R; p1G = CS.P.r1G; p1B = CS.P.r1B;
            g1R = CS.G.r1R; g1G = CS.G.r1G; g1B = CS.G.r1B;
            range = CS.r1Range;
        } else {
            s1 = l1;
            p1R = CS.P.l1R; p1G = CS.P.l1G; p1B = CS.P.l1B;
            g1R = CS.G.l1R; g1G = CS.G.l1G; g1B = CS.G.l1B;
            range = CS.l1Range;
            // Default Case to Prevent Crashes and as Backup
        }
        s1R = s1.getNormalizedColors().red;
        s1G = s1.getNormalizedColors().green;
        s1B = s1.getNormalizedColors().blue;

//        s2R = s2.getNormalizedColors().red;
//        s2G = s2.getNormalizedColors().green;
//        s2B = s2.getNormalizedColors().blue;

        if (inCRange(s1R,s1G,s1B, g1R,g1G,g1B, range))
            s1Color = BallColor.G;
        else if (inCRange(s1R,s1G,s1B,p1R,p1G,p1B, range))
            s1Color = BallColor.P;
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

    boolean inCRange(double r,double g,double b, double tr, double tg, double tb, double range) {
        return (Math.abs(tr-r) < range && Math.abs(tg-g) < range && Math.abs(tb-b) < range);
    }
//    public String[] shootSequence() { // THIS IS THE SHITTIEST THING EVER ITS JUST A PROTOTYPE
//        ArrayList<String> sequence = new ArrayList<String>();
//        BallColor expected = BallColor.N;
//        if (motif == Motif.GPP) expected = BallColor.G;
//        else if (motif == Motif.PGP || motif == Motif.PPG) expected = BallColor.P;
//        if (lColor == expected && !sequence.contains("l")) sequence.add("l");
//        if (mColor == expected && !sequence.contains("m")) sequence.add("m");
//        if (rColor == expected && !sequence.contains("r")) sequence.add("r");
//
//        if (motif == Motif.PGP) expected = BallColor.G;
//        else if (motif == Motif.GPP || motif == Motif.PPG) expected = BallColor.P;
//        if (lColor == expected && !sequence.contains("l")) sequence.add("l");
//        if (mColor == expected && !sequence.contains("m")) sequence.add("m");
//        if (rColor == expected && !sequence.contains("r")) sequence.add("r");
//
//        if (motif == Motif.PPG) expected = BallColor.G;
//        else if (motif == Motif.PGP || motif == Motif.GPP) expected = BallColor.P;
//        if (lColor == expected && !sequence.contains("l")) sequence.add("l");
//        if (mColor == expected && !sequence.contains("m")) sequence.add("m");
//        if (rColor == expected && !sequence.contains("r")) sequence.add("r");
//
//        return sequence.toArray(new String[0]);
//    }
    public String[] shootSequence() {
        if (lColor == BallColor.G) {
            switch (motif) {
                case GPP: return new String[]{"l","m","r"};
                case PGP: return new String[]{"r","l","m"};
                case PPG: return new String[]{"r","m","l"};
            }
        }
        if (rColor == BallColor.G) {
            switch (motif) {
                case GPP: return new String[]{"r","m","l"};
                case PGP: return new String[]{"l","r","m"};
                case PPG: return new String[]{"l","m","r"};
            }
        }
        switch (motif) {
            case GPP: return new String[]{"m","r","l"};
            case PGP: return new String[]{"l","m","r"};
            case PPG: return new String[]{"l","r","m"};
        }
        return new String[]{"l","m","r"};
    }
//    public KOrder shootSequence() {
//        if (lColor == BallColor.G) {
//            switch (motif) {
//                case GPP: return new String[]{"l","m","r"};
//                case PGP: return new String[]{"r","l","m"};
//                case PPG: return new String[]{"r","m","l"};
//            }
//        }
//        if (rColor == BallColor.G) {
//            switch (motif) {
//                case GPP: return new String[]{"r","m","l"};
//                case PGP: return new String[]{"l","r","m"};
//                case PPG: return new String[]{"l","m","r"};
//            }
//        }
//        switch (motif) {
//            case GPP: return new String[]{"m","r","l"};
//            case PGP: return new String[]{"l","m","r"};
//            case PPG: return new String[]{"l","r","m"};
//        }
//        return new String[]{"l","m","r"};
//    }
}
