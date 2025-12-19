package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.Motif;

import java.util.ArrayList;

public class ballSensors {
    Motif motif;
    RevColorSensorV3 m1,m2, l1,l2,r1,r2;
    BallColor lColor = BallColor.N, mColor = BallColor.N, rColor = BallColor.N;
    boolean left, mid, right = false;

    public ballSensors(HardwareMap h) {
        m1 = h.get(RevColorSensorV3.class, "m1");
        l1 = h.get(RevColorSensorV3.class, "l1");
        l2 = h.get(RevColorSensorV3.class, "l2");
        r1 = h.get(RevColorSensorV3.class, "r1");
        r2 = h.get(RevColorSensorV3.class, "r2");
    }

    public void motif(Motif motif) {this.motif = motif;}
    
    public void periodic() {
        if(l1.getDistance(DistanceUnit.MM) < 30 || l2.getDistance(DistanceUnit.MM) < 25)
            left=true;
        else left = false;

        if(r1.getDistance(DistanceUnit.MM) < 30 || r2.getDistance(DistanceUnit.MM) < 25)
            right=true;
        else right = false;

        if(m1.getDistance(DistanceUnit.MM) < 34)
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
        if (m == "l") {
            s1 = l1; s2 = l2;
        } else if (m == "m") {
            s1 = m1; s2 = m1;
        } else if (m == "r") {
            s1 = r1; s2 = r2;
        } else {
            s1 = l1; s2 = l2; // Default Case to Prevent Crashes and as Backup
        }
        s1R = s1.getNormalizedColors().red;
        s1G = s1.getNormalizedColors().green;
        s1B = s1.getNormalizedColors().blue;

        s2R = s2.getNormalizedColors().red;
        s2G = s2.getNormalizedColors().green;
        s2B = s2.getNormalizedColors().blue;

        if (inCRange(s1R,s1G,s1B, ApolloConstants.CS.G.l1R,ApolloConstants.CS.G.l1G,ApolloConstants.CS.G.l1B))
            s1Color = BallColor.G;
        else if (inCRange(s1R,s1G,s1B,ApolloConstants.CS.P.l1R,ApolloConstants.CS.P.l1G,ApolloConstants.CS.P.l1B))
            s1Color = BallColor.P;
        else
            s1Color = BallColor.N;

        if (inCRange(s2R,s2G,s2B, ApolloConstants.CS.G.l2R,ApolloConstants.CS.G.l2G,ApolloConstants.CS.G.l2B))
            s2Color = BallColor.G;
        else if (inCRange(s2R,s2G,s2B,ApolloConstants.CS.P.l2R,ApolloConstants.CS.P.l2G,ApolloConstants.CS.P.l2B))
            s2Color = BallColor.P;
        else
            s2Color = BallColor.N;

        if (s2Color != BallColor.N) return s2Color;
        if (s1Color != BallColor.N) return s1Color;
        return BallColor.N;
    }

    boolean inCRange(double r,double g,double b, double tr, double tg, double tb) {
        return (Math.abs(tr-r) < ApolloConstants.CS.error && Math.abs(tg-g) < ApolloConstants.CS.error && Math.abs(tb-b) < ApolloConstants.CS.error);
    }
    public String[] shootSequence() {
        ArrayList<String> sequence = new ArrayList<String>();
        BallColor expected = BallColor.N;
        if (motif == Motif.GPP) expected = BallColor.G;
        else if (motif == Motif.PGP || motif == Motif.PPG) expected = BallColor.P;
        if (lColor == expected && !sequence.contains("l")) sequence.add("l");
        if (mColor == expected && !sequence.contains("m")) sequence.add("m");
        if (rColor == expected && !sequence.contains("r")) sequence.add("r");

        if (motif == Motif.PGP) expected = BallColor.G;
        else if (motif == Motif.GPP || motif == Motif.PPG) expected = BallColor.P;
        if (lColor == expected && !sequence.contains("l")) sequence.add("l");
        if (mColor == expected && !sequence.contains("m")) sequence.add("m");
        if (rColor == expected && !sequence.contains("r")) sequence.add("r");

        if (motif == Motif.PPG) expected = BallColor.G;
        else if (motif == Motif.PGP || motif == Motif.GPP) expected = BallColor.P;
        if (lColor == expected && !sequence.contains("l")) sequence.add("l");
        if (mColor == expected && !sequence.contains("m")) sequence.add("m");
        if (rColor == expected && !sequence.contains("r")) sequence.add("r");

        return sequence.toArray(new String[0]);
    }
}
