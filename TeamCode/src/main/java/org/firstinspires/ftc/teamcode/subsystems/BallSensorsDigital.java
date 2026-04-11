package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.FileConfig;
import org.firstinspires.ftc.teamcode.util.Motif;
import org.firstinspires.ftc.teamcode.util.Pattern;
import org.opencv.core.Scalar;

public class BallSensorsDigital {
    Motif motif = Motif.GPP ;
    public DigitalChannel m1, l1,r1;
    public DigitalChannel m2, l2,r2;

    BallColor lColor = BallColor.N, mColor = BallColor.N, rColor = BallColor.N;
    boolean left, mid, right = false;

    ApolloConstants.colorRanges colorRanges;

    public BallSensorsDigital(HardwareMap h) {
        m1 = h.digitalChannel.get("bm1");
        m2 = h.digitalChannel.get("bm2");
        l1 = h.digitalChannel.get("bl1");
        l2 = h.digitalChannel.get("bl2");
        r1 = h.digitalChannel.get("br1");
        r2 = h.digitalChannel.get("br2");
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
        left = l1.getState();
        mid = m1.getState();
        right = r1.getState();
        lColor = sense(l1,l2);
        mColor = sense(m1,m2);
        rColor = sense(r1,r2);
    }
    public BallColor leftC() {return lColor;}
    public BallColor middleC() {return mColor;}
    public BallColor rightC() {return rColor;}

    public boolean leftD() {return left;}
    public boolean middleD() {return mid;}
    public boolean rightD() {return right;}
    public BallColor sense(DigitalChannel p0, DigitalChannel p1) {
        BallColor s1Color = BallColor.N;

//        if (p0.getState()) s1Color = BallColor.P;
        if (p1.getState()) s1Color = BallColor.G;

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
