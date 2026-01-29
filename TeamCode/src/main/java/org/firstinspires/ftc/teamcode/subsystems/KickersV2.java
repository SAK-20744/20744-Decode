package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.KDOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.KUP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.LKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.LKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.MKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.MKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.RKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.RKICKER_UP;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.ApolloHardwareNames;

import java.util.ArrayList;

public class KickersV2 {

    public enum Kicker {
        L,M,R
    }

    Servo lKicker, mKicker, rKicker;
    double kickerUpTime = KUP, kickerDownTime = KDOWN; //Time kicker waits till going back down and time before next kicker can go up
    ElapsedTime kickerTimer = new ElapsedTime();
    Kicker currentUp = null, queuedUp = null;
    ArrayList<Kicker> queue = new ArrayList<Kicker>();
    public KickersV2(HardwareMap hardwareMap) {
        lKicker = hardwareMap.servo.get(ApolloHardwareNames.lKicker);
        mKicker = hardwareMap.servo.get(ApolloHardwareNames.mKicker);
        rKicker = hardwareMap.servo.get(ApolloHardwareNames.rKicker);
    }
    public void init() {
        lKicker.setPosition(LKICKER_DOWN);
        mKicker.setPosition(MKICKER_DOWN);
        rKicker.setPosition(RKICKER_DOWN);
    }
    public void kick(Kicker kicker) {
        if (queue.size() >= 2) return;
        queue.add(kicker);
        queuedUp = kicker;
    }
    public void periodic() {
        if (currentUp == null && !queue.isEmpty()) {
            kickerTimer.reset();
//            currentUp = queuedUp;
//            queuedUp = null;
            currentUp = queue.get(0);
            queue.remove(0);
        }
        if (currentUp != null)
            switch (currentUp) {
            case L: updateKicker(lKicker,LKICKER_UP,LKICKER_DOWN); break;
            case M: updateKicker(mKicker,MKICKER_UP,MKICKER_DOWN); break;
            case R: updateKicker(rKicker,RKICKER_UP,RKICKER_DOWN); break;
        }
    }
    public Kicker getUp() {return currentUp;}
    public Kicker getQueued() {return queuedUp;}
    public double kickerTime() {return kickerTimer.milliseconds();}
    public boolean kickerUp() {return kickerTime() < kickerUpTime;}
    public boolean kickerDown() {return kickerTime() < kickerDownTime+kickerUpTime;}
    private void updateKicker(Servo servo, double upPos, double downPos) {
        if (kickerUp())           servo.setPosition(upPos);
        else if (kickerDown())    servo.setPosition(downPos);
        else                      currentUp = null;
    }
}
