package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.*;
import org.firstinspires.ftc.teamcode.config.ApolloHardwareNames;
public class Kickers {
    Servo lKicker, mKicker, rKicker;
    double kickerUpTime = 0.1, kickerDownTime = 0.1; //Time kicker waits till going back down and time before next kicker can go up
    ElapsedTime kickerTimer = new ElapsedTime();
    Kicker currentUp = null, queuedUp = null;
    public Kickers(HardwareMap hardwareMap) {
        lKicker = hardwareMap.servo.get(ApolloHardwareNames.lKicker);
        mKicker = hardwareMap.servo.get(ApolloHardwareNames.mKicker);
        rKicker = hardwareMap.servo.get(ApolloHardwareNames.rKicker);

    }
    public void kick(Kicker kicker) {queuedUp = kicker;}
    public void update() {
        if (currentUp == null && queuedUp != null) {
            kickerTimer.reset();
            currentUp = queuedUp;
            queuedUp = null;
        }
        switch (currentUp) {
            case L: updateKicker(lKicker,LKICKER_UP,LKICKER_DOWN); break;
            case M: updateKicker(mKicker,MKICKER_UP,MKICKER_DOWN); break;
            case R: updateKicker(rKicker,RKICKER_UP,RKICKER_DOWN); break;
        }
    }
    public Kicker getUp() {return currentUp;}
    public Kicker getQueued() {return currentUp;}
    public double kickerTime() {return kickerTimer.seconds();}
    public boolean kickerUp() {return kickerTime() < kickerUpTime;}
    public boolean kickerDown() {return kickerTime() < kickerDownTime;}
    private void updateKicker(Servo servo, double upPos, double downPos) {
        if (kickerUp())           servo.setPosition(upPos);
        else if (kickerDown())    servo.setPosition(downPos);
        else                      currentUp = null;
    }
}
enum Kicker {
    L,M,R
}
