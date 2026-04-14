package org.firstinspires.ftc.teamcode.util;

public enum Motif {
    GPP(0),
    PGP(1),
    PPG(2);

    final int id;

    Motif(int id) {
        this.id = id;
    }

    public int id() {
        return id;
    }
}
