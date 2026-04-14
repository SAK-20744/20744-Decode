package org.firstinspires.ftc.teamcode.subsystems;

public enum Kicker {
    NULL(-1),
    LEFT(0),
    MIDDLE(1),
    RIGHT(2);

    final int id;

    Kicker(int id) {
        this.id = id;
    }

    public int id() {
        return id;
    }

    public static Kicker withId(int id) {
        if (id == LEFT.id()) return LEFT;
        if (id == MIDDLE.id()) return MIDDLE;
        if (id == RIGHT.id()) return RIGHT;
        if (id == NULL.id()) return NULL;
        throw new RuntimeException("No object found with id " + id);
    }
}