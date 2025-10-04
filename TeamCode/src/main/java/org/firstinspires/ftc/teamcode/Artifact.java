package org.firstinspires.ftc.teamcode;

public enum Artifact {
    NONE(0, 0, 0),
    GREEN(0, 255, 0),
    PURPLE(128, 0, 128);

    public final int r, g, b;

    Artifact(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }
}
