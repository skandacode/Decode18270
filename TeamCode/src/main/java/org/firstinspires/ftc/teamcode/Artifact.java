package org.firstinspires.ftc.teamcode;

public enum Artifact {
    NONE(0, 0, 0),
    GREEN(0.093, 0.514, 0.393),
    PURPLE(0.2, 0.25, 0.524);

    public final double r, g, b;

    Artifact(double r, double g, double b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }
}
