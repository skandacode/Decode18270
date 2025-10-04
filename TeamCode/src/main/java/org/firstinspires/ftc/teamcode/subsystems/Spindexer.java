package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Artifact;

import solverslib.hardware.ServoEx;

public class Spindexer implements Subsystem{
    ServoEx spin1, spin2;

    private Artifact[] artifactPositions;
    private int currentIndex = 0;

    public enum SpindexerPositions{
        INTAKE1(0.038),
        INTAKE2(0.322),
        INTAKE3(0.606),
        SHOOT1(0.18),
        SHOOT2(0.464),
        SHOOT3(0.748);

        public final double pos;

        SpindexerPositions(double pos) {
            this.pos = pos;
        }
    }


    public Spindexer(HardwareMap hardwareMap, Artifact[] startingPositions) {
        spin1 = new ServoEx(hardwareMap, "spin1");
        spin2 = new ServoEx(hardwareMap, "spin2");
        artifactPositions = startingPositions;
    }
    public Spindexer(HardwareMap hardwareMap) {
        this(hardwareMap, new Artifact[]{Artifact.NONE, Artifact.NONE, Artifact.NONE});
    }

    public Artifact[] getArtifactPositions() {
        return artifactPositions;
    }

    public void setPositions(SpindexerPositions position) {
        spin1.setPosition(position.pos);
        spin2.setPosition(position.pos);
    }

    public void shootPos(int index) {
        if (index < 0 || index >= 3) return;
        currentIndex = index;
        if (index == 0) {
            setPositions(SpindexerPositions.SHOOT1);
        } else if (index == 1) {
            setPositions(SpindexerPositions.SHOOT2);
        } else {
            setPositions(SpindexerPositions.SHOOT3);
        }
    }
    public void afterShoot(){
        artifactPositions[currentIndex] = Artifact.NONE;
    }

    public void intakePos(int index) {
        if (index < 0 || index >= 3) return;
        currentIndex = index;
        if (index == 0) {
            setPositions(SpindexerPositions.INTAKE1);
        } else if (index == 1) {
            setPositions(SpindexerPositions.INTAKE2);
        } else {
            setPositions(SpindexerPositions.INTAKE3);
        }
    }
    public void afterIntake(Artifact artifact) {
        artifactPositions[currentIndex] = artifact;
    }

    @Override
    public void update() {
        spin1.update();
        spin2.update();
    }
}
