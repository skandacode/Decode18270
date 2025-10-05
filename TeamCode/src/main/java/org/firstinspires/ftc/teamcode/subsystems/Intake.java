package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Artifact;

import solverslib.hardware.motors.Motor;

public class Intake implements Subsystem {
    private Motor intakeMotor;
    private double power = 0.0;

    private RevColorSensorV3 colorSensor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = new Motor(hardwareMap, "intake_motor");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "intake_color_sensor");
    }

    public void setPower(double power) {
        this.power = power;
    }
    @Override
    public void update() {
        intakeMotor.set(power);
        intakeMotor.update();
    }

    public double getIntakeDistance(){
        return colorSensor.getDistance(DistanceUnit.INCH);
    }
    public int[] getColor(){
        return new int[]{colorSensor.red(), colorSensor.green(), colorSensor.blue()};
    }

    public boolean isIntaked(){
        return getIntakeDistance() < 3.0;
    }
    public Artifact getArtifact(){
        if(!isIntaked()) return Artifact.NONE;

        int[] colors = new int[3];
        for (int i = 0; i < 10; i++) {
            int[] sample = getColor();
            colors[0] += sample[0];
            colors[1] += sample[1];
            colors[2] += sample[2];
        }
        colors[0] /= 10;
        colors[1] /= 10;
        colors[2] /= 10;

        double dist_purp =
                (Artifact.PURPLE.r-colors[0])*(Artifact.PURPLE.r-colors[0])
                +(Artifact.PURPLE.g-colors[1])*(Artifact.PURPLE.g-colors[1])
                +(Artifact.PURPLE.b-colors[2])*(Artifact.PURPLE.b-colors[2]);
        double dist_green =
                (Artifact.GREEN.r-colors[0])*(Artifact.GREEN.r-colors[0])
                +(Artifact.GREEN.g-colors[1])*(Artifact.GREEN.g-colors[1])
                +(Artifact.GREEN.b-colors[2])*(Artifact.GREEN.b-colors[2]);

        if(dist_green < dist_purp) return Artifact.GREEN;
        else return Artifact.PURPLE;

    }
}
