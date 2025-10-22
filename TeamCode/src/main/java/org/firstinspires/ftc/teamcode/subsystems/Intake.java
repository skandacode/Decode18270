package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Artifact;

import java.util.Arrays;

import solverslib.hardware.motors.Motor;

@Configurable
public class Intake implements Subsystem {
    private Motor intakeMotor;
    private double power = 0.0;

    private RevColorSensorV3 colorSensor;

    public static int read_times = 10;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = new Motor(hardwareMap, "intake");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "intake_color_sensor");
    }

    public void setPower(double power) {
        this.power = power;
    }
    @Override
    public void update() {
        intakeMotor.set(-power);
        intakeMotor.update();
    }

    public double getIntakeDistance(){
        return colorSensor.getDistance(DistanceUnit.INCH);
    }
    public double[] getColor(){
        int[] raw = new int[]{colorSensor.red(), colorSensor.green(), colorSensor.blue()};
        double sum = raw[0]+raw[1]+raw[2];
        return new double[]{raw[0]/sum, raw[1]/sum, raw[2]/sum};
    }

    public boolean isIntaked(){
        return getIntakeDistance() < 1.5;
    }
    public Artifact getArtifact(){
        if(!isIntaked()) return Artifact.NONE;

        double[] colors = new double[3];
        for (int i = 0; i < read_times; i++) {
            double [] sample = getColor();
            colors[0] += sample[0];
            colors[1] += sample[1];
            colors[2] += sample[2];
        }
        colors[0] /= read_times;
        colors[1] /= read_times;
        colors[2] /= read_times;

        System.out.println("Artifact color "+Arrays.toString(colors));

        double dist_purp =
                (Artifact.PURPLE.r-colors[0])*(Artifact.PURPLE.r-colors[0])
                +(Artifact.PURPLE.g-colors[1])*(Artifact.PURPLE.g-colors[1])
                +(Artifact.PURPLE.b-colors[2])*(Artifact.PURPLE.b-colors[2]);
        double dist_green =
                (Artifact.GREEN.r-colors[0])*(Artifact.GREEN.r-colors[0])
                +(Artifact.GREEN.g-colors[1])*(Artifact.GREEN.g-colors[1])
                +(Artifact.GREEN.b-colors[2])*(Artifact.GREEN.b-colors[2]);
        System.out.println("Artifact color "+dist_purp+"   "+dist_green);
        if(dist_green < dist_purp) return Artifact.GREEN;
        else return Artifact.PURPLE;

    }
}
