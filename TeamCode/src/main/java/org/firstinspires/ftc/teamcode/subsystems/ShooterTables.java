package org.firstinspires.ftc.teamcode.subsystems;


public class ShooterTables {
    public static double getHoodPosition(double distance){
        return 0.850804 / (1 + Math.exp(-(0.0456316 * distance - 1.20445)));
    }
    public static double getShooterVelocity(double distance){
        return 0.0000215834*Math.pow(distance, 4)
                - 0.00934763*Math.pow(distance, 3)
                + 1.4205*Math.pow(distance, 2)
                - 83.13285*distance
                + 2740.41945;}
}
