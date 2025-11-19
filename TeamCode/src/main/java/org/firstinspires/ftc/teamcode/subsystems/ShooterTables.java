package org.firstinspires.ftc.teamcode.subsystems;


public class ShooterTables {
    public static double getHoodPosition(double distance){
        return 0.850804 / (1 + Math.exp(-(0.0456316 * distance - 1.20445)));
    }
    public static double getShooterVelocity(double distance){
        return 5.56579*distance+835.1988;
    }
}
