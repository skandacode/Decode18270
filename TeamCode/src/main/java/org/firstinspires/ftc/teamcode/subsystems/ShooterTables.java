package org.firstinspires.ftc.teamcode.subsystems;


public class ShooterTables {
    public static double getHoodPosition(double distance){
        return -0.000023472*distance*distance+0.00676915*distance+0.338142;
    }
    public static double getShooterVelocity(double distance){
        return 5.24405*distance+768.96475;
    }
}
