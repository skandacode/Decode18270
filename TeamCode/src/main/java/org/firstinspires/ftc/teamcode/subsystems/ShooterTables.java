package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTables {
    public static double time = 1;

    public static double getHoodPosition(double distance){
        return 0.813747 / (1 + Math.exp(-(0.0564885 * distance - 1.11718)));
    }

    public static double getShooterVelocity(double distance) {
        return 0.000240449  * Math.pow(distance, 3)
                -0.0632857     * Math.pow(distance, 2)
                +9.65368      * distance
                +734.47049;
    }
    public static double getAirTime(double distance){
        return time;
    }
}
