package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Configurable
public class DrivetrainTest extends LinearOpMode {
    Drivetrain drivetrain;

    public static double forward = 0.0;
    public static double strafe = 0.0;
    public static double turn = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            drivetrain.driveRobotCentric(forward, strafe, turn);
            drivetrain.update();
        }
    }
}
