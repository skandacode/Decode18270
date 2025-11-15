package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Position;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;

@TeleOp
@Configurable
public class ShooterTestTrack extends LinearOpMode {
    Shooter shooter;
    Follower follower;
    public static Shooter.Goal target = Shooter.Goal.BLUE;
    public static double hoodPosition = 0.9;
    public static double shooterVelocity = 1500;
    public static boolean enableAutoRanging = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        shooter = new Shooter(hardwareMap);
        follower = createFollower(hardwareMap);
        follower.setStartingPose(Position.pose);
        waitForStart();
        long t1 = System.nanoTime();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()){
                follower.setPose(new Pose(63, 0, Math.toRadians(180)));
            }
            follower.updatePose();
            telemetry.addData("Angle and distance:", Arrays.toString(shooter.getAngleDistance(follower.getPose(), target)));
            shooter.aimAtTarget(follower.getPose(), target);
            if (!enableAutoRanging) {
                shooter.setHood(hoodPosition);
                shooter.setTargetVelocity(shooterVelocity);
            }
            Position.pose = follower.getPose();
            telemetry.addData("Current Pos", follower.getPose());
            telemetry.addData("Shooter Target", shooter.getTargetVelo());
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Turret Voltage", shooter.getTurretVoltage());
            shooter.update();
            long t2 = System.nanoTime();
            long dt = t2-t1;
            telemetry.addData("Loops ms", dt/1000000.0);
            t1=t2;
            telemetry.update();
        }
    }
}