package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import solverslib.gamepad.GamepadEx;
import solverslib.gamepad.GamepadKeys;
@TeleOp
@Configurable
public class drivev2 extends LinearOpMode {
    Drivetrain drivetrain;
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        GamepadKeys.Button slowModeButton = GamepadKeys.Button.RIGHT_BUMPER;

        drivetrain = new Drivetrain(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            // Drivetrain control
            double forward = gamepadEx.getLeftY();
            double strafe = gamepadEx.getLeftX();
            double turn = gamepadEx.getRightX();
            if (gamepadEx.getButton(slowModeButton)){
                forward *= 0.3;
                strafe *= 0.3;
                turn *= 0.3;
            }
            drivetrain.driveRobotCentric(forward, strafe, turn);
            gamepadEx.readButtons();
            drivetrain.update();
            telemetry.update();
        }
    }
}
