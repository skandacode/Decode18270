package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.Arrays;

import solverslib.gamepad.Button;
import solverslib.gamepad.GamepadButton;
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
