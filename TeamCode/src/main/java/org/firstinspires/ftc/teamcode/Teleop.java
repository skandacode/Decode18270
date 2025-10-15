package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import solverslib.gamepad.GamepadEx;
import solverslib.gamepad.GamepadKeys;

@Configurable
public class Teleop extends LinearOpMode {
    Drivetrain drivetrain;
    Intake intake;
    Shooter shooter;
    Spindexer spindexer;

    boolean currEject = false;

    double targetVelo = 2000;

    enum RobotState {
        Intake1,
        Intake2,
        Intake3,
        WaitForShoot,
        PreShoot1,
        Shoot1,
        PreShoot2,
        Shoot2,
        PreShoot3,
        Shoot3
    }

    int[] shootorder = {0, 1, 2};
    private static final int NUM_INTAKE_STATES = 3;
    private static final int NUM_SHOOT_STATES = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        GamepadKeys.Button shooterButton = GamepadKeys.Button.A;
        GamepadKeys.Button slowModeButton = GamepadKeys.Button.RIGHT_BUMPER;
        GamepadKeys.Button intakeStopButton = GamepadKeys.Button.LEFT_BUMPER;

        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        StateMachineBuilder builder = new StateMachineBuilder();

        for (int i = 0; i < NUM_INTAKE_STATES; i++) {
            final int pos = i;
            RobotState currentState = RobotState.Intake1;
            if (i == 1) currentState = RobotState.Intake2;
            else if (i == 2) currentState = RobotState.Intake3;
            
            builder.state(currentState)
                .onEnter(() -> {
                    intake.setPower(1);
                    spindexer.intakePos(pos);
                })
                .onExit(() -> {
                    spindexer.afterIntake(intake.getArtifact());
                    if (pos < NUM_INTAKE_STATES - 1) {
                        spindexer.intakePos(pos + 1);
                    } else {
                        spindexer.shootPos(shootorder[0]);
                    }
                })
                .transition(() -> intake.isIntaked());
        }

        builder.state(RobotState.WaitForShoot)
            .transition(() -> gamepadEx.getButton(shooterButton));

        // Build shoot states dynamically
        for (int i = 0; i < NUM_SHOOT_STATES; i++) {
            final int shootIndex = i;
            RobotState preShootState = RobotState.PreShoot1;
            if (i == 1) preShootState = RobotState.PreShoot2;
            else if (i == 2) preShootState = RobotState.PreShoot3;

            RobotState shootState = RobotState.Shoot1;
            if (i == 1) shootState = RobotState.Shoot2;
            else if (i == 2) shootState = RobotState.Shoot3;
            
            // PreShoot state
            builder.state(preShootState)
                .onEnter(() -> spindexer.shootPos(shootorder[shootIndex]))
                .transitionTimed(0.5);

            // Shoot state
            StateMachineBuilder stateBuilder = builder.state(shootState)
                .onEnter(() -> shooter.kickerUp());
            
            if (i < NUM_SHOOT_STATES - 1) {
                // Not the last shoot state
                stateBuilder.transitionTimed(0.3)
                    .onExit(() -> {
                        shooter.kickerDown();
                        spindexer.afterShoot();
                        spindexer.shootPos(shootorder[shootIndex + 1]);
                    });
            } else {
                // Last shoot state - loop back to Intake1
                stateBuilder.transitionTimed(0.3, RobotState.Intake1)
                    .onExit(() -> {
                        shooter.kickerDown();
                        spindexer.afterShoot();
                        spindexer.intakePos(0);
                    });
            }
        }

        StateMachine stateMachine = builder.build();

        waitForStart();
        stateMachine.start();
        
        while (opModeIsActive()) {
            // Drivetrain control
            double forward = gamepadEx.getLeftY();
            double strafe = gamepadEx.getLeftX();
            double turn = gamepadEx.getRightX();
            
            if (gamepadEx.getButton(slowModeButton)) {
                forward *= 0.3;
                strafe *= 0.3;
                turn *= -0.3;
            }
            
            drivetrain.driveRobotCentric(forward, strafe, turn);

            // Intake control
            if (gamepadEx.getButton(intakeStopButton)) {
                intake.setPower(0);
            } else if (!currEject) {
                intake.setPower(1);
            }
            
            shooter.setTargetVelo(targetVelo);
            
            // Update all subsystems
            stateMachine.update();
            drivetrain.update();
            intake.update();
            shooter.update();
            spindexer.update();
            telemetry.update();
        }
    }
}
