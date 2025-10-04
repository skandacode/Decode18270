package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import solverslib.gamepad.Button;
import solverslib.gamepad.GamepadButton;
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

        StateMachine stateMachine = new StateMachineBuilder()
                .state(RobotState.Intake1)
                .onEnter(()->{
                    intake.setPower(1);
                    spindexer.intakePos(0);
                })
                .onExit(()->{
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.intakePos(1);
                })
                .transition(()->intake.isIntaked())

                .state(RobotState.Intake2)
                .onEnter(()->{
                    intake.setPower(1);
                    spindexer.intakePos(1);
                })
                .onExit(()->{
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.intakePos(2);
                })
                .transition(()->intake.isIntaked())

                .state(RobotState.Intake3)
                .onEnter(()->{
                    intake.setPower(1);
                    spindexer.intakePos(2);
                })
                .onExit(()->{
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.shootPos(shootorder[0]);
                })
                .transition(()->intake.isIntaked())

                .state(RobotState.WaitForShoot)
                .transition(()->gamepadEx.getButton(shooterButton))

                .state(RobotState.PreShoot1)
                .onEnter(()->{
                    spindexer.shootPos(shootorder[0]);
                })
                .transitionTimed(0.5)

                .state(RobotState.Shoot1)
                .onEnter(()->{
                    shooter.kickerUp();
                })
                .transitionTimed(0.3)

                .onExit(()->{
                    shooter.kickerDown();
                    spindexer.afterShoot();
                    spindexer.shootPos(shootorder[1]);
                })

                .state(RobotState.PreShoot2)
                .onEnter(()->{
                    spindexer.shootPos(shootorder[1]);
                })
                .transitionTimed(0.5)

                .state(RobotState.Shoot2)
                .onEnter(()->{
                    shooter.kickerUp();
                })
                .transitionTimed(0.3)
                .onExit(()->{
                    shooter.kickerDown();
                    spindexer.afterShoot();
                    spindexer.shootPos(shootorder[2]);
                })
                .state(RobotState.PreShoot3)
                .onEnter(()->{
                    spindexer.shootPos(shootorder[2]);
                })
                .transitionTimed(0.5)
                .state(RobotState.Shoot3)
                .onEnter(()->{
                    shooter.kickerUp();
                })
                .transitionTimed(0.3, RobotState.Intake1)
                .onExit(()->{
                    shooter.kickerDown();
                    spindexer.afterShoot();
                    spindexer.intakePos(0);
                })

                .build();


        waitForStart();
        stateMachine.start();
        while (opModeIsActive()) {
            // Drivetrain control
            double forward = gamepadEx.getLeftY();
            double strafe = gamepadEx.getLeftX();
            double turn = gamepadEx.getRightX();
            if (gamepadEx.getButton(slowModeButton)){
                forward *= 0.3;
                strafe *= 0.3;
                turn *= -0.3;
            }
            drivetrain.driveRobotCentric(forward, strafe, turn);

            // Intake control
            if (gamepadEx.getButton(intakeStopButton)) {
                intake.setPower(0); // Stop intake
            } else {
                if (!currEject) {
                    intake.setPower(1); // Run intake
                }
            }
            shooter.setTargetVelo(targetVelo);
            stateMachine.update();
            drivetrain.update();
            intake.update();
            shooter.update();
            spindexer.update();
            telemetry.update();
        }
    }
}
