package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
public class Teleop extends LinearOpMode {
    Drivetrain drivetrain;
    Intake intake;
    Shooter shooter;
    Spindexer spindexer;
    boolean currEject = false;

    double targetVelo = 1700;

    enum RobotState {
        Intake1, wait1,
        Intake2, wait2,
        Intake3, wait3,
        WaitForShoot,
        PreShoot1,
        Shoot1,
        waitforrelease1,
        waitforpress2,
        PreShoot2,
        Shoot2,
        waitforrelease2,
        waitforpress3,
        PreShoot3,
        Shoot3
    }

    public static int[] shootorder = {0, 1, 2};

    public static boolean autofire = true;

    public static Artifact currShoot = Artifact.NONE;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        GamepadEx gamepadEx = new GamepadEx(gamepad1);


        GamepadKeys.Button shooterButtonAll = GamepadKeys.Button.B;
        GamepadKeys.Button shooterButtonPurple = GamepadKeys.Button.Y;
        GamepadKeys.Button shooterButtonGreen = GamepadKeys.Button.X;

        GamepadKeys.Button slowModeButton = GamepadKeys.Button.RIGHT_BUMPER;
        GamepadKeys.Button intakeStopButton = GamepadKeys.Button.A;
        GamepadKeys.Button farShootButton = GamepadKeys.Button.LEFT_BUMPER;


        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        StateMachine stateMachine = new StateMachineBuilder()
                .state(RobotState.Intake1)
                .onEnter(()->{
                    intake.setPower(-1);
                    spindexer.intakePos(0);
                })
                .onExit(()->{
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.intakePos(1);
                })
                .transition(()->intake.isIntaked())
                .state(RobotState.wait1)
                .transitionTimed(0.25)
                .state(RobotState.Intake2)
                .onEnter(()->{
                    intake.setPower(-1);
                    spindexer.intakePos(1);
                })
                .onExit(()->{
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.intakePos(2);
                })
                .transition(()->intake.isIntaked())
                .state(RobotState.wait2)
                .transitionTimed(0.25)
                .state(RobotState.Intake3)
                .onEnter(()->{
                    intake.setPower(-1);
                    spindexer.intakePos(2);
                })
                .onExit(() -> {
                    spindexer.afterIntake(intake.getArtifact());
                })
                .transition(()->intake.isIntaked())

                .state(RobotState.wait3)
                .transitionTimed(0.25)

                .state(RobotState.WaitForShoot)
                .transition(()->gamepadEx.getButton(shooterButtonAll), ()->autofire=true)
                .transition(()->gamepadEx.getButton(shooterButtonGreen) && spindexer.getIndex(Artifact.GREEN) != -1, ()->{
                    autofire=false;
                    currShoot = Artifact.GREEN;
                })
                .transition(()->gamepadEx.getButton(shooterButtonPurple) && spindexer.getIndex(Artifact.PURPLE) != -1, ()->{
                    autofire=false;
                    currShoot=Artifact.PURPLE;
                })


                .state(RobotState.PreShoot1)
                .onEnter(()->{
                    if (autofire) {
                        spindexer.shootPos(shootorder[0]);
                    }else{
                        spindexer.shootPos(spindexer.getIndex(currShoot));
                        currShoot=Artifact.NONE;
                    }
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
                })

                .state(RobotState.waitforrelease1)
                .transition(()->!gamepadEx.getButton(shooterButtonGreen) && !gamepadEx.getButton(shooterButtonPurple))
                .transition(()->autofire)

                .state(RobotState.waitforpress2)
                .transition(()->autofire)
                .transition(()->gamepadEx.getButton(shooterButtonGreen) && spindexer.getIndex(Artifact.GREEN) != -1, ()->currShoot=Artifact.GREEN)
                .transition(()->gamepadEx.getButton(shooterButtonPurple) && spindexer.getIndex(Artifact.PURPLE) != -1, ()->currShoot=Artifact.PURPLE)


                .state(RobotState.PreShoot2)
                .onEnter(()->{
                    if (autofire) {
                        spindexer.shootPos(shootorder[1]);
                    }else{
                        spindexer.shootPos(spindexer.getIndex(currShoot));
                        currShoot=Artifact.NONE;
                    }
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
                })


                .state(RobotState.waitforrelease2)
                .transition(()->!gamepadEx.getButton(shooterButtonGreen) && !gamepadEx.getButton(shooterButtonPurple))
                .transition(()->autofire)

                .state(RobotState.waitforpress3)
                .transition(()->autofire)
                .transition(()->gamepadEx.getButton(shooterButtonGreen) && spindexer.getIndex(Artifact.GREEN) != -1, ()->currShoot=Artifact.GREEN)
                .transition(()->gamepadEx.getButton(shooterButtonPurple) && spindexer.getIndex(Artifact.PURPLE) != -1, ()->currShoot=Artifact.PURPLE)



                .state(RobotState.PreShoot3)
                .onEnter(()->{
                    if (autofire) {
                        spindexer.shootPos(shootorder[2]);
                    }else{
                        spindexer.shootPos(spindexer.getIndex(currShoot));
                        currShoot=Artifact.NONE;
                    }
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
                turn *= 0.3;
            }
            drivetrain.driveRobotCentric(forward, strafe, turn);
            shooter.setTurretAngle(0);
            // Intake control
            if (gamepadEx.getButton(intakeStopButton)) {
                intake.setPower(0); // Stop intake
            } else {
                if (!currEject) {
                    intake.setPower(-1); // Run intake
                }
            }
            if (gamepadEx.getButton(farShootButton)){
                targetVelo=2800;
            }else{
                targetVelo=1850;

            }
            telemetry.addData("Artifact colors", Arrays.toString(spindexer.getArtifactPositions()));
            telemetry.addData("State machine state", stateMachine.getState());

            shooter.setTargetVelo(targetVelo);
            gamepadEx.readButtons();
            stateMachine.update();
            drivetrain.update();
            intake.update();
            shooter.update();
            spindexer.update();
            telemetry.update();
        }
    }
}
