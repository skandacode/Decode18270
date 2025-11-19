package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Position;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.Arrays;
import java.util.List;

import solverslib.gamepad.Button;
import solverslib.gamepad.GamepadButton;
import solverslib.gamepad.GamepadEx;
import solverslib.gamepad.GamepadKeys;

@TeleOp
@Configurable
public class Teleop extends LinearOpMode {
    Intake intake;
    Shooter shooter;
    Spindexer spindexer;
    Follower follower;

    public static double timeforkicker = 0.2;
    public static double timeforspin = 0.41;
    public static double timeForIntake = 0.23;
    private enum RobotState {
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

    public static Shooter.Goal target = Shooter.Goal.BLUE;

    public static double powerOffsetIncrements = 20;
    public static double turretOffsetIncrements = 2;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        // Lynx modules & manual bulk caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        LynxModule controlhub = null;
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (hub.isParent()) {
                controlhub = hub;
                break;
            }
        }

        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        GamepadKeys.Button shooterButtonAll = GamepadKeys.Button.B;
        GamepadKeys.Button forceShootButton = GamepadKeys.Button.B;
        GamepadKeys.Button shooterButtonPurple = GamepadKeys.Button.Y;
        GamepadKeys.Button shooterButtonGreen = GamepadKeys.Button.X;

        GamepadKeys.Button slowModeButton = GamepadKeys.Button.RIGHT_BUMPER;
        GamepadKeys.Button intakeStopButton = GamepadKeys.Button.A;
        GamepadKeys.Button intakeEjectButton = GamepadKeys.Button.OPTIONS;

        GamepadKeys.Trigger spindexerDebugRight = GamepadKeys.Trigger.LEFT_TRIGGER;
        GamepadKeys.Trigger spindexerDebugLeft = GamepadKeys.Trigger.RIGHT_TRIGGER;

        GamepadKeys.Button positionResetButton = GamepadKeys.Button.LEFT_BUMPER;
        GamepadKeys.Button positionResetButtonHeading = GamepadKeys.Button.DPAD_DOWN;


        // subsystems
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        follower = createFollower(hardwareMap);
        follower.setStartingPose(Position.pose);
        StateMachine stateMachine = new StateMachineBuilder()
                .state(RobotState.Intake1)
                .onEnter(() -> spindexer.intakePos(0))
                .transition(() -> intake.isIntaked())
                .transition(() -> gamepadEx.isDown(forceShootButton))

                .state(RobotState.wait1)
                .onEnter(() -> {
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.intakePos(1);
                })
                .transitionTimed(timeForIntake)
                .transition(() -> gamepadEx.isDown(forceShootButton))

                .state(RobotState.Intake2)
                .onEnter(() -> spindexer.intakePos(1))
                .transition(() -> intake.isIntaked())
                .transition(() -> gamepadEx.isDown(forceShootButton))

                .state(RobotState.wait2)
                .onEnter(() -> {
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.intakePos(2);
                })
                .transitionTimed(timeForIntake)
                .transition(() -> gamepadEx.isDown(forceShootButton))

                .state(RobotState.Intake3)
                .onEnter(() -> spindexer.intakePos(2))
                .transition(() -> intake.isIntaked())
                .transition(() -> gamepadEx.isDown(forceShootButton))

                .state(RobotState.wait3)
                .onEnter(() -> {
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.shootPos(0);
                })
                .transition(() -> spindexer.atTarget())
                .transitionTimed(0.3)

                .state(RobotState.WaitForShoot)
                .transition(() -> gamepadEx.getButton(shooterButtonAll), () -> autofire = true)
                .transition(() -> gamepadEx.getButton(shooterButtonGreen) && spindexer.getIndex(Artifact.GREEN) != -1, () -> {
                    autofire = false;
                    currShoot = Artifact.GREEN;
                })
                .transition(() -> gamepadEx.getButton(shooterButtonPurple) && spindexer.getIndex(Artifact.PURPLE) != -1, () -> {
                    autofire = false;
                    currShoot = Artifact.PURPLE;
                })

                .state(RobotState.PreShoot1)
                .onEnter(() -> {
                    if (autofire) {
                        spindexer.shootPos(shootorder[0]);
                    } else {
                        spindexer.shootPos(spindexer.getIndex(currShoot));
                        currShoot = Artifact.NONE;
                    }
                })
                .transition(() -> spindexer.atTarget())
                .transitionTimed(1)

                .state(RobotState.Shoot1)
                .onEnter(() -> shooter.kickerUp())
                .transitionTimed(timeforkicker)
                .onExit(() -> {
                    shooter.kickerDown();
                    spindexer.afterShoot();
                })

                .state(RobotState.waitforrelease1)
                .transition(() -> !gamepadEx.getButton(shooterButtonGreen) && !gamepadEx.getButton(shooterButtonPurple))
                .transition(() -> autofire)

                .state(RobotState.waitforpress2)
                .transition(() -> autofire)
                .transition(() -> gamepadEx.getButton(shooterButtonGreen) && spindexer.getIndex(Artifact.GREEN) != -1, () -> currShoot = Artifact.GREEN)
                .transition(() -> gamepadEx.getButton(shooterButtonPurple) && spindexer.getIndex(Artifact.PURPLE) != -1, () -> currShoot = Artifact.PURPLE)

                .state(RobotState.PreShoot2)
                .onEnter(() -> {
                    if (autofire) {
                        spindexer.shootPos(shootorder[1]);
                    } else {
                        spindexer.shootPos(spindexer.getIndex(currShoot));
                        currShoot = Artifact.NONE;
                    }
                })
                .transitionTimed(timeforspin)

                .state(RobotState.Shoot2)
                .onEnter(() -> shooter.kickerUp())
                .transitionTimed(timeforkicker)
                .onExit(() -> {
                    shooter.kickerDown();
                    spindexer.afterShoot();
                })

                .state(RobotState.waitforrelease2)
                .transition(() -> !gamepadEx.getButton(shooterButtonGreen) && !gamepadEx.getButton(shooterButtonPurple))
                .transition(() -> autofire)

                .state(RobotState.waitforpress3)
                .transition(() -> autofire)
                .transition(() -> gamepadEx.getButton(shooterButtonGreen) && spindexer.getIndex(Artifact.GREEN) != -1, () -> currShoot = Artifact.GREEN)
                .transition(() -> gamepadEx.getButton(shooterButtonPurple) && spindexer.getIndex(Artifact.PURPLE) != -1, () -> currShoot = Artifact.PURPLE)

                .state(RobotState.PreShoot3)
                .onEnter(() -> {
                    if (autofire) {
                        spindexer.shootPos(shootorder[2]);
                    } else {
                        spindexer.shootPos(spindexer.getIndex(currShoot));
                        currShoot = Artifact.NONE;
                    }
                })
                .transitionTimed(timeforspin)

                .state(RobotState.Shoot3)
                .onEnter(() -> shooter.kickerUp())
                .transitionTimed(timeforkicker, RobotState.Intake1)
                .onExit(() -> {
                    shooter.kickerDown();
                    spindexer.afterShoot();
                    spindexer.intakePos(0);
                })

                .build();

        // Init loop - keep clearing cache and update follower for stable init telemetry
        while (opModeInInit()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();
            follower.update();
            if (gamepad1.a) {
                target = Shooter.Goal.BLUE;
            }
            if (gamepad1.b) {
                target = Shooter.Goal.RED;
            }
            telemetry.addData("Shooter Target", target);
            telemetry.addData("Current Pos", follower.getPose());
            telemetry.update();
        }

        waitForStart();
        stateMachine.start();
        shooter.kickerDown();
        follower.startTeleopDrive();

        long lastLoopTime = System.nanoTime();

        while (opModeIsActive()) {
            if (controlhub != null) {
                controlhub.clearBulkCache();
            } else {
                for (LynxModule hub : allHubs) hub.clearBulkCache();
            }

            // read gamepad inputs into GamepadEx immediately after clearing cache
            gamepadEx.readButtons();

            // timing
            long currentTime = System.nanoTime();
            double loopTime = (double) (currentTime - lastLoopTime) / 1_000_000.0;
            lastLoopTime = currentTime;

            // follower pose updates (use sensors while cache is fresh)
            follower.update();
            Position.pose = follower.getPose();

            // Shooter aiming (depends on fresh pose)
            telemetry.addData("Angle and distance:", Arrays.toString(shooter.getAngleDistance(Position.pose, target)));
            shooter.aimAtTarget(Position.pose, target);

            telemetry.addData("Current Pos", follower.getPose());
            telemetry.addData("Shooter Target", shooter.getTargetVelo());
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Turret Voltage", shooter.getTurretVoltage());

            // Update state machine (transitions use gamepadEx state)
            stateMachine.update();

            // ---Driver controls--- (read from GamepadEx which was updated above)
            double forward = gamepadEx.getLeftY();
            double strafe = gamepadEx.getLeftX();
            double turn = gamepadEx.getRightX();
            if (gamepadEx.getButton(slowModeButton)) {
                forward *= 0.3;
                strafe *= 0.3;
                turn *= 0.3;
            }

            follower.setTeleOpDrive(forward, -1*strafe, -0.8*turn, true);


            // ---Intake controls---
            if (gamepadEx.getButton(intakeStopButton)) {
                intake.setPower(0);
            } else if (gamepadEx.getButton(intakeEjectButton)) {
                intake.setPower(-1);
            } else {
                intake.setPower(1);
            }

            // ---Spindexer debug controls---
            if (gamepadEx.getTrigger(spindexerDebugRight) > 0.5) {
                spindexer.setRawPower(-0.4);
            } else if (gamepadEx.getTrigger(spindexerDebugLeft) > 0.5) {
                spindexer.setRawPower(0.4);
            } else {
                spindexer.setRawPower(0);
            }
            // ---Shooter aiming code---
            if (gamepadEx.wasJustPressed(positionResetButton)){
                follower.setPose(new Pose(65, 0, Math.toRadians(180)));
            }

            // Shooter offset
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                Shooter.powerOffset -= powerOffsetIncrements;
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                Shooter.turretOffset -= turretOffsetIncrements;
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                Shooter.turretOffset += turretOffsetIncrements;
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                Shooter.powerOffset += powerOffsetIncrements;
            }

            // Subsystem updates
            intake.update();
            shooter.update();
            spindexer.update();

            // Telemetry (add loop time & state info at the end for accuracy)
            telemetry.addData("Loop time", loopTime);
            telemetry.addData("Artifact colors", Arrays.toString(spindexer.getArtifactPositions()));
            telemetry.addData("State machine state", stateMachine.getState());
            telemetry.addData("Shooter Velo", shooter.getCurrentVelocity());
            telemetry.addData("Spindexer Pos", spindexer.getCurr_pos());
            telemetry.addData("Encoder Pos", spindexer.getEncoderPosition());
            telemetry.update();
        }
    }
}
