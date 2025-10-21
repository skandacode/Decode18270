package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import solverslib.gamepad.GamepadEx;
import solverslib.gamepad.GamepadKeys;
@Autonomous(preselectTeleOp = "AutomatedTeleop")
public class SampleAutoPedro8Maybe extends LinearOpMode {
    private Follower follower;
    Intake intake;
    Shooter shooter;
    Drivetrain drivetrain;
    Spindexer spindexer;
    StateMachine autoMachine;
    public enum stateMachineStates {
        IDLE, EXTEND, SENSORWAIT, SENSE, LIFTUP, RETRACT, PULSE, WAIT, CLOSE, LIFT, PARTIALFLIP, SCORE, AUTOWAIT, OPEN, LOWERLIFT, EJECTFLIP, REVERSE, REINTAKE, EJECTLIDOPEN
    }

    enum AutoStates{MOVETOSHOOT1, SHOOT1,
        MOVETOINTAKE1, INTAKE1,
        MOVETOSHOOT2, SHOOT2,
        MOVETOINTAKE3, INTAKE3,
        MOVETOSHOOT3, SHOOT3,
        MOVETOINTAKE4, INTAKE4,
        MOVETOGATE, OPENGATE,
        MOVETOSHOOT4, SHOOT4,
        MOVETOINTAKE5, INTAKE5,
        MOVETOSHOOT5, SHOOT5,
        MOVETOINTAKE6, INTAKE6,
        MOVETOSHOOT6, SHOOT6,
        LEAVE}

    @Override
    public void runOpMode() throws InterruptedException {
        follower.setStartingPose(startPose);
        StateMachine stateMachine = new StateMachineBuilder()
                .state(Teleop.RobotState.Intake1)
                .onEnter(()->{
                    intake.setPower(-1);
                    spindexer.intakePos(0);
                })
                .onExit(()->{
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.intakePos(1);
                })
                .transition(()->intake.isIntaked())
                .state(Teleop.RobotState.wait1)
                .transitionTimed(0.25)
                .state(Teleop.RobotState.Intake2)
                .onEnter(()->{
                    intake.setPower(-1);
                    spindexer.intakePos(1);
                })
                .onExit(()->{
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.intakePos(2);
                })
                .transition(()->intake.isIntaked())
                .state(Teleop.RobotState.wait2)
                .transitionTimed(0.25)
                .state(Teleop.RobotState.Intake3)
                .onEnter(()->{
                    intake.setPower(-1);
                    spindexer.intakePos(2);
                })
                .onExit(() -> {
                    spindexer.afterIntake(intake.getArtifact());
                })
                .transition(()->intake.isIntaked())

                .state(Teleop.RobotState.wait3)
                .transitionTimed(0.25)

                .state(Teleop.RobotState.WaitForShoot)
                .transition(()->gamepadEx.getButton(shooterButtonAll), ()->autofire=true)
                .transition(()->gamepadEx.getButton(shooterButtonGreen) && spindexer.getIndex(Artifact.GREEN) != -1, ()->{
                    autofire=false;
                    currShoot = Artifact.GREEN;
                })
                .transition(()->gamepadEx.getButton(shooterButtonPurple) && spindexer.getIndex(Artifact.PURPLE) != -1, ()->{
                    autofire=false;
                    currShoot=Artifact.PURPLE;
                })


                .state(Teleop.RobotState.PreShoot1)
                .onEnter(()->{
                    if (autofire) {
                        spindexer.shootPos(shootorder[0]);
                    }else{
                        spindexer.shootPos(spindexer.getIndex(currShoot));
                        currShoot=Artifact.NONE;
                    }
                })
                .transitionTimed(0.5)

                .state(Teleop.RobotState.Shoot1)
                .onEnter(()->{
                    shooter.kickerUp();
                })
                .transitionTimed(0.3)

                .onExit(()->{
                    shooter.kickerDown();
                    spindexer.afterShoot();
                })

                .state(Teleop.RobotState.waitforrelease1)
                .transition(()->!gamepadEx.getButton(shooterButtonGreen) && !gamepadEx.getButton(shooterButtonPurple))
                .transition(()->autofire)

                .state(Teleop.RobotState.waitforpress2)
                .transition(()->autofire)
                .transition(()->gamepadEx.getButton(shooterButtonGreen) && spindexer.getIndex(Artifact.GREEN) != -1, ()->currShoot=Artifact.GREEN)
                .transition(()->gamepadEx.getButton(shooterButtonPurple) && spindexer.getIndex(Artifact.PURPLE) != -1, ()->currShoot=Artifact.PURPLE)


                .state(Teleop.RobotState.PreShoot2)
                .onEnter(()->{
                    if (autofire) {
                        spindexer.shootPos(shootorder[1]);
                    }else{
                        spindexer.shootPos(spindexer.getIndex(currShoot));
                        currShoot=Artifact.NONE;
                    }
                })
                .transitionTimed(0.5)

                .state(Teleop.RobotState.Shoot2)
                .onEnter(()->{
                    shooter.kickerUp();
                })
                .transitionTimed(0.3)
                .onExit(()->{
                    shooter.kickerDown();
                    spindexer.afterShoot();
                })


                .state(Teleop.RobotState.waitforrelease2)
                .transition(()->!gamepadEx.getButton(shooterButtonGreen) && !gamepadEx.getButton(shooterButtonPurple))
                .transition(()->autofire)

                .state(Teleop.RobotState.waitforpress3)
                .transition(()->autofire)
                .transition(()->gamepadEx.getButton(shooterButtonGreen) && spindexer.getIndex(Artifact.GREEN) != -1, ()->currShoot=Artifact.GREEN)
                .transition(()->gamepadEx.getButton(shooterButtonPurple) && spindexer.getIndex(Artifact.PURPLE) != -1, ()->currShoot=Artifact.PURPLE)



                .state(Teleop.RobotState.PreShoot3)
                .onEnter(()->{
                    if (autofire) {
                        spindexer.shootPos(shootorder[2]);
                    }else{
                        spindexer.shootPos(spindexer.getIndex(currShoot));
                        currShoot=Artifact.NONE;
                    }
                })
                .transitionTimed(0.5)
                .state(Teleop.RobotState.Shoot3)
                .onEnter(()->{
                    shooter.kickerUp();
                })
                .transitionTimed(0.3, Teleop.RobotState.Intake1)
                .onExit(()->{
                    shooter.kickerDown();
                    spindexer.afterShoot();
                    spindexer.intakePos(0);
                })

                .build();


        autoMachine=new StateMachineBuilder();

        shooter.setMotors(-0.1);
        shooter.setTargetPos(0);
        intake.setIntakeFlip(0.3);
        hang.setLatchPos(Hang.LatchPositions.FULLY_RETRACTED);
        extendPressed=false;
        scorePressed=false;
        known=true;
        while (opModeInInit()) {
            if (!(controlhub==null)) {
                controlhub.clearBulkCache();
                telemetry.addLine("bulk reading only chub");
            }else{
                for (LynxModule hub:allHubs){
                    hub.clearBulkCache();
                }
            }
            follower.updatePose();
            intake.update();
            shooter.update();
            telemetry.addData("Outtake motor power", shooter.currPower);
            if (gamepad1.a) {
                allianceColor = Intake.SampleColor.BLUE;
                gamepad1.setLedColor(0, 0, 1, 1000);
            }
            if (gamepad1.b) {
                allianceColor = Intake.SampleColor.RED;
                gamepad1.setLedColor(1, 0, 0, 1000);
            }
            telemetry.addData("Pose", follower.getPose().toString());
            telemetry.addData("Alliance Color", allianceColor.toString());
            telemetry.addData("Extend Pressed", extendPressed);
            telemetry.addData("Score Pressed", scorePressed);

            telemetry.update();
        }
        waitForStart();
        autoMachine.start();
        stateMachine.start();

        }
    }
}
