package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.State;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import java.util.List;
@Autonomous(name = "BlueAutoFar", group = "Auto")
public class BlueAutoFar extends LinearOpMode {

    private Follower follower;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static int[] shootorder = {0, 1, 2};
    public static Artifact currShoot = Artifact.NONE;
    public boolean shooterButtonGreen = false;
    public boolean shooterButtonPurple = false;
    Drivetrain drivetrain;
    Intake intake;
    Shooter shooter;
    Spindexer spindexer;
    private boolean autofire = false;
    private StateMachine stateMachine;

    public static double timeforkicker = 0.2;
    public static double timeforspin = 0.34;
    public static double timeForIntake = 0.23;

    private final Pose startPose = new Pose(43, 0, Math.toRadians(180));
    private final Pose shootPose = new Pose(130, -2.6, Math.toRadians(-135));
    private final Pose intake1Pose = new Pose(120, -7, Math.toRadians(90));
    private final Pose intake2Pose = new Pose(96, -7, Math.toRadians(90));
    private final Pose intake3Pose = new Pose(70,-7, Math.toRadians(90));
    private final Pose intake1donePose = new Pose(120, 15, Math.toRadians(90));
    private final Pose intake2donePose = new Pose(96, 15, Math.toRadians(90));
    private final Pose intake3donePose = new Pose(70, 15, Math.toRadians(90));
    private final Pose leave = new Pose(16, 33, Math.toRadians(-135));


    public enum AutoStates {
        MOVETOSHOOT1, SHOOT1,
        MOVETOINTAKE1, INTAKE1,
        MOVETOSHOOT2, SHOOT2,
        MOVETOINTAKE2, INTAKE2,
        MOVETOSHOOT3, SHOOT3,
        MOVETOINTAKE3, INTAKE3,
        MOVETOSHOOT4, SHOOT4,
        LEAVE
    }
    enum RobotState {
        Intake1, wait1,
        Intake2, wait2,
        Intake3, wait3,
        WaitForShoot,
        PreShoot1, Shoot1, waitforrelease1, waitforpress2,
        PreShoot2, Shoot2, waitforrelease2, waitforpress3,
        PreShoot3, Shoot3
    }

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        follower = createFollower(hardwareMap);
        PathChain toShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        PathChain toIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake1Pose.getHeading())
                .build();

        PathChain toIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake2Pose.getHeading())
                .build();

        PathChain toIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake3Pose.getHeading())
                .build();
        PathChain toIntake1fin = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, intake1donePose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), intake1donePose.getHeading())
                .build();

        PathChain toIntake2fin = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, intake2donePose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), intake2donePose.getHeading())
                .build();

        PathChain toIntake3fin = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, intake3donePose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), intake3donePose.getHeading())
                .build();

        PathChain toScore1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1donePose, shootPose))
                .setLinearHeadingInterpolation(intake1donePose.getHeading(), shootPose.getHeading())
                .build();

        PathChain toScore2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2donePose, shootPose))
                .setLinearHeadingInterpolation(intake2donePose.getHeading(), shootPose.getHeading())
                .build();

        PathChain toScore3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3donePose, shootPose))
                .setLinearHeadingInterpolation(intake3donePose.getHeading(), shootPose.getHeading())
                .build();
        PathChain park = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leave))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leave.getHeading())
                .build();

        follower.setStartingPose(startPose);

        StateMachine stateMachine = new StateMachineBuilder()
                .state(RobotState.Intake1)
                .onEnter(()->{
                    spindexer.intakePos(0);
                })
                .transition(()->intake.isIntaked())

                .state(RobotState.wait1)
                .onEnter(()->{
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.intakePos(1);
                })
                .transitionTimed(timeForIntake)

                .state(RobotState.Intake2)
                .onEnter(()->{
                    spindexer.intakePos(1);
                })
                .transition(()->intake.isIntaked())

                .state(RobotState.wait2)
                .onEnter(()->{
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.intakePos(2);
                })
                .transitionTimed(timeForIntake)

                .state(RobotState.Intake3)
                .onEnter(()->{
                    spindexer.intakePos(2);
                })
                .transition(()->intake.isIntaked())

                .state(RobotState.wait3)
                .onEnter(()->{
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.shootPos(0);
                })
                .transition(()->spindexer.atTarget())

                .state(RobotState.WaitForShoot)
                .transition(()->shooterButtonGreen && spindexer.getIndex(Artifact.GREEN) != -1, ()->{
                    autofire=false;
                    currShoot = Artifact.GREEN;
                })
                .transition(()->shooterButtonPurple && spindexer.getIndex(Artifact.PURPLE) != -1, ()->{
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
                .transition(()->spindexer.atTarget())

                .state(RobotState.Shoot1)
                .onEnter(()->{
                    shooter.kickerUp();
                })
                .transitionTimed(timeforkicker)

                .onExit(()->{
                    shooter.kickerDown();
                    spindexer.afterShoot();
                })

                .state(RobotState.waitforrelease1)
                .transition(()->!shooterButtonGreen && !shooterButtonPurple)
                .transition(()->autofire)

                .state(RobotState.waitforpress2)
                .transition(()->autofire)
                .transition(()->shooterButtonGreen && spindexer.getIndex(Artifact.GREEN) != -1, ()->currShoot=Artifact.GREEN)
                .transition(()->shooterButtonPurple && spindexer.getIndex(Artifact.PURPLE) != -1, ()->currShoot=Artifact.PURPLE)


                .state(RobotState.PreShoot2)
                .onEnter(()->{
                    if (autofire) {
                        spindexer.shootPos(shootorder[1]);
                    }else{
                        spindexer.shootPos(spindexer.getIndex(currShoot));
                        currShoot=Artifact.NONE;
                    }
                })
                .transitionTimed(timeforspin)

                .state(RobotState.Shoot2)
                .onEnter(()->{
                    shooter.kickerUp();
                })
                .transitionTimed(timeforkicker)
                .onExit(()->{
                    shooter.kickerDown();
                    spindexer.afterShoot();
                })


                .state(RobotState.waitforrelease2)
                .transition(()->!shooterButtonGreen && !shooterButtonPurple)
                .transition(()->autofire)

                .state(RobotState.waitforpress3)
                .transition(()->autofire)
                .transition(()->shooterButtonGreen && spindexer.getIndex(Artifact.GREEN) != -1, ()->currShoot=Artifact.GREEN)
                .transition(()-> shooterButtonPurple && spindexer.getIndex(Artifact.PURPLE) != -1, ()->currShoot=Artifact.PURPLE)



                .state(RobotState.PreShoot3)
                .onEnter(()->{
                    if (autofire) {
                        spindexer.shootPos(shootorder[2]);
                    }else{
                        spindexer.shootPos(spindexer.getIndex(currShoot));
                        currShoot=Artifact.NONE;
                    }
                })
                .transitionTimed(timeforspin)

                .state(RobotState.Shoot3)
                .onEnter(()->{
                    shooter.kickerUp();
                })
                .transitionTimed(timeforkicker, RobotState.Intake1)
                .onExit(()->{
                    shooter.kickerDown();
                    spindexer.afterShoot();
                    spindexer.intakePos(0);
                })

                .build();




        StateMachine autoMachine = new StateMachineBuilder() //Autonomia
                .state(AutoStates.MOVETOSHOOT1)
                .onEnter(()->{
                    follower.followPath(toShoot, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)

                .state(AutoStates.SHOOT1)
                .onEnter(()->{
                })
                .transitionTimed(1)
                .state(AutoStates.MOVETOINTAKE1)
                .onEnter(()->{
                    follower.followPath(toIntake1, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1)
                .state(AutoStates.INTAKE1)
                .onEnter(()->{
                    follower.followPath(toIntake1fin, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)
                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(()->{
                    follower.followPath(toScore1, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1)

                .state(AutoStates.SHOOT2)
                .onEnter(()->{
                })
                .transitionTimed(1)
                .state(AutoStates.MOVETOINTAKE2)
                .onEnter(()->{
                    follower.followPath(toIntake2, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1)
                .state(AutoStates.INTAKE2)
                .onEnter(()->{
                    follower.followPath(toIntake2fin, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)
                .state(AutoStates.MOVETOSHOOT3)
                .onEnter(()->{
                    follower.followPath(toScore2, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1)

                .state(AutoStates.SHOOT3)
                .onEnter(()->{
                })
                .transitionTimed(1)
                .state(AutoStates.MOVETOINTAKE3)
                .onEnter(()->{
                    follower.followPath(toIntake3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1)
                .state(AutoStates.INTAKE3)
                .onEnter(()->{
                    follower.followPath(toIntake3fin, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)
                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{
                    follower.followPath(toScore3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1)
                .state(AutoStates.SHOOT4)
                .onEnter(()->{
                })
                .transitionTimed(1)
                .state(AutoStates.LEAVE)
                .onEnter(()->{
                    follower.followPath(park, true);
                })
                .transition(()->follower.atParametricEnd())

                .build();





        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            follower.update();
            panelsTelemetry.debug("Init Pose: " + follower.getPose());
            panelsTelemetry.update(telemetry);
        }

        waitForStart();
        stateMachine.start();
        autoMachine.start();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            stateMachine.update();
            autoMachine.update();
            follower.update();
            intake.update();
            shooter.update();
            spindexer.update();

            panelsTelemetry.debug("State: " + stateMachine.getState());
            panelsTelemetry.debug("State auto: " + autoMachine.getState());
            panelsTelemetry.debug("Pose: " + follower.getPose());
            panelsTelemetry.update(telemetry);

            telemetry.update();
        }
    }
}
