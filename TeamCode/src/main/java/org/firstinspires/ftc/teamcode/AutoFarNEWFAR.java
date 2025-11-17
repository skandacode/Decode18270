package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.pedroPathing.Position;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimelightMotif;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.Arrays;
import java.util.List;

@Configurable
@Autonomous(name = "AutoFarNEWFARSHOOT", group = "Auto")
public class AutoFarNEWFAR extends LinearOpMode {
    private Follower follower;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static int[] shootorder = {0, 1, 2};
    public boolean shooterButtonAll = false;
    LimelightMotif limelightMotif;
    Intake intake;
    String colorAlliance = "BLUE";
    int Posmultiplier = 1;
    Shooter shooter;
    Spindexer spindexer;
    public int pattern = 1;
    public static double timeforkicker = 0.2;
    public static double timeforspin = 0.53;
    public static double timeForIntake = 0.23;
    public static double waitforkickerdown = 0.1;

    public static Shooter.Goal shooterTarget = Shooter.Goal.BLUE;

    public enum AutoStates {
        MOVETOSHOOT1, wait1, SHOOT1,
        MOVETOINTAKE1, INTAKE1, INTAKE1BACK,
        MOVETOSHOOT2, wait2, SHOOT2,
        MOVETOINTAKE2, INTAKE2, INTAKE2BACK,
        MOVETOSHOOT3, wait3,SHOOT3,
        MOVETOINTAKE3, INTAKE3, INTAKE3BACK,
        MOVETOSHOOT4, wait4,SHOOT4,
        MOVETOINTAKE4, INTAKE4, INTAKE4BACK,
        MOVETOSHOOT5, wait5,SHOOT5,
        LEAVE
    }
    private enum RobotState {
        Intake1, wait1,
        Intake2, wait2,
        Intake3, wait3, reverseIntake,
        WaitForShoot,
        PreShoot1,
        Shoot1, spin1,
        PreShoot2,
        Shoot2, spin2,
        PreShoot3,
        Shoot3, spin3
    }


    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        limelightMotif = new LimelightMotif(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        follower = createFollower(hardwareMap);
        shooter.kickerDown();

        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            follower.update();
            int currpattern = limelightMotif.getMotif();
            if (currpattern != 0){
                pattern = currpattern;
            }else{
                panelsTelemetry.addLine("Don't see anything");
            }
            panelsTelemetry.debug("Pattern", pattern);
            panelsTelemetry.debug("Init Pose: " + follower.getPose());
            panelsTelemetry.debug("ALLIANCE: " + colorAlliance);
            panelsTelemetry.update(telemetry);
            if (gamepad1.a){
                colorAlliance="BLUE";
                shooterTarget = Shooter.Goal.BLUE;
                Posmultiplier=1;
            }
            if (gamepad1.b){
                colorAlliance="RED";
                shooterTarget = Shooter.Goal.RED;
                Posmultiplier=-1;
            }
        }

        telemetry.update();
        shooter.kickerDown();
        shooter.setHood(0.9);
        waitForStart();
        Pose startPose = new Pose(65, -27*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose shootPose = new Pose(63, -20*Posmultiplier, Math.toRadians(-89*Posmultiplier));
        Pose shootPoseangcycles = new Pose(63, -20*Posmultiplier, Math.toRadians(-88*Posmultiplier));
        Pose shootPoseangstart = new Pose(63, -20*Posmultiplier, Math.toRadians(-88*Posmultiplier));
        Pose intake1Pose = new Pose(40, -26*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake2Pose = new Pose(30, -58*Posmultiplier, Math.toRadians(-0*Posmultiplier));
        Pose intake3Pose = new Pose(64,-30*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake1donePose = new Pose(40, -59*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake2donePose = new Pose(63, -69*Posmultiplier, Math.toRadians(-0*Posmultiplier));
        Pose intake3donePose = new Pose(64, -60*Posmultiplier, Math.toRadians(-95*Posmultiplier));
        Pose leave = new Pose(50, -50*Posmultiplier, Math.toRadians(-85*Posmultiplier));


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
        PathChain toIntake1back= follower.pathBuilder()
                .addPath(new BezierLine(intake1donePose, intake1Pose))
                .setLinearHeadingInterpolation(intake1donePose.getHeading(), intake1Pose.getHeading())
                .build();

        PathChain toIntake2back= follower.pathBuilder()
                .addPath(new BezierLine(intake2donePose, intake2Pose))
                .setLinearHeadingInterpolation(intake2donePose.getHeading(), intake2Pose.getHeading())
                .build();
        PathChain toIntake3back= follower.pathBuilder()
                .addPath(new BezierLine(intake3donePose, intake3Pose))
                .setLinearHeadingInterpolation(intake3donePose.getHeading(), intake3Pose.getHeading())
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
                .addPath(new BezierLine(intake2Pose, shootPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shootPose.getHeading())
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
                .onEnter(() -> spindexer.intakePos(0))
                .transition(() -> intake.isIntaked())
                .transition(() -> shooterButtonAll)


                .state(RobotState.wait1)
                .onEnter(() -> {
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.intakePos(1);
                })
                .transitionTimed(timeForIntake)
                .transition(() -> shooterButtonAll)


                .state(RobotState.Intake2)
                .onEnter(() -> spindexer.intakePos(1))
                .transition(() -> intake.isIntaked())
                .transition(() -> shooterButtonAll)


                .state(RobotState.wait2)
                .onEnter(() -> {
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.intakePos(2);
                })
                .transitionTimed(timeForIntake)
                .transition(() -> shooterButtonAll)


                .state(RobotState.Intake3)
                .onEnter(() -> spindexer.intakePos(2))
                .transition(() -> intake.isIntaked())
                .transition(() -> shooterButtonAll)
                .state(RobotState.wait3)
                .onEnter(() -> {
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.shootPos(0);
                })
                .transition(() -> spindexer.atTarget())
                .transition(() -> shooterButtonAll)
                .transitionTimed(0.3)

                .state(RobotState.reverseIntake)
                .onEnter(() -> intake.setPower(-0.4))
                .transitionTimed(0.15)
                .transition(() -> shooterButtonAll)


                .state(RobotState.WaitForShoot)
                .onEnter(() -> intake.setPower(1))
                .transition(() -> shooterButtonAll)

                .state(RobotState.PreShoot1)
                .onEnter(() -> {
                    shooterButtonAll=false;
                    spindexer.shootPos(shootorder[0]);
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

                .state(RobotState.spin1)
                .transitionTimed(waitforkickerdown)
                .onExit(() ->  spindexer.afterShoot())

                .state(RobotState.PreShoot2)
                .onEnter(() -> spindexer.shootPos(shootorder[1]))
                .transitionTimed(timeforspin)


                .state(RobotState.Shoot2)
                .onEnter(() -> shooter.kickerUp())
                .transitionTimed(timeforkicker)
                .onExit(() -> {
                    shooter.kickerDown();
                    spindexer.afterShoot();
                })
                .state(RobotState.spin2)
                .transitionTimed(waitforkickerdown)
                .onExit(() ->  spindexer.afterShoot())
                .state(RobotState.PreShoot3)
                .onEnter(() -> spindexer.shootPos(shootorder[2]))
                .transitionTimed(timeforspin)


                .state(RobotState.Shoot3)
                .onEnter(() -> shooter.kickerUp())
                .transitionTimed(timeforkicker)
                .onExit(() -> {
                    shooter.kickerDown();
                    spindexer.afterShoot();
                })

                .state(RobotState.spin3)
                .onExit(()->spindexer.intakePos(0))
                .transitionTimed(waitforkickerdown, RobotState.Intake1)
                .build();


        StateMachine autoMachine = new StateMachineBuilder() //Autonomia
                .state(AutoStates.MOVETOSHOOT1)
                .onEnter(()->{
                    follower.followPath(toShoot, true);
                    shooter.aimAtTarget(shootPoseangstart, shooterTarget);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.5)
                .state(AutoStates.wait1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT1);
                })
                .transitionTimed(1.6)
                .state(AutoStates.SHOOT1)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(2.5)
                .transition(()->stateMachine.getStateEnum() == RobotState.spin3)

                .state(AutoStates.MOVETOINTAKE1)
                .onEnter(()->{
                    follower.followPath(toIntake1, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.7)

                .state(AutoStates.INTAKE1)
                .onEnter(()->{
                    shooter.aimAtTarget(shootPoseangcycles, shooterTarget);
                    follower.followPath(toIntake1fin, 0.5, true);
                })
                .transitionTimed(1.7)
                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(()->{
                    intake.setPower(1);
                    follower.followPath(toScore1, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)

                .state(AutoStates.wait2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT1);
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT2)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(2.5)
                .transition(()->stateMachine.getStateEnum() == RobotState.spin3)
                .state(AutoStates.MOVETOINTAKE2)
                .onEnter(()->{
                    follower.followPath(toIntake2, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.9)

                .state(AutoStates.INTAKE2)
                .onEnter(()->{
                    follower.followPath(toIntake2fin, 0.67, true);
                })
                .transition(()->stateMachine.getStateEnum() == RobotState.WaitForShoot)
                .transitionTimed(2)

                .state(AutoStates.INTAKE2BACK)
                .onEnter(()->{
                    follower.followPath(toIntake2back, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.3)

                .state(AutoStates.MOVETOSHOOT3)
                .onEnter(()->{
                    follower.followPath(toScore2, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.4)

                .state(AutoStates.wait3)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT1);
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT3)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(2.4)
                .transition(()->stateMachine.getStateEnum() == RobotState.spin3)
                .state(AutoStates.MOVETOINTAKE3)
                .onEnter(()->{
                    follower.followPath(toIntake3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.INTAKE3)
                .onEnter(()->{
                    follower.followPath(toIntake3fin, 0.5, true);
                })
                .transitionTimed(2)
                .transition(()->stateMachine.getStateEnum() == RobotState.WaitForShoot)
                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{
                    follower.followPath(toScore3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.5)
                .state(AutoStates.wait4)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT1);
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT4)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(2)
                .transition(()->stateMachine.getStateEnum() == RobotState.spin3)
                .state(AutoStates.MOVETOINTAKE4)
                .onEnter(()->{
                    follower.followPath(toIntake3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.INTAKE4)
                .onEnter(()->{
                    follower.followPath(toIntake3fin, 0.5, true);
                })
                .transitionTimed(2)
                .transition(()->stateMachine.getStateEnum() == RobotState.WaitForShoot)
                .state(AutoStates.MOVETOSHOOT5)
                .onEnter(()->{
                    follower.followPath(toScore3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.5)
                .state(AutoStates.wait5)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT1);
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT5)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(2.4)
                .transition(()->stateMachine.getStateEnum() == RobotState.spin3)
                .state(AutoStates.LEAVE)
                .onEnter(()->{
                    follower.followPath(park, true);
                })
                .build();

        stateMachine.start();
        stateMachine.setState(RobotState.WaitForShoot);

        spindexer.setArtifactPositions(new String[] {"GREEN", "PURPLE", "PURPLE"});
        autoMachine.start();
        intake.setPower(1);
        spindexer.shootPos(0);
        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            Position.pose = follower.getPose();

            stateMachine.update();
            autoMachine.update();
            follower.update();
            intake.update();
            shooter.update();
            spindexer.update();
            panelsTelemetry.debug("Artifact colors", Arrays.toString(spindexer.getArtifactPositions()));
            panelsTelemetry.debug("State: " + stateMachine.getState());
            panelsTelemetry.debug("State auto: " + autoMachine.getState());
            panelsTelemetry.debug("Pose: " + follower.getPose());
            panelsTelemetry.update(telemetry);


            telemetry.update();
        }
    }
}

