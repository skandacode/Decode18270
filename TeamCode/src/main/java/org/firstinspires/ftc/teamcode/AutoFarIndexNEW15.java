package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
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
@Autonomous(name = "AutoFarIndex15try", group = "Auto")
public class AutoFarIndexNEW15 extends LinearOpMode {
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
    public static double timeforspin = 0.3;
    public static double timeforkickerlast = 0.26;
    public static double timeforspinlast = 0.3;
    public static double timeForIntake = 0.15;
    public static double waitforkickerdown = 0.03;

    public static Shooter.Goal shooterTarget = Shooter.Goal.BLUE;

    public enum AutoStates {
        MOVETOSHOOT1, wait1, SHOOT1,
        MOVETOINTAKE1, INTAKE1,
        MOVETOSHOOT2, wait2, SHOOT2,
        MOVETOINTAKE2, INTAKE2, INTAKE2BACK, BACK, GATE, waitgate,
        MOVETOSHOOT3, wait3,SHOOT3,
        MOVETOINTAKE3, INTAKE3,
        MOVETOSHOOT4, wait4,SHOOT4,
        MOVETOINTAKE4, INTAKE4, INTAKE4BACK, INTAKE4BACKIN,
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
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
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
                telemetry.addLine("Don't see anything");
            }
            telemetry.addData("Pattern", pattern);
            telemetry.addData("Init Pose: ", follower.getPose());
            telemetry.addData("ALLIANCE: ", colorAlliance);
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
            telemetry.update();
        }

        shooter.kickerDown();
        waitForStart();


        Pose opengate = new Pose(2, -56*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose opengateback = new Pose(2, -50*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose startPose = new Pose(65, -24*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose shootPose = new Pose(-24, -24*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose shootPoselast = new Pose(60, -20*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose shootPoselastshot = new Pose(67, -30*Posmultiplier, Math.toRadians(-90*Posmultiplier));

        Pose intake1Pose = new Pose(-10, -22*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake2Pose = new Pose(14, -26*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake3Pose = new Pose(37,-26*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake4Pose = new Pose(62,-25*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake4donePose = new Pose(66, -61*Posmultiplier, Math.toRadians(-90*Posmultiplier));

        Pose intake1donePose = new Pose(-10, -53*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake2donePose = new Pose(14, -60*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake3donePose = new Pose(40, -60*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose leave = new Pose(60, -46*Posmultiplier, Math.toRadians(-90*Posmultiplier));


        PathChain toShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .setBrakingStrength(1.5)
                .build();

        PathChain toIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake1Pose.getHeading())
                .build();

        PathChain toIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake2Pose.getHeading())
                .setBrakingStrength(0.9)
                .build();

        PathChain toIntake1back = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, opengateback))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), opengateback.getHeading())
                .build();

        PathChain intake2ToGate= follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, opengate))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), opengate.getHeading())
                .build();

        PathChain toIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake3Pose.getHeading())
                .setBrakingStrength(0.8)
                .build();
        PathChain toIntake4 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake4Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake4Pose.getHeading())
                .build();

        PathChain toIntake1fin = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, intake1donePose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), intake1donePose.getHeading())
                .build();

        PathChain toIntake2fin = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, intake2donePose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), intake2donePose.getHeading())
                .build();

        PathChain toIntake2back= follower.pathBuilder()
                .addPath(new BezierLine(intake2donePose, intake2Pose))
                .setLinearHeadingInterpolation(intake2donePose.getHeading(), intake2Pose.getHeading())
                .build();

        PathChain toIntake3fin = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, intake3donePose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), intake3donePose.getHeading())
                .build();
        PathChain toIntake4fin = follower.pathBuilder()
                .addPath(new BezierLine(intake4Pose, intake4donePose))
                .setLinearHeadingInterpolation(intake4Pose.getHeading(), intake4donePose.getHeading())
                .build();
        PathChain toIntake4back = follower.pathBuilder()
                .addPath(new BezierLine(intake4donePose, intake4Pose))
                .setLinearHeadingInterpolation(intake4donePose.getHeading(), intake4Pose.getHeading())
                .build();

        PathChain toScore1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1donePose, shootPose))
                .setLinearHeadingInterpolation(intake1donePose.getHeading(), shootPose.getHeading())
                .setBrakingStrength(0.8)
                .build();

        PathChain toScore2 = follower.pathBuilder()
                .addPath(new BezierLine(opengate, shootPose))
                .setLinearHeadingInterpolation(opengate.getHeading(), shootPose.getHeading())
                .setBrakingStrength(0.9)
                .build();

        PathChain toScore3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3donePose, shootPoselast))
                .setLinearHeadingInterpolation(intake3donePose.getHeading(), shootPoselast.getHeading())
                .build();

        PathChain park = follower.pathBuilder()
                .addPath(new BezierLine(shootPoselast, leave))
                .setLinearHeadingInterpolation(shootPoselast.getHeading(), leave.getHeading())
                .build();


        follower.setStartingPose(startPose);


        StateMachine stateMachine = new StateMachineBuilder()
                .state(RobotState.Intake1)
                .onEnter(() -> spindexer.intakePos(0))
                .transition(() -> intake.isIntaked())
                .transition(() -> shooterButtonAll)


                .state(RobotState.wait1)
                .onEnter(() -> {
                    spindexer.intakePos(1);
                    spindexer.update();
                    spindexer.afterIntake(intake.getArtifact());
                })
                .transitionTimed(timeForIntake)
                .transition(() -> shooterButtonAll)


                .state(RobotState.Intake2)
                .onEnter(() -> spindexer.intakePos(1))
                .transition(() -> intake.isIntaked())
                .transition(() -> shooterButtonAll)


                .state(RobotState.wait2)
                .onEnter(() -> {
                    spindexer.intakePos(2);
                    spindexer.update();
                    spindexer.afterIntake(intake.getArtifact());
                })
                .transitionTimed(timeForIntake)
                .transition(() -> shooterButtonAll)


                .state(RobotState.Intake3)
                .onEnter(() -> spindexer.intakePos(2))
                .transition(() -> intake.isIntaked())
                .transition(() -> shooterButtonAll)


                .state(RobotState.wait3)
                .onEnter(() -> {
                    spindexer.shootPos(0);
                    spindexer.update();
                    spindexer.afterIntake(intake.getArtifact());
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
                .transitionTimed(timeforspinlast)


                .state(RobotState.Shoot3)
                .onEnter(() -> shooter.kickerUp())
                .transitionTimed(timeforkickerlast)
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
                    shooter.setHood(0.68);
                    shooter.setTargetVelocity(1050);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-40*Posmultiplier));
                    if (pattern==1){
                        shootorder = new int[]{0, 1, 2};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT1);
                    }else if (pattern==2){
                        shootorder = new int[]{2, 0, 1};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT3);
                    }else{
                        shootorder = new int[]{1, 2, 0};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT2);

                    }
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.8)
                .state(AutoStates.wait1)
                .onEnter(()->{

                })
                .transitionTimed(0)
                .state(AutoStates.SHOOT1)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(1.4)
                .transition(()->stateMachine.getStateEnum() == RobotState.spin3)

                .state(AutoStates.MOVETOINTAKE1)
                .onEnter(()->{
                    follower.followPath(toIntake1, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.7)

                .state(AutoStates.INTAKE1)
                .onEnter(()->{
                    follower.followPath(toIntake1fin, 0.9, true);
                })
                .transitionTimed(1.3)
                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(()->{
                    intake.setPower(1);
                    follower.followPath(toScore1, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)

                .state(AutoStates.wait2)
                .onEnter(()->{
                    if (pattern==1){
                        shootorder = new int[]{2, 0, 1};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT3);
                    }else if (pattern==2){
                        shootorder = new int[]{1, 2, 0};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT2);
                    }else{
                        shootorder = new int[]{0, 1, 2};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT1);
                    }
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT2)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(1.7)
                .transition(()->stateMachine.getStateEnum() == RobotState.spin3)
                .state(AutoStates.MOVETOINTAKE2)
                .onEnter(()->{
                    follower.followPath(toIntake2, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1)

                .state(AutoStates.INTAKE2)
                .onEnter(()->{
                    follower.followPath(toIntake2fin, 0.9, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.8)

                .state(AutoStates.INTAKE2BACK)
                .onEnter(()->{
                    follower.followPath(toIntake2back, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.2)
                .state(AutoStates.GATE)
                .onEnter(()->{
                    follower.followPath(intake2ToGate, 1, true);
                    intake.setPower(0);
                })
                .transitionTimed(0.3)
                .state(AutoStates.waitgate)
                .onEnter(()->{
                    intake.setPower(0);
                })
                .transitionTimed(2)
                .state(AutoStates.MOVETOSHOOT3)
                .onEnter(()->{
                    follower.followPath(toScore2, true);
                    intake.setPower(1);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.6)

                .state(AutoStates.wait3)
                .onEnter(()->{
                    if (pattern==1){
                        shootorder = new int[]{1, 2, 0};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT2);
                    }else if (pattern==2){
                        shootorder = new int[]{0, 1, 2};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT1);
                    }else{
                        shootorder = new int[]{2, 0, 1};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT3);
                    }
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT3)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(2)
                .transition(()->stateMachine.getStateEnum() == RobotState.spin3)

                .state(AutoStates.MOVETOINTAKE3)
                .onEnter(()->{
                    follower.followPath(toIntake3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.INTAKE3)
                .onEnter(()->{
                    shooter.aimAtTarget(shootPoselastshot,shooterTarget);
                    follower.followPath(toIntake3fin, 0.7, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.55)
                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{
                    follower.followPath(toScore3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.7)
                .state(AutoStates.wait4)
                .onEnter(()->{
                    if (pattern==1){
                        shootorder = new int[]{0, 1, 2};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT1);
                    }else if (pattern==2){
                        shootorder = new int[]{2, 0, 1};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT3);
                    }else{
                        shootorder = new int[]{1, 2, 0};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT2);

                    }
                    shooter.setHood(0.84);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-108 * Posmultiplier));
                    shooter.setTargetVelocity(1490);                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT4)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(1.55)
                .transition(()->stateMachine.getStateEnum() == RobotState.spin3)
                .state(AutoStates.MOVETOINTAKE4)
                .onEnter(()->{
                    follower.followPath(toIntake4, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.2)
                .state(AutoStates.INTAKE4)
                .onEnter(()->{
                    follower.followPath(toIntake4fin, 0.8, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.7)
                .state(AutoStates.INTAKE4BACK)
                .onEnter(()->{
                    follower.followPath(toIntake4back, 1, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.3)
                .state(AutoStates.INTAKE4BACKIN)
                .onEnter(()->{
                    follower.followPath(toIntake4fin, 0.8, true);
                })
                .transition(()->stateMachine.getStateEnum() == RobotState.WaitForShoot)
                .transition(()->stateMachine.getStateEnum()== RobotState.reverseIntake)
                .transitionTimed(1)
                .state(AutoStates.MOVETOSHOOT5)
                .onEnter(()->{
                    PathChain toScore4 = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPoselast))
                            .setLinearHeadingInterpolation(follower.getHeading(), shootPoselast.getHeading())
                            .build();
                    follower.followPath(toScore4, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.wait5)
                .onEnter(()->{
                    if (pattern==1){
                        shootorder = new int[]{0, 1, 2};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT1);
                    }else if (pattern==2){
                        shootorder = new int[]{2, 0, 1};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT3);
                    }else{
                        shootorder = new int[]{1, 2, 0};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT2);

                    }
                    shooter.setHood(0.84);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-108 * Posmultiplier));
                    shooter.setTargetVelocity(1490);
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT5)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(2)
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
            telemetry.addData("Artifact colors", Arrays.toString(spindexer.getArtifactPositions()));
            telemetry.addData("State: ", stateMachine.getState());
            telemetry.addData("State auto: ", autoMachine.getState());
            telemetry.addData("Pose: ", follower.getPose());
            telemetry.update();
        }
    }
}

