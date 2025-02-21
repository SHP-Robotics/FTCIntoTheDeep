package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;


@Autonomous(name = "*** 5 + 0 SPECIMEN***")
public class FiveSpecimen extends OpMode {
    VerticalSubsystem vertical;
    PivotSubsystem pivot;
    RotateSubsystem rotate;
    HorizSubsystem horiz;
    ClawSubsystem claw;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private ElapsedTime elapsedTime, autoTime;


    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    private final Pose startPose = new Pose(7, 61);
    private final Pose scorePose = new Pose(35, 72);
    private final Pose pickup1Pose = new Pose(30, 24);
    private final Pose pickup2Pose = new Pose(30, 16);
    private final Pose pickup3Pose = new Pose(11, 10);

    private final Pose pickupPose = new Pose(12, 35);
    private final Pose deposit1Pose = new Pose(34, 70);
    private final Pose deposit2Pose = new Pose(34, 69);
    private final Pose deposit3Pose = new Pose(34, 68);
    private final Pose deposit4Pose = new Pose(34, 67);


    private static Path scorePreload, deposit1, deposit2, deposit3, deposit4, park;
    private static PathChain pushChain, grab2, grab3, grab4;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pushChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose),
                        new Point(new Pose(16, 4)),
                        new Point(new Pose(75, 50)),
                        new Point(new Pose(44, 28)),
                        new Point(new Pose(76, 16)),
                        new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierCurve(
                        new Point(pickup1Pose),
                        new Point(new Pose(60, 24)),
                        new Point(new Pose(61, 28)),
                        new Point(new Pose(62, 6)),
                        new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup2Pose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierCurve(
                        new Point(pickup2Pose),
                        new Point(new Pose(68, 14)),
                        new Point(new Pose(58.5, 21)),
                        new Point(new Pose(64, 10)),
                        new Point(new Pose(64, 10)),
                        new Point(new Pose(63, 8)),
                        new Point(new Pose(44, 11)),
                        new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup3Pose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        deposit1 = new Path(new BezierCurve(new Point(pickup3Pose),
                new Point(new Pose(21, 66)),
                new Point(deposit1Pose)));
        deposit1.setLinearHeadingInterpolation(pickup3Pose.getHeading(), deposit1Pose.getHeading());

        grab2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(deposit1Pose),
                        new Point(new Pose(9, 48)),
                        new Point(new Pose(23, 33)),
                        new Point(pickupPose)))
                .setLinearHeadingInterpolation(deposit1Pose.getHeading(), pickupPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        deposit2 = new Path(new BezierCurve(new Point(pickupPose),
                new Point(new Pose(21, 66)),
                new Point(deposit2Pose)));
        deposit2.setLinearHeadingInterpolation(pickupPose.getHeading(), deposit2Pose.getHeading());

        grab3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(deposit2Pose),
                        new Point(new Pose(9, 48)),
                        new Point(new Pose(23, 33)),
                        new Point(pickupPose)))
                .setLinearHeadingInterpolation(deposit2Pose.getHeading(), pickupPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        deposit3 = new Path(new BezierCurve(new Point(pickupPose),
                new Point(new Pose(21, 66)),
                new Point(deposit3Pose)));
        deposit3.setLinearHeadingInterpolation(pickupPose.getHeading(), deposit3Pose.getHeading());

        grab4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(deposit3Pose),
                        new Point(new Pose(9, 48)),
                        new Point(new Pose(23, 33)),
                        new Point(pickupPose)))
                .setLinearHeadingInterpolation(deposit3Pose.getHeading(), pickupPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        deposit4 = new Path(new BezierCurve(new Point(pickupPose),
                new Point(new Pose(21, 66)),
                new Point(deposit4Pose)));
        deposit4.setLinearHeadingInterpolation(pickupPose.getHeading(), deposit4Pose.getHeading());

        park = new Path(new BezierCurve(new Point(deposit4Pose), new Point(pickupPose)));
        park.setLinearHeadingInterpolation(deposit4Pose.getHeading(), pickupPose.getHeading());
    }

    public void autonomousPathUpdate() {
        if (follower.isBusy())
            return;

        switch (pathState) {
            case 0:
                //deposits preload
                vertical.setSlidePower(true);
                follower.setMaxPower(0.775);
                follower.followPath(scorePreload);
                prepArm();
                pathState += 1;
                return;
            case 1:
                //finishes deposit and pushes to sample 1
                follower.setMaxPower(1.0);
                claw.open();
                follower.followPath(pushChain, true);
                lowerArm(false);
                pathState = 5;
                return;
            case 5:
                follower.setMaxPower(1.0);
                finishWallIntake();
                follower.followPath(deposit1);
                prepArm();
                pathState += 1;
                return;
            case 6:
                follower.setMaxPower(0.9);
                claw.open();
                follower.followPath(grab2);
                lowerArm(false);
                pathState += 1;
                return;
            case 7:
                follower.setMaxPower(0.9);
                finishWallIntake();
                follower.followPath(deposit2);
                prepArm();
                pathState += 1;
                return;
            case 8:
                follower.setMaxPower(0.9);
                claw.open();
                follower.followPath(grab3);
                lowerArm(false);
                pathState += 1;
                return;
            case 9:
                follower.setMaxPower(0.9);
                finishWallIntake();
                follower.followPath(deposit3);
                prepArm();
                pathState += 1;
                return;
            case 10:
                follower.setMaxPower(0.9);
                claw.open();
                follower.followPath(grab4);
                lowerArm(false);
                pathState += 1;
                return;
            case 11:
                follower.setMaxPower(0.9);
                finishWallIntake();
                if(autoTime.seconds() < 27) {
                    follower.followPath(deposit4);
                    prepArm();
                    pathState += 1;
                }
                return;
            case 12:
                follower.setMaxPower(1.0);
                claw.open();
                follower.followPath(park);
                lowerArm(true);
                pathState = -1;
                return;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        try {
            CommandScheduler.getInstance().run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        CommandScheduler.resetInstance();

        // Initialize your subsystems and devices
        vertical = new VerticalSubsystem(hardwareMap);
        rotate = new RotateSubsystem(hardwareMap);
        pivot = new PivotSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);
        horiz = new HorizSubsystem(hardwareMap);

        claw.close();
        pivot.setState(PivotSubsystem.State.START_SPEC_AUTO);
        pivot.processState();

        rotate.setState(RotateSubsystem.State.DROPOFF);
        rotate.processState();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setHeadingOffset(Math.toRadians(0));
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

        Clock.start();
        CommandScheduler.getInstance().setTelemetry(telemetry);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        autoTime = new ElapsedTime();
        autoTime.reset();
    }

    @Override
    public void stop() {
    }


    public void updateCommands(){
        try {
            CommandScheduler.getInstance().run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void updateCommands(double sec){
        elapsedTime.reset();
        while (elapsedTime.seconds() < sec) {
            follower.update();
            updateCommands();
        }
    }

    public void finishWallIntake(){
        horiz.setState(HorizSubsystem.State.DRIVING);
        updateCommands(0.15);
        claw.close();
        updateCommands(0.25);
        if(!claw.isBlockInClaw()){
            claw.open();
            horiz.setState(HorizSubsystem.State.WALL_PICKUP_AUTO);
            updateCommands(0.5);

            finishWallIntake();
        }
        pivot.setState(PivotSubsystem.State.PASSIVE);
        updateCommands();
    }

    /** Prepares passive deposit */
    public void prepArm(){
        horiz.setState(HorizSubsystem.State.PASSIVE);
        vertical.setState(VerticalSubsystem.State.PASSIVE);
        updateCommands(0.5);

        pivot.setState(PivotSubsystem.State.PASSIVE);
        rotate.setState(RotateSubsystem.State.DROPOFF);
        updateCommands();
    }

    /** Deposits, and lowers arm */
    public void lowerArm(boolean done){
        claw.open();
        horiz.setState(HorizSubsystem.State.DRIVING);
        pivot.setState(PivotSubsystem.State.FINISH_PASSIVE);
        updateCommands(0.25);

        vertical.setState(VerticalSubsystem.State.BOTTOM);
        updateCommands(0.05);

        if(!done) {
            pivot.setState(PivotSubsystem.State.PICKUP);
            horiz.setState(HorizSubsystem.State.WALL_PICKUP_AUTO);
            updateCommands(0.05);
            rotate.setState(RotateSubsystem.State.PICKUP);
            claw.open();
            updateCommands();
        }
        else{
            pivot.setState(PivotSubsystem.State.DRIVING);
            horiz.setState(HorizSubsystem.State.DRIVING);
            updateCommands();
        }
    }
}