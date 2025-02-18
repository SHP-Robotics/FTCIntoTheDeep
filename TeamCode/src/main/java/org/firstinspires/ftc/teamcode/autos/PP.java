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


@Autonomous(name = "*** Not William's PP ***")
public class PP extends OpMode {
    VerticalSubsystem vertical;
    PivotSubsystem pivot;
    RotateSubsystem rotate;
    HorizSubsystem horiz;
    ClawSubsystem claw;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private ElapsedTime elapsedTime;


    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    private final Pose startPose = new Pose(7, 61);
    private final Pose scorePose = new Pose(36, 72);
//    private final Pose sample1GrabPose = new Pose(26, 39.5, Math.toRadians(-36));
//    private final Pose sample1DepositPose = new Pose(26, 39.5, Math.toRadians(-156));
//    private final Pose sample2GrabPose = new Pose(26, 29.75, Math.toRadians(-36));
//    private final Pose sample2DepositPose = new Pose(26, 29.75, Math.toRadians(-156));
//    private final Pose sample3GrabPose = new Pose(26, 22, Math.toRadians(-36));
//    private final Pose sample3DepositPose = new Pose(26, 22, Math.toRadians(-156));
//    private final Pose grabSamplePose = new Pose(7, 30);
//    private final Pose score1Pose = new Pose(35, 68);



    private final Pose pickup1Pose = new Pose(30, 24);
    private final Pose pickup2Pose = new Pose(30, 16);
    private final Pose pickup3Pose = new Pose(10, 10);
    private final Pose pickupPose = new Pose(13, 37);
    private final Pose deposit1Pose = new Pose(36, 68);
    private final Pose deposit2Pose = new Pose(36, 68);
    private final Pose deposit3Pose = new Pose(36, 68);
    private final Pose deposit4Pose = new Pose(36, 68);


    private static Path scorePreload, pickup1, pickup2, pickup3, grab1, deposit1, grab2, deposit2, grab3, deposit3, grab4, deposit4, park;
    private static PathChain pushChain;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

//        pickup1 = new Path(new BezierCurve(
//                new Point(scorePose),
//                new Point(new Pose(15, 72)),
//                new Point(sample1GrabPose)
//        ));
//        pickup1.setLinearHeadingInterpolation(scorePose.getHeading(), sample1GrabPose.getHeading());
//
////        deposit1 = new Path(new BezierLine(new Point(sample1GrabPose), new Point(sample1DepositPose)));
////        deposit1.setLinearHeadingInterpolation(sample1GrabPose.getHeading(), sample1DepositPose.getHeading());
//
//        pickup2 = new Path(new BezierCurve(new Point(sample1DepositPose), new Point(sample2GrabPose)));
//        pickup2.setLinearHeadingInterpolation(sample1DepositPose.getHeading(), sample2GrabPose.getHeading());
//
////        deposit2 = new Path(new BezierLine(new Point(sample2GrabPose), new Point(sample2DepositPose)));
////        deposit2.setLinearHeadingInterpolation(sample2GrabPose.getHeading(), sample2DepositPose.getHeading());
//
//        pickup3 = new Path(new BezierCurve(new Point(sample2DepositPose), new Point(sample3GrabPose)));
//        pickup3.setLinearHeadingInterpolation(sample2DepositPose.getHeading(), sample3GrabPose.getHeading());
//
////        deposit3 = new Path(new BezierLine(new Point(sample3GrabPose), new Point(sample3DepositPose)));
////        deposit3.setLinearHeadingInterpolation(sample3GrabPose.getHeading(), sample3DepositPose.getHeading());
//
//        grab1 = new Path(new BezierCurve(
//                        new Point(sample3DepositPose),
//                        new Point(new Pose(17, 30)),
//                        new Point(grabSamplePose)));
//        grab1.setLinearHeadingInterpolation(sample3DepositPose.getHeading(), grabSamplePose.getHeading());
//
//        score1 = new Path(new BezierCurve(
//                new Point(grabSamplePose),
//                new Point(new Pose(13, 61)),
//                new Point(score1Pose)));
//        score1.setLinearHeadingInterpolation(grabSamplePose.getHeading(), score1Pose.getHeading());
//
//        pickup1 = new Path(new BezierCurve(
//                        new Point(scorePose),
//                        new Point(new Pose(16, 4)),
//                        new Point(new Pose(75, 50)),
//                        new Point(new Pose(44, 28)),
//                        new Point(new Pose(76, 16)),
//                        new Point(pickup1Pose)));
//        pickup1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading());
//
//        pickup2 = new Path(new BezierCurve(
//                new Point(pickup1Pose),
//                new Point(new Pose(60, 24)),
//                new Point(new Pose(61, 28)),
//                new Point(new Pose(62, 6)),
//                new Point(pickup2Pose)
//        ));
//        pickup2.setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup2Pose.getHeading());
//
//        pickup3 = new Path(new BezierCurve(
//                new Point(pickup2Pose),
//                new Point(new Pose(75, 25)),
//                new Point(new Pose(58.5, 21)),
//                new Point(new Pose(57, 10)),
//                new Point(new Pose(57, 10)),
//                new Point(new Pose(74, 10)),
//                new Point(pickup3Pose)
//        ));
//        pickup3.setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup3Pose.getHeading());

        pushChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose),
                        new Point(new Pose(16, 4)),
                        new Point(new Pose(75, 50)),
                        new Point(new Pose(44, 28)),
                        new Point(new Pose(76, 16)),
                        new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierCurve(
                        new Point(pickup1Pose),
                        new Point(new Pose(60, 24)),
                        new Point(new Pose(61, 28)),
                        new Point(new Pose(62, 6)),
                        new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierCurve(
                        new Point(pickup2Pose),
                        new Point(new Pose(75, 25)),
                        new Point(new Pose(58.5, 21)),
                        new Point(new Pose(57, 10)),
                        new Point(new Pose(57, 10)),
                        new Point(new Pose(74, 10)),
                        new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup3Pose.getHeading())
                .setPathEndTimeoutConstraint(0.5)
                .build();

        deposit1 = new Path(new BezierCurve(new Point(pickup3Pose),
                new Point(new Pose(21, 66)),
                new Point(deposit1Pose)));
        deposit1.setLinearHeadingInterpolation(pickup3Pose.getHeading(), deposit1Pose.getHeading());

        grab2 = new Path(new BezierCurve(new Point(deposit1Pose),
//                new Point(new Pose(24, 68)),
//                new Point(new Pose(28, 29)),
                new Point(pickupPose)));
        grab2.setLinearHeadingInterpolation(deposit1Pose.getHeading(), pickupPose.getHeading());

        deposit2 = new Path(new BezierCurve(new Point(pickupPose),
                new Point(new Pose(21, 66)),
                new Point(deposit2Pose)));
        deposit2.setLinearHeadingInterpolation(pickupPose.getHeading(), deposit2Pose.getHeading());

        grab3 = new Path(new BezierCurve(new Point(deposit2Pose),
//                new Point(new Pose(24, 68)),
//                new Point(new Pose(28, 29)),
                new Point(pickupPose)));
        grab3.setLinearHeadingInterpolation(deposit2Pose.getHeading(), pickupPose.getHeading());

        deposit3 = new Path(new BezierCurve(new Point(pickupPose),
                new Point(new Pose(21, 66)),
                new Point(deposit3Pose)));
        deposit3.setLinearHeadingInterpolation(pickupPose.getHeading(), deposit3Pose.getHeading());

        grab4 = new Path(new BezierCurve(new Point(deposit3Pose),
//                new Point(new Pose(24, 68)),
//                new Point(new Pose(28, 29)),
                new Point(pickupPose)));
        grab4.setLinearHeadingInterpolation(deposit3Pose.getHeading(), pickupPose.getHeading());

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
                follower.setMaxPower(0.7);
                follower.followPath(scorePreload);
                prepArm();
                pathState += 1;
                return;
            case 1:
                //finishes deposit and pushes to sample 1
                follower.setMaxPower(1.0);
                claw.open();
                follower.followPath(pushChain, true);
                lowerArm();
                pathState = 5;
                return;
//            case 2:
//                //pushes sample 2
//                follower.followPath(pickup2);
//                pathState += 1;
//                return;
//            case 3:
//                //pushes sample 3
//                follower.followPath(pickup3);
//                pathState += 1;
//                return;

            case 5:
                follower.setMaxPower(0.9);
                finishWallIntake();
                follower.followPath(deposit1);
                prepArm();
                pathState += 1;
                return;
            case 6:
                follower.setMaxPower(1.0);
                claw.open();
                follower.followPath(grab2);
                lowerArm();
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
                follower.setMaxPower(1.0);
                claw.open();
                follower.followPath(grab3);
                lowerArm();
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
                follower.setMaxPower(1.0);
                claw.open();
                follower.followPath(grab4);
                lowerArm();
                pathState += 1;
                return;
            case 11:
                follower.setMaxPower(0.9);
                finishWallIntake();
                follower.followPath(deposit4);
                prepArm();
                pathState += 1;
                return;
            case 12:
                follower.setMaxPower(1.0);
                claw.open();
                follower.followPath(park);
                lowerArm();
                pathState = -1;
                return;
//            case 4:
//                follower.turnTo(sample2DepositPose.getHeading());
//                returnToDriveMode();
//                dropSample();
//                pathState += 1;
//                return;
//            case 5:
//                follower.followPath(pickup3);
//                prepToGrabSample();
//                startIntake();
//                pathState += 1;
//                return;
//            case 6:
//                follower.turnTo(sample3DepositPose.getHeading());
//                returnToDriveMode();
//                dropSample();
//                pathState += 1;
//                return;
//            case 7:
//                follower.followPath(grab1);
//                wallIntake();
//                pathState += 1;
//                return;
//            case 8:
//                follower.followPath(score1);
//                prepArm();
//                pathState = 9;
//                return;
//            case 9:
//                lowerArm();
//                return;
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
        follower.setHeadingOffset(Math.toRadians(0)); // TODO: check if I am high
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

    /** Prepares the intake for wall, opens claw, and closes */
    public void wallIntake(){
        //prep intake
        pivot.setState(PivotSubsystem.State.PICKUP);
        horiz.setState(HorizSubsystem.State.WALL_PICKUP_AUTO);
        updateCommands(0.15);
        rotate.setState(RotateSubsystem.State.PICKUP);
        claw.open();
        updateCommands(0.25);
    }
    public void finishWallIntake(){
        horiz.setState(HorizSubsystem.State.DRIVING);
        updateCommands(0.15);

        claw.close();
        updateCommands(0.15);
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
    public void lowerArm(){
        claw.open();
        horiz.setState(HorizSubsystem.State.DRIVING);
        pivot.setState(PivotSubsystem.State.FINISH_PASSIVE);
        updateCommands(0.25);

        vertical.setState(VerticalSubsystem.State.BOTTOM);
        updateCommands(0.5);

        pivot.setState(PivotSubsystem.State.PICKUP);
        horiz.setState(HorizSubsystem.State.WALL_PICKUP_AUTO);
        updateCommands(0.15);
        rotate.setState(RotateSubsystem.State.PICKUP);
        claw.open();
        updateCommands();
    }
}

