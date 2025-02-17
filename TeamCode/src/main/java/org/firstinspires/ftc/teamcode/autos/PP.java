package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem.State.WALL_PICKUP_AUTO;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
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

    private final Pose startPose = new Pose(7, 64, Math.toRadians(0));
    private final Pose scorePose = new Pose(35, 72, Math.toRadians(0));
    private final Pose sample1GrabPose = new Pose(24, 38, Math.toRadians(-36));
    private final Pose sample1DepositPose = new Pose(24, 38, Math.toRadians(-126));
    private final Pose sample2GrabPose = new Pose(24, 28, Math.toRadians(-36));
    private final Pose sample2DepositPose = new Pose(24, 28, Math.toRadians(-126));
    private final Pose sample3GrabPose = new Pose(24, 18, Math.toRadians(-36));
    private final Pose sample3DepositPose = new Pose(24, 18, Math.toRadians(-126));
    private final Pose grabSamplePose = new Pose(24, 22, Math.toRadians(0));
    private final Pose score1Pose = new Pose(35, 68, Math.toRadians(0));



//    private final Pose pickup1Pose = new Pose(61.5, 23.8, Math.toRadians(0));
//    private final Pose push1Pose = new Pose(7, 24, Math.toRadians(0));
//    private final Pose pickup2Pose = new Pose(7, 16, Math.toRadians(0));
//    private final Pose pickup3Pose = new Pose(7, 9, Math.toRadians(0));
//    private final Pose subPose = new Pose(42, 67, Math.toRadians(0));
//    private final Pose pickupPose = new Pose(7, 32, Math.toRadians(0));

    private static Path scorePreload, pickup1, pickup2, deposit2, pickup3, deposit3, grab1, score1;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pickup1 = new Path(new BezierCurve(new Point(scorePose), new Point(sample1GrabPose)));
        pickup1.setLinearHeadingInterpolation(scorePose.getHeading(), sample1GrabPose.getHeading());

//        deposit1 = new Path(new BezierLine(new Point(sample1GrabPose), new Point(sample1DepositPose)));
//        deposit1.setLinearHeadingInterpolation(sample1GrabPose.getHeading(), sample1DepositPose.getHeading());

        pickup2 = new Path(new BezierCurve(new Point(sample1DepositPose), new Point(sample2GrabPose)));
        pickup2.setLinearHeadingInterpolation(sample1DepositPose.getHeading(), sample2GrabPose.getHeading());

//        deposit2 = new Path(new BezierLine(new Point(sample2GrabPose), new Point(sample2DepositPose)));
//        deposit2.setLinearHeadingInterpolation(sample2GrabPose.getHeading(), sample2DepositPose.getHeading());

        pickup3 = new Path(new BezierCurve(new Point(sample2DepositPose), new Point(sample3GrabPose)));
        pickup3.setLinearHeadingInterpolation(sample2DepositPose.getHeading(), sample3GrabPose.getHeading());

//        deposit3 = new Path(new BezierLine(new Point(sample3GrabPose), new Point(sample3DepositPose)));
//        deposit3.setLinearHeadingInterpolation(sample3GrabPose.getHeading(), sample3DepositPose.getHeading());

        grab1 = new Path(new BezierCurve(
                        new Point(sample3DepositPose),
                        new Point(new Pose(17, 30, Math.toRadians(0))),
                        new Point(grabSamplePose)));
        grab1.setLinearHeadingInterpolation(sample3DepositPose.getHeading(), grabSamplePose.getHeading());

        score1 = new Path(new BezierCurve(
                new Point(grabSamplePose),
                new Point(new Pose(13, 61, Math.toRadians(0))),
                new Point(score1Pose)));
        score1.setLinearHeadingInterpolation(grabSamplePose.getHeading(), score1Pose.getHeading());

//        pickup1 = new Path(new BezierCurve(
//                        new Point(scorePose),
//                        new Point(new Pose(10, 8, Math.toRadians(0))),
//                        new Point(new Pose(75, 58, Math.toRadians(0))),
//                        new Point(pickup1Pose)));
//        pickup1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading());
//
//        pickupToPush1 = new Path(new BezierLine(new Point(pickup1Pose), new Point(push1Pose)));
//        pickupToPush1.setLinearHeadingInterpolation(pickup1Pose.getHeading(), push1Pose.getHeading());
//
//        pickup2 = new Path(new BezierCurve(
//                new Point(push1Pose),
//                new Point(new Pose(63, 40.5, Math.toRadians(0))),
//                new Point(new Pose(78, 16, Math.toRadians(0))),
//                new Point(new Pose(62, 6, Math.toRadians(0))),
//                new Point(pickup2Pose)
//        ));
//        pickup2.setLinearHeadingInterpolation(push1Pose.getHeading(), pickup2Pose.getHeading());
//
//        pickup3 = new Path(new BezierCurve(
//                new Point(pickup2Pose),
//                new Point(new Pose(76.5, 32.5, Math.toRadians(0))),
//                new Point(new Pose(77.5, 0.4, Math.toRadians(0))),
//                new Point(pickup3Pose)
//        ));
//        pickup3.setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup3Pose.getHeading());
//
//        sub = new Path(new BezierLine(new Point(pickup3Pose), new Point(subPose)));
//        sub.setLinearHeadingInterpolation(pickup3Pose.getHeading(), subPose.getHeading());
//
//        grab = new Path(new BezierLine(new Point(subPose), new Point(pickupPose)));
//        grab.setLinearHeadingInterpolation(subPose.getHeading(), pickupPose.getHeading());
    }

    public void autonomousPathUpdate() {
        if (follower.isBusy())
            return;

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                prepArm();
                pathState += 1;
                break;
            case 1:
                claw.open();
                follower.followPath(pickup1);
                lowerArm();
                prepToGrabSample();
                startIntake();
                pathState += 1;
                break;
            case 2:
                follower.turnTo(sample1DepositPose.getHeading());
                returnToDriveMode();
                dropSample();
                pathState += 1;
                break;
            case 3:
                follower.followPath(pickup2);
                prepToGrabSample();
                startIntake();
                pathState += 1;
                break;
            case 4:
                follower.turnTo(sample2DepositPose.getHeading());
                returnToDriveMode();
                dropSample();
                pathState += 1;
                break;
            case 5:
                follower.followPath(pickup3);
                prepToGrabSample();
                startIntake();
                pathState += 1;
                break;
            case 6:
                follower.turnTo(sample3DepositPose.getHeading());
                returnToDriveMode();
                dropSample();
                pathState += 1;
                break;
            case 7:
                follower.followPath(grab1);
                wallIntake();
                pathState += 1;
                break;
            case 8:
                follower.followPath(score1);
                prepArm();
                pathState = 9;
                break;
            case 9:
                lowerArm();
                break;
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

    /** Prepares the intake sample */
    public void prepToGrabSample(){
        updateCommands(1.5);
        horiz.setState(HorizSubsystem.State.PREP_AUTO_INTAKE);
        pivot.setState(PivotSubsystem.State.PREPARE_INTAKE); //PREPARE_INTAKE_HIGHER
        rotate.setState(RotateSubsystem.State.INTAKE);
        updateCommands(0.25);
        claw.open();
        horiz.setState(HorizSubsystem.State.INTAKING_EXTENDED);
        rotate.setState(RotateSubsystem.State.SAMPLE);
        updateCommands(0.25);
    }
    /** Grabs sample */
    public void startIntake(){
        pivot.setState(PivotSubsystem.State.INTAKE);
        updateCommands(0.25);
        claw.close();
        updateCommands(0.25);

        //reattempt if fail
        if(!claw.isBlockInClaw()){
            claw.setColor(ClawSubsystem.ColorState.RED);

            pivot.setState(PivotSubsystem.State.PREPARE_INTAKE);
            claw.open();
            updateCommands(0.25);

            pivot.setState(PivotSubsystem.State.INTAKE);
            updateCommands(0.25);
            claw.close();
            updateCommands(0.25);

            if(claw.isBlockInClaw()) {
                claw.setColor(ClawSubsystem.ColorState.GREEN);
                updateCommands();
            }
        }

    }
    /** Returns to driving mode */
    public void returnToDriveMode(){
        rotate.setState(RotateSubsystem.State.NEUTRAL);
        pivot.setState(PivotSubsystem.State.DRIVING);
        horiz.setState(HorizSubsystem.State.DRIVING);
        updateCommands(0.25); //removed wait
        claw.setColor(ClawSubsystem.ColorState.OFF);
        updateCommands();
    }

    /** Drops sample at human **/

    public void dropSample(){
        horiz.setState(HorizSubsystem.State.INTAKING_EXTENDED);
        pivot.setState(PivotSubsystem.State.PREPARE_INTAKE); //PREPARE_INTAKE_HIGHER
        updateCommands(0.25);
        claw.open();
        updateCommands();
    }

    /** Prepares the intake for wall, opens claw, and closes */
    public void wallIntake(){
        //prep intake
        pivot.setState(PivotSubsystem.State.AUTO_INTAKE);
        horiz.setState(HorizSubsystem.State.DRIVING);
        updateCommands(0.25);
        rotate.setState(RotateSubsystem.State.PICKUP);
        claw.open();
        updateCommands(0.55);

        horiz.setState(WALL_PICKUP_AUTO);
        updateCommands(0.05);

        claw.close();
        updateCommands(0.15);
    }

    /** Prepares passive deposit */
    public void prepArm(){
        horiz.setState(HorizSubsystem.State.PASSIVE);
        vertical.setState(VerticalSubsystem.State.PASSIVE);
        updateCommands(1);

        pivot.setState(PivotSubsystem.State.PASSIVE);
        rotate.setState(RotateSubsystem.State.DROPOFF);
        updateCommands();
    }

    /** Deposits, and lowers arm */
    public void lowerArm(){
        claw.open();
        horiz.setState(HorizSubsystem.State.DRIVING);
        pivot.setState(PivotSubsystem.State.FINISH_PASSIVE);
        updateCommands(0.5);

        vertical.setState(VerticalSubsystem.State.BOTTOM);
        updateCommands(0.5);

        claw.close();
        pivot.setState(PivotSubsystem.State.DRIVING);
        updateCommands();

        pivot.setState(PivotSubsystem.State.PREPARE_PICKUP);
        horiz.setState(HorizSubsystem.State.INTAKE_WALL);
        updateCommands(0.5);
        rotate.setState(RotateSubsystem.State.PICKUP);
        pivot.setState(PivotSubsystem.State.PICKUP);
        claw.open();

    }
}

