package org.firstinspires.ftc.teamcode.autos;

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

    private final Pose pickupSubPose = new Pose(12, 35);
    private final Pose parkPose = new Pose(34, 70);


    private static Path scorePreload,
            deposit1, deposit2, deposit3, deposit4,
            grab1, grab2, grab3, grab4,
            park;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grab1 = new Path(new BezierCurve(new Point(scorePose), new Point(pickup1Pose)));
        grab1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading());

        deposit1 = new Path(new BezierCurve(new Point(pickup1Pose), new Point(scorePose)));
        deposit1.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading());

        grab2 = new Path(new BezierCurve(new Point(scorePose), new Point(pickup2Pose)));
        grab2.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading());

        deposit2 = new Path(new BezierCurve(new Point(pickup2Pose), new Point(scorePose)));
        deposit2.setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading());

        grab3 = new Path(new BezierCurve(new Point(scorePose), new Point(pickup3Pose)));
        grab3.setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading());

        deposit3 = new Path(new BezierCurve(new Point(pickup3Pose), new Point(scorePose)));
        deposit3.setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading());

        grab4 = new Path(new BezierCurve(new Point(scorePose), new Point(pickupSubPose)));
        grab4.setLinearHeadingInterpolation(scorePose.getHeading(), pickupSubPose.getHeading());

        deposit4 = new Path(new BezierCurve(new Point(pickupSubPose), new Point(scorePose)));
        deposit4.setLinearHeadingInterpolation(pickupSubPose.getHeading(), scorePose.getHeading());

        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        if (follower.isBusy())
            return;

        switch (pathState) {
            case 0:
                vertical.setSlidePower(true);
                follower.setMaxPower(1.0);
                follower.followPath(scorePreload);
//                prepArm();
                pathState += 1;
                return;
            case 1:
                follower.followPath(grab1);
//                lowerArm();
//                prepIntake();
                pathState += 1;
                return;
            case 2:
//                finishIntake();
                follower.followPath(deposit1);
//                prepArm();
                pathState += 1;
                return;
            case 3:
                follower.followPath(grab2);
//                lowerArm();
//                prepIntake();
                pathState += 1;
                return;
            case 4:
//                finishIntake();
                follower.followPath(deposit2);
//                prepArm();
                pathState += 1;
                return;
            case 5:
                follower.followPath(grab3);
//                lowerArm();
//                rotateIntake();
//                prepIntake();
                pathState += 1;
                return;
            case 6:
//                finishIntake();
                follower.followPath(deposit3);
//                prepArm();
                pathState += 1;
                return;
            case 7:
                follower.followPath(grab4);
//                lowerArm();
//                prepIntake();
                pathState += 1;
                return;
            case 8:
//                finishIntake();
                follower.followPath(deposit4);
//                prepArm();
                pathState += 1;
                return;
            case 12:
                
                follower.followPath(park);
//                lowerArm();
//                parkArm();
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
    /** Prepares the intake sample */
    public void prepIntake(){
        horiz.setState(HorizSubsystem.State.PREP_AUTO_INTAKE);
        pivot.setState(PivotSubsystem.State.PREPARE_INTAKE);
        rotate.setState(RotateSubsystem.State.INTAKE);
        updateCommands(0.25);
        claw.open();
        horiz.setState(HorizSubsystem.State.INTAKING_EXTENDED);
        updateCommands();
    }
    /** Rotates for third sample */
    public void rotateIntake(){
        rotate.setState(RotateSubsystem.State.SAMPLE);
        updateCommands();
    }
    /** Grabs sample */
    public void finishIntake(){
        pivot.setState(PivotSubsystem.State.INTAKE);
        updateCommands(0.25);
        claw.close();
        updateCommands(0.15);
        if(!claw.isBlockInClaw()){
            pivot.setState(PivotSubsystem.State.PREPARE_INTAKE);
            claw.open();
            updateCommands(0.25);

            finishIntake();
        }
        rotate.setState(RotateSubsystem.State.NEUTRAL);
        pivot.setState(PivotSubsystem.State.DRIVING);
        horiz.setState(HorizSubsystem.State.DRIVING);
        updateCommands(0.25);
    }

    /** Raises the Vertical */
    public void prepArm(){
        vertical.setDepositState(VerticalSubsystem.State.HIGH_BUCKET);
        vertical.setState(VerticalSubsystem.State.DEPOSITING);
        horiz.setState(HorizSubsystem.State.DRIVING);
        updateCommands(1);
        pivot.setState(PivotSubsystem.State.OUTTAKE_BUCKET);
        updateCommands(0.5);
        rotate.setState(RotateSubsystem.State.DROPOFF_BUCKET);
        updateCommands(0.25); // 0.5 -> 0.25
    }

    /** Deposits, and lowers arm */
    public void lowerArm(){
        claw.open();
        updateCommands(0.25);
        rotate.setState(RotateSubsystem.State.DROPOFF);
        pivot.setState(PivotSubsystem.State.DRIVING);
        claw.close();
        vertical.setState(VerticalSubsystem.State.BOTTOM);
        updateCommands(0.5);
    }

    public void parkArm(){
        pivot.setState(PivotSubsystem.State.PARK);
        horiz.setState(HorizSubsystem.State.PARK);
        updateCommands();
    }
}

