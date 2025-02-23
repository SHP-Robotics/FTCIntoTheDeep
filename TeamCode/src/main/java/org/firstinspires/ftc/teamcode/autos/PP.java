package org.firstinspires.ftc.teamcode.autos;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DetectSample;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;

import java.util.ArrayList;

@Autonomous(name = "*** 0 + 5 Sample ***")
public class PP extends LinearOpMode {
    VerticalSubsystem vertical;
    PivotSubsystem pivot;
    RotateSubsystem rotate;
    HorizSubsystem horiz;
    ClawSubsystem claw;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    public ArrayList<Pose2D> positions;
    public DetectSample detectSample;
    Pose2D lastDetection;
    PIDFController transPID;
    public static double kp = 0.0015;
    public static double kd = 0;
    MecanumController mecanumController;
    DeterministicTracker tracker;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private ElapsedTime elapsedTime;
    private ElapsedTime clawAlignment;


    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    private final Pose startPose = new Pose(7, 103, Math.toRadians(270));
    private final Pose preloadScorePose = new Pose(11.5, 130, Math.toRadians(315));

    private final Pose scorePose = new Pose(14.5, 131, Math.toRadians(315));
    private final Pose pickup1Pose = new Pose(21, 121, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(21, 131, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(23.5, 132, Math.toRadians(22.5));
    private final Pose pickupSubPose = new Pose(60, 100, Math.toRadians(270));
    private final Pose parkPose = new Pose(65, 92, Math.toRadians(270));

    private static Path scorePreload,
            deposit1, deposit2, deposit3, deposit4,
            grab1, grab2, grab3, grab4,
            park;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(preloadScorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), preloadScorePose.getHeading());

        grab1 = new Path(new BezierCurve(new Point(preloadScorePose), new Point(pickup1Pose)));
        grab1.setLinearHeadingInterpolation(preloadScorePose.getHeading(), pickup1Pose.getHeading());

        deposit1 = new Path(new BezierCurve(new Point(pickup1Pose), new Point(scorePose)));
        deposit1.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading());

        grab2 = new Path(new BezierCurve(new Point(scorePose), new Point(pickup2Pose)));
        grab2.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading());

        deposit2 = new Path(new BezierCurve(
                new Point(pickup2Pose),
                new Point(16, 125),
                new Point(scorePose)));
        deposit2.setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading());

        grab3 = new Path(new BezierCurve(new Point(scorePose), new Point(pickup3Pose)));
        grab3.setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading());

        deposit3 = new Path(new BezierCurve(
                new Point(pickup3Pose),
                new Point(12, 122),
                new Point(scorePose)));
        deposit3.setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading());

        grab4 = new Path(new BezierCurve(
                new Point(scorePose),
                new Point(64, 110),
                new Point(60, 100),
                new Point(pickupSubPose)));
        grab4.setLinearHeadingInterpolation(scorePose.getHeading(), pickupSubPose.getHeading());

        deposit4 = new Path(new BezierCurve(
                new Point(pickupSubPose),
                new Point(48, 137),
                new Point(18, 118),
                new Point(scorePose)));
        deposit4.setLinearHeadingInterpolation(pickupSubPose.getHeading(), scorePose.getHeading());

        park = new Path(new BezierCurve(
                new Point(scorePose),
                new Point(62, 114),
                new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        if (follower.isBusy())
            return;

        switch (pathState) {
            case 0: //Deposit preload
                vertical.setSlidePower(true);
                follower.setMaxPower(0.8);

                prepArm();
                follower.followPath(scorePreload);

                pathState += 1;
                return;
            case 1:
                if(vertical.getSlidePosition() > 2700) {
                    follower.setMaxPower(0.7);

                    lowerArm();
                    follower.followPath(grab1);
                    updateCommands(1);
                    prepIntake(false);
                    pathState += 1;
                }
                return;
            case 2:
            case 5:
            case 8:
                finishIntake();
                pathState += 1;
                return;
            case 3: //deposit 1
                follower.setMaxPower(0.8);

                prepArm();
                follower.followPath(deposit1);

                pathState += 1;
                return;
            case 4:
                if(vertical.getSlidePosition() > 2700) {
                    follower.setMaxPower(0.7);

                    lowerArm();
                    follower.followPath(grab2);
                    updateCommands(1);
                    prepIntake(false);
                    pathState += 1;
                }
                return;
            case 6: //deposit 2
                follower.setMaxPower(0.7);
                prepArm();
                follower.followPath(deposit2);

                pathState += 1;
                return;
            case 7:
                if(vertical.getSlidePosition() > 2700) {
                    follower.setMaxPower(0.7);
                    lowerArm();
                    follower.followPath(grab3);
                    rotateIntake();
                    updateCommands(1);
                    prepIntake(false);
                    pathState += 1;
                }
                return;
            case 9:
                follower.setMaxPower(0.7);
                prepArm();
                follower.followPath(deposit3);

                pathState += 1;
                return;
            case 10:
                if(vertical.getSlidePosition() > 2700) {
                    follower.setMaxPower(1.0);
                    lowerArm();
                    rotateIntake();
                    follower.followPath(grab4);
                    updateCommands(1);
                    prepIntake(true);
                    pathState += 2;
                }
                return;
            case 12:
                while(!autoRotateIntake()) {
                    if(rotate.aligned)
                        claw.setColor(ClawSubsystem.ColorState.GREEN);
                    else
                        claw.setColor(ClawSubsystem.ColorState.RED);

                    if(opmodeTimer.getElapsedTimeSeconds() > 29) {
                        pathState = 15;
                        return;
                    }
                }
                finishIntake();
                pathState += 1;
                return;
            case 13:
                if(claw.isBlockInClaw()) {
                    follower.followPath(deposit4);
                    updateCommands(0.5);
                    if (opmodeTimer.getElapsedTimeSeconds() < 27)
                        prepArm();
                }
                pathState += 1;
                return;
            case 14:
                if(vertical.getSlidePosition() > 2700 || opmodeTimer.getElapsedTimeSeconds() < 29) {
                    lowerArm();
                }
                follower.followPath(park);
                parkArm();
                pathState = -1;
                return;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() {
        // TODO: INIT
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

        detectSample = new DetectSample(hardwareMap, telemetry);

        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);

        lastDetection = new Pose2D(0,0,0);
        clawAlignment = new ElapsedTime();

        transPID = new PIDFController(new PIDCoefficients(
                kp, 0, kd
        ));

        // TODO: START
        setPathState(0);

        Clock.start();
        CommandScheduler.getInstance().setTelemetry(telemetry);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        opmodeTimer.resetTimer();

        // TODO: LOOP

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            autonomousPathUpdate();

            CommandScheduler.updateCommands();

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

    public void updateCommands(double sec){
        elapsedTime.reset();
        while (elapsedTime.seconds() < sec && opModeIsActive() && !isStopRequested()) {
            follower.update();
            CommandScheduler.updateCommands();
        }
    }
    /** Prepares the intake sample */
    public void prepIntake(boolean sub){
        horiz.setState(HorizSubsystem.State.PREP_AUTO_INTAKE);
        updateCommands(0.1);
        pivot.setState(PivotSubsystem.State.PREPARE_INTAKE);
        rotate.setState(RotateSubsystem.State.INTAKE);
        updateCommands(0.25);
        claw.open();
        if(!sub) {
            horiz.setState(HorizSubsystem.State.INTAKING_EXTENDED);
            pivot.setState(PivotSubsystem.State.PREPARE_INTAKE_AUTO);
        }
        else{
            horiz.setState(HorizSubsystem.State.SUB_AUTO_INTAKE);
        }
        updateCommands(0.4);
    }

    /** Rotates for third sample */
    public void rotateIntake(){
        rotate.setState(RotateSubsystem.State.SAMPLE);
        CommandScheduler.updateCommands();
    }
    /** Grabs sample */
    public void finishIntake(){
        pivot.setState(PivotSubsystem.State.INTAKE);
        updateCommands(0.4);
        claw.close();
        updateCommands(0.25); //try to lower
        if(!claw.isBlockInClaw()){
            pivot.setState(PivotSubsystem.State.PREPARE_INTAKE);
            claw.open();
            updateCommands(0.35);
            pivot.setState(PivotSubsystem.State.INTAKE);
            updateCommands(0.35);
            claw.close();
            updateCommands(0.25);
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
        CommandScheduler.updateCommands();
    }

    /** Deposits, and lowers arm */
    public void lowerArm(){
        pivot.setState(PivotSubsystem.State.OUTTAKE_BUCKET);
        rotate.setState(RotateSubsystem.State.DROPOFF_BUCKET);
        updateCommands(0.5);
        claw.open();
        updateCommands(0.25);
        rotate.setState(RotateSubsystem.State.DROPOFF);
        pivot.setState(PivotSubsystem.State.DRIVING);
        claw.close();
        vertical.setState(VerticalSubsystem.State.BOTTOM);
        CommandScheduler.updateCommands();
    }

    public void parkArm(){
        pivot.setState(PivotSubsystem.State.PARK);
        horiz.setState(HorizSubsystem.State.PARK);
        CommandScheduler.updateCommands();
    }

    public boolean autoRotateIntake(){
        //rotation detection
        positions = detectSample.getPositions();
        if(positions.isEmpty()) {
            mecanumController.drive(0,0, 0); //TODO do something... maybe follow path until something found
            return false;
        }

        lastDetection = selectPos(positions);

        if (lastDetection == null) {
            lastDetection = new Pose2D(0, 0, 0);
            clawAlignment.reset();
        }

        tracker.update();

//        horiz.setAutoPos(lastDetection.getY()-360);

        rotate.turn(lastDetection.getHeadingRadians());
        rotate.processState();
        CommandScheduler.updateCommands();

        //rotation movement
        if(!sampleCentered()){
            double x, y;
            x = transPID.update(-lastDetection.getX(), 200*tracker.getRobotVelocity().getX());
            y = transPID.update(lastDetection.getY(), 200*tracker.getRobotVelocity().getY());
            mecanumController.drive(y/2, x/2, 0);
        }
        if (!rotate.aligned) {
            clawAlignment.reset();
            claw.open();
        } else {
            claw.setColor(ClawSubsystem.ColorState.GREEN);
        }

        return clawAlignment.seconds() > 0.2 && sampleCentered() && elapsedTime.seconds() > 25; //&& sampleCentered();
    }

    public Pose2D selectPos(ArrayList<Pose2D> positions){
        double shortest = Double.POSITIVE_INFINITY;
        Pose2D result = null;
        for(Pose2D position : positions) {
            double dist = Vector2D.dist(position.asVector(),lastDetection.asVector());
            if (dist < shortest){
                shortest = dist;
                result = position;
            }
        }

        return result;
    }

    public boolean sampleCentered(){
        double rotation = lastDetection.getHeadingRadians();

        telemetry.addData("X", lastDetection.getX());
        telemetry.addData("Y", lastDetection.getY());
        telemetry.addData("rotation", rotation);
        telemetry.update();

        return abs(lastDetection.getX() - (-89)) < (200) && abs(lastDetection.getY() - (123)) < (150); //107, 51
    }
}

