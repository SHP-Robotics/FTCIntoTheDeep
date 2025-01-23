package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;

@Config
@Autonomous(name = "Testing 3 + 0")
public class TestAuto extends LinearOpMode {
    private MecanumController mecanumController;
    private DeterministicTracker tracker;
    VerticalSubsystem vertical;
    PivotSubsystem pivot;
    RotateSubsystem rotate;
    HorizSubsystem horizontal;
    ClawSubsystem claw;
    PathContainer startToSub,
            sweepBlock1a, sweepBlock1b,
            sweepBlock2a, sweepBlock2b,
            sweepBlock3a, sweepBlock3b,
            grabBlock1, depositBlock1,
            grabBlock2, depositBlock2,
            grabBlock3, depositBlock3,
            grabBlock4, depositBlock4,
            park;

    PathFollower pathFollower;

    private ElapsedTime elapsedTime;
    public static double kp = 0.0;

    public PathFollower generatePathFollower(PathContainer pathContainer, double deceleration, double speed) {
        return new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)
                .setEndpointPID(new PID(kp,0, 0)) //make 0.02
                .setHeadingPID(new PID(0.375, 0, 0)) //0.3
                .setDeceleration(2.0)
                .setSpeed(speed)
//                .setDecelerationFunction(PathFollower.SQUID_DECELERATION)
                //^^^ combats static friction
                // takes the square root of PID. PID controls drive speed
                // call this "SQUID"
                //.setCheckFinishedFunction()
                .setEndTolerance(0.4, Math.toRadians(0.5))
                .setEndVelocityTolerance(4)
                .setTimeAfterDeceleration(deceleration)
                .build();
    }

    @Override
    public void runOpMode() {
        CommandScheduler.resetInstance();

        startToSub = new PathContainer.PathContainerBuilder()
                .setIncrement(0.1)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, 0),
                                        new Vector2D(5, -31)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .build();

        sweepBlock1a = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(5, -31),
                                        new Vector2D(-19, -25) //-35, -25
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0,
                                Math.toRadians(135),
                                Math.toRadians(135),
                                Math.toRadians(135),
                                Math.toRadians(135),
                                Math.toRadians(135),
                                Math.toRadians(135),
                                Math.toRadians(135),
                                Math.toRadians(135)
                        })
                )
                .build();

        sweepBlock1b = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-19, -25),
                                        new Vector2D(-14, -15) //-35, -25
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(135),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40)
                        })
                )
                .build();

        sweepBlock2a = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-14, -15),
                                        new Vector2D(-27.5, -25) //-35, -25
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(40),
                                Math.toRadians(130),
                                Math.toRadians(130),
                                Math.toRadians(130),
                                Math.toRadians(130),
                                Math.toRadians(130),
                                Math.toRadians(130),
                                Math.toRadians(130),
                                Math.toRadians(130),
                                Math.toRadians(130),
                                Math.toRadians(130),
                                Math.toRadians(130),
                                Math.toRadians(130)
                        })
                )
                .build();

        sweepBlock2b = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-27.4, -25),
                                        new Vector2D(-20, -15) //-35, -25
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(130),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40),
                                Math.toRadians(40)
                        })
                )
                .build();

//        sweepBlock3a = new PathContainer.PathContainerBuilder()
//                .setIncrement(0.02)
//                .addCurve(
//                        new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(-20, -22),
//                                        new Vector2D(-40, -25.1) //-35, -25
//                                }
//                        ),
//                        new ParametricHeading(new double[]{
//                                Math.toRadians(40),
//                                Math.toRadians(130),
//                                Math.toRadians(130),
//                                Math.toRadians(130),
//                                Math.toRadians(130),
//                                Math.toRadians(130),
//                                Math.toRadians(130),
//                                Math.toRadians(130),
//                                Math.toRadians(130),
//                                Math.toRadians(130),
//                                Math.toRadians(130),
//                                Math.toRadians(130),
//                                Math.toRadians(130)
//                        })
//                )
//                .build();
//
//        sweepBlock3b = new PathContainer.PathContainerBuilder()
//                .setIncrement(0.02)
//                .addCurve(
//                        new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(-40, -25.1),
//                                        new Vector2D(-44, -13.1) //-35, -25
//                                }
//                        ),
//                        new ParametricHeading(new double[]{
//                                Math.toRadians(130),
//                                Math.toRadians(25),
//                                Math.toRadians(25),
//                                Math.toRadians(25),
//                                Math.toRadians(25),
//                                Math.toRadians(25),
//                                Math.toRadians(25),
//                                Math.toRadians(25),
//                                Math.toRadians(25),
//                                Math.toRadians(25),
//                                Math.toRadians(25),
//                                Math.toRadians(25),
//                                Math.toRadians(25)
//                        })
//                )
//                .build();
        grabBlock1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-20, -15),
                                        new Vector2D(-5, -20)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(40),
                                Math.toRadians(0),
                                Math.toRadians(0),
                                Math.toRadians(0),
                                Math.toRadians(0),
                                Math.toRadians(0),
                                Math.toRadians(0),
                                Math.toRadians(0)
                        })
                )
                .build();

        depositBlock1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-5, -20), //with 2 blocks (-47, 10)
                                        new Vector2D(5, -11),
                                        new Vector2D(5, -11),
                                        new Vector2D(5, -11),
                                        new Vector2D(5, -11)

                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(5, -11), //with 2 blocks (-47, 10)
                                        new Vector2D(6.5, -29.5)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .build();

        grabBlock2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(6.5, -29.5), //with 2 blocks (-47, 10)
                                        new Vector2D(5, -20)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-5, -20),
                                        new Vector2D(-27.8, -20)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-27.8, -20),
                                        new Vector2D(-27.8, -3)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .build();

        depositBlock2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-27.8, -3), //with 2 blocks (-47, 10)
                                        new Vector2D(5, -11),
                                        new Vector2D(5, -11),
                                        new Vector2D(5, -11),
                                        new Vector2D(5, -11)

                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(5, -11), //with 2 blocks (-47, 10)
                                        new Vector2D(4, -29.5)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .build();

        grabBlock3 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(4, -29.5), //with 2 blocks (-47, 10)
                                        new Vector2D(5, -20)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-5, -20),
                                        new Vector2D(-27.8, -20)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-27.8, -20),
                                        new Vector2D(-27.8, -3)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .build();

        depositBlock3 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-27.8, -3), //with 2 blocks (-47, 10)
                                        new Vector2D(5, -11),
                                        new Vector2D(5, -11),
                                        new Vector2D(5, -11),
                                        new Vector2D(5, -11)

                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(5, -11), //with 2 blocks (-47, 10)
                                        new Vector2D(2, -29.5)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .build();

        grabBlock4 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(2, -29.5), //with 2 blocks (-47, 10)
                                        new Vector2D(5, -20)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-5, -20),
                                        new Vector2D(-27.8, -20)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-27.8, -20),
                                        new Vector2D(-27.8, -3)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .build();
        depositBlock4 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-27.8, -3), //with 2 blocks (-47, 10)
                                        new Vector2D(5, -11),
                                        new Vector2D(5, -11),
                                        new Vector2D(5, -11),
                                        new Vector2D(5, -11)

                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(5, -11), //with 2 blocks (-47, 10)
                                        new Vector2D(1, -29.5)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .build();

        park = new PathContainer.PathContainerBuilder()
                .setIncrement(0.03)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(1, -29.5), //with 2 blocks (-47, 10)
                                        new Vector2D(-30, -6)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .build();

        Clock.start();
        CommandScheduler.getInstance().setTelemetry(telemetry);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        vertical = new VerticalSubsystem(hardwareMap);
        pivot = new PivotSubsystem(hardwareMap);
        rotate = new RotateSubsystem(hardwareMap);
        horizontal = new HorizSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);

        claw.close();
        pivot.processState(PivotSubsystem.State.DRIVING);
        rotate.processState(RotateSubsystem.State.NEUTRAL);

        //TODO THIS IS THE START

        waitForStart();

        //Prepare to deposit preload
        prepArm();
        raiseArm();

        //drive to sub
        followPath(startToSub, 0.25, 0.8);

        //Deposit preload
        lowerArm();

        //Push Block 1
        followPath(sweepBlock1a, 0.75, 0.8);
        sweepA();
        sweepB();
        followPath(sweepBlock1b, 0.75, 0.8);
        sweepC();

        //Push Block 2
        followPath(sweepBlock2a, 0.75, 0.8);
        sweepA();
        sweepB();
        followPath(sweepBlock2b, 0.75, 0.8);
        sweepC();

//        //Push Block 3
//        followPath(sweepBlock3a, 0.5, 0.8);
//        sweepA();
//        sweepThree();
//        sweepB();
//        followPath(sweepBlock3b, 0.5, 0.8);
//        sweepC();

//        //Grab Block 1
//        prepIntake();
//        followPath(grabBlock1, 0.25, 0.8);
//        finishIntake();
//        prepArm();
//        raiseArm();
//
//        //Deposit Block 1
//        followPath(depositBlock1, 0.25, 0.8);
//        lowerArm();
//
//        //Grab Block 2
//        prepIntake();
//        followPath(grabBlock2, 0.25, 0.8);
//        finishIntake();
//        prepArm();
//        raiseArm();
//
//        //Deposit Block 2
//        followPath(depositBlock2, 0.25, 0.6);
//        lowerArm();
//
//        //Grab Block 3
//        prepIntake();
//        followPath(grabBlock3, 0.25, 0.8);
//        finishIntake();
//        prepArm();
//        raiseArm();
//
//        //Deposit Block 2
//        followPath(depositBlock3, 0.25, 0.6);
//        lowerArm();
//
//        //Grab Block 4
//        prepIntake();
//        followPath(grabBlock4, 0.25, 0.8);
//        finishIntake();
//        prepArm();
//        raiseArm();
//
//        //Deposit Block 4
//        followPath(depositBlock4, 0.25, 0.6);
//        lowerArm();
//
//        followPath(park, 0.25, 0.6);

    }

    public void loopOpMode() {
        try {
            CommandScheduler.getInstance().run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        tracker.update();
        pathFollower.update();

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        telemetry.addData("x", tracker.getCurrentPosition().getX());
        telemetry.addData("y", tracker.getCurrentPosition().getY());
        telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
        elapsedTime.reset();
        telemetry.update();
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
            tracker.update();
            updateCommands();
            if (isStopRequested()) return;
        }
    }
    public void followPath(PathContainer path, double deceleration, double speed){
        if (isStopRequested()) return;

        pathFollower = generatePathFollower(path, deceleration, speed);

        while (opModeIsActive() && !isStopRequested() && !pathFollower.isCompleted()) {
            loopOpMode();
        }
    }
    /**
     *  Prepares intake for sweep
     */

    public void sweepA(){
        horizontal.setState(HorizSubsystem.State.INTAKINGEXTENDED);
        pivot.setState(PivotSubsystem.State.PREPAREINTAKE);
        rotate.setState(RotateSubsystem.State.SPECIMEN);
        claw.open();
        updateCommands(0.75);
    }
    public void sweepB(){
        pivot.setState(PivotSubsystem.State.INTAKE);
        updateCommands(0.25);

        claw.close();
        if(claw.isBlockInClaw()){
            claw.setColor(ClawSubsystem.ColorState.GREEN);
        }
        updateCommands(0.25);

        pivot.setState(PivotSubsystem.State.PREPAREINTAKE);
        updateCommands();
    }
    public void sweepC(){
        claw.open();
        claw.setColor(ClawSubsystem.ColorState.OFF);
        updateCommands();
    }

    public void sweepThree(){
        horizontal.setState(HorizSubsystem.State.AUTOINTAKE);
        updateCommands(0.25);
    }

    /** Prepares the intake for wall, opens claw */
    public void prepIntake(){
        //prep intake
        pivot.setState(PivotSubsystem.State.PICKUP);
        horizontal.setState(HorizSubsystem.State.AUTOINTAKE);
        updateCommands();
        rotate.setState(RotateSubsystem.State.PICKUP);
        claw.open();
        updateCommands(0.5);
    }
    /** Grabs specimen and returns to driving mode */
    public void finishIntake(){

        claw.close();
        updateCommands(0.5);

        pivot.setState(PivotSubsystem.State.PICKUP2);
        rotate.setState(RotateSubsystem.State.NEUTRAL);
        updateCommands(0.5);
    }

    /** Raises the pivot */
    public void prepArm(){
        horizontal.setState(HorizSubsystem.State.DRIVING);
        pivot.setState(PivotSubsystem.State.OUTTAKE1);
        updateCommands(0.5);

        pivot.setState(PivotSubsystem.State.OUTTAKE2);
        rotate.setState(RotateSubsystem.State.NEUTRAL);
        updateCommands();
    }
    /** Raises the Vertical */
    public void raiseArm(){
        vertical.setDepositState(VerticalSubsystem.State.HIGHBAR);
        vertical.setState(VerticalSubsystem.State.DEPOSITING);
        updateCommands(0.5);
    }

    /** Deposits, and lowers arm */
    public void lowerArm(){

        vertical.setState(VerticalSubsystem.State.DOWN);
        updateCommands(0.5);

        claw.open();
        vertical.setState(VerticalSubsystem.State.BOTTOM);
        pivot.setState(PivotSubsystem.State.DRIVING);
        updateCommands();
    }
}