package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem.State.WALL_PICKUP_AUTO;

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
@Autonomous(name = "***TEST 4 + 0 TEST***")
public class TestAuto extends LinearOpMode {
    private MecanumController mecanumController;
    private DeterministicTracker tracker;
    VerticalSubsystem vertical;
    PivotSubsystem pivot;
    RotateSubsystem rotate;
    HorizSubsystem horizontal;
    ClawSubsystem claw;
    PathContainer startToSub,
            pushBlock1,
            pushBlock2a, pushBlock2b, pushBlock2c,
            pushBlock3a, pushBlock3b, pushBlock3c,
            grabBlock1a, grabBlock1b, depositBlock1,
            grabBlock2a, grabBlock2b, depositBlock2,
            grabBlock3a, grabBlock3b, depositBlock3,
            park;

    PathFollower pathFollower;

    private ElapsedTime elapsedTime;
    public static double kp = 0.0;

    public PathFollower generatePathFollower(PathContainer pathContainer, double deceleration, double speed) {
        return new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)
                .setEndpointPID(new PID(0.0044, 0, 0.0005)) //0.03/0.025 , 0.005
                .setHeadingPID(new PID(1, 0, 0.0015))
                .setDeceleration(2.0)
                .setSpeed(speed)
//                .setDecelerationFunction(PathFollower.SQUID_DECELERATION)
                //^^^ combats static friction
                // takes the square root of PID. PID controls drive speed
                // call this "SQUID"
                //.setCheckFinishedFunction()
                .setEndTolerance(0.5, Math.toRadians(0.5))
                .setEndVelocityTolerance(4)
                .setTimeAfterDeceleration(deceleration)
                .build();
    }

    @Override
    public void runOpMode() {
        CommandScheduler.resetInstance();

        startToSub = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, 0),
                                        new Vector2D(6, -31),
                                        new Vector2D(6, -31),
                                        new Vector2D(6, -31),
                                        new Vector2D(6, -31)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0 //90
                        })
                )
                .build();

        pushBlock1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)

                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(6, -31),
                                        new Vector2D(-35, -12),
                                        new Vector2D(-35, -12),
                                        new Vector2D(-35, -12),
                                        new Vector2D(-35, -12)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, Math.toRadians(180), Math.toRadians(180), Math.toRadians(180), Math.toRadians(180), Math.toRadians(180), Math.toRadians(180), Math.toRadians(180), Math.toRadians(180) //90
                        })
                )


//                .addCurve(
//                        new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(6, -33),
//                                        new Vector2D(-24, -28),
//                                        new Vector2D(-24, -28),
//                                        new Vector2D(-24, -28),
//                                        new Vector2D(-24, -28)
//                                }
//                        ),
//                        new ParametricHeading(new double[]{
//                                0, 0 //90
//                        })
//                )
//                .addCurve(
//                        new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(-24, -28),
//                                        new Vector2D(-24, -50),
//                                        new Vector2D(-24, -50),
//                                        new Vector2D(-24, -50)
//                                }
//                        ),
//                        new ParametricHeading(new double[]{
//                                0, 0 //90
//                        }))
//                .addCurve(
//                        new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(-24, -50),
//                                        new Vector2D(-35, -50),
//                                        new Vector2D(-35, -50),
//                                        new Vector2D(-35, -50)
//                                }
//                        ),
//                        new ParametricHeading(new double[]{
//                                0, 0 //90
//                        })
//
//                )
//                .addCurve(new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(-35, -50),
//                                        new Vector2D(-35, -10),
//                                        new Vector2D(-35, -10),
//                                        new Vector2D(-35, -10)
//                                }
//                        ),
//                        new ParametricHeading(new double[]{
//                                0, 0 //90
//                        })
//                )

                .build();

        pushBlock2a = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-35, -12),
                                        new Vector2D(-45, -12),
                                        new Vector2D(-45, -12),
                                        new Vector2D(-45, -12)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(180), Math.toRadians(180), Math.toRadians(180) //90
                        })
                )


//                .addCurve(new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(-35, -10),
//                                        new Vector2D(-35, -45),
//                                        new Vector2D(-35, -45),
//                                        new Vector2D(-35, -45)
//                                }
//                        ),
//                        new ParametricHeading(new double[]{
//                                0, 0 //90
//                        })
//                )
//                .addCurve(new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(-35, -45),
//                                        new Vector2D(-52, -45),
//                                        new Vector2D(-52, -45),
//                                        new Vector2D(-52, -45)
//                                }
//                        ),
//                        new ParametricHeading(new double[]{
//                                0, 0 //90
//                        })
//                )
                .build();

//        pushBlock2b = new PathContainer.PathContainerBuilder()
//                .setIncrement(0.02)
//                .addCurve(new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(-35, -45),
//                                        new Vector2D(-48, -45),
//                                        new Vector2D(-48, -45),
//                                        new Vector2D(-48, -45)
//                                }
//                        ),
//                        new ParametricHeading(new double[]{
//                                0, 0 //90
//                        })
//                )
//                .build();

        pushBlock2c = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-52, -45),
                                        new Vector2D(-52, -10),
                                        new Vector2D(-52, -10),
                                        new Vector2D(-52, -10)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .build();

        pushBlock3a = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-45, -12),
                                        new Vector2D(-45, -12),
                                        new Vector2D(-45, -12),
                                        new Vector2D(-45, -12)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(180), Math.toRadians(220), Math.toRadians(220), Math.toRadians(220), Math.toRadians(220), Math.toRadians(220), Math.toRadians(220), Math.toRadians(220), Math.toRadians(220), Math.toRadians(220) //90
                        })
                )

//                .addCurve(new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(-52, -10),
//                                        new Vector2D(-50, -50),
//                                        new Vector2D(-50, -50),
//                                        new Vector2D(-50, -50)
//                                }
//                        ),
//                        new ParametricHeading(new double[]{
//                                0, 0 //90
//                        })
//                )
//                .addCurve(new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(-50, -50),
//                                        new Vector2D(-57, -50),
//                                        new Vector2D(-57, -50),
//                                        new Vector2D(-57, -50)
//                                }
//                        ),
//                        new ParametricHeading(new double[]{
//                                0, 0 //90
//                        })
//                )
                .build();
        pushBlock3b = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)

                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-35, -10),
                                        new Vector2D(-35, -10),
                                        new Vector2D(-35, -10),
                                        new Vector2D(-35, -10)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(220), 0, 0, 0, 0, 0, 0 //90
                        })
                )


//                .addCurve(new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(-48, -50),
//                                        new Vector2D(-57, -50),
//                                        new Vector2D(-57, -50),
//                                        new Vector2D(-57, -50)
//                                }
//                        ),
//                        new ParametricHeading(new double[]{
//                                0, 0 //90
//                        })
//                )
                .build();

        pushBlock3c = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-57, -50),
                                        new Vector2D(-57, -12),
                                        new Vector2D(-57, -12),
                                        new Vector2D(-57, -12)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .build();

        grabBlock1a = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-57, -12),
                                        new Vector2D(-40, -12),
                                        new Vector2D(-40, -12),
                                        new Vector2D(-40, -12)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )

                .build();
        grabBlock1b = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-40, -12),
                                        new Vector2D(-40, -7.5),
                                        new Vector2D(-40, -7.5),
                                        new Vector2D(-40, -7.5)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .build();

        depositBlock1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-40, -7.5), //with 2 blocks (-47, 10)
                                        new Vector2D(8, -20),
                                        new Vector2D(8, -20),
                                        new Vector2D(8, -20),
                                        new Vector2D(8, -20)

                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(8, -20), //with 2 blocks (-47, 10)
                                        new Vector2D(8, -30.5),
                                        new Vector2D(8, -30.5),
                                        new Vector2D(8, -30.5),
                                        new Vector2D(8, -30.5)

                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .build();

        grabBlock2a = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(8, -30.5), //with 2 blocks (-47, 10)
                                        new Vector2D(8, -27)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(8, -27),
                                        new Vector2D(-27.8, -15)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .build();

        grabBlock2b = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-27.8, -15),
                                        new Vector2D(-27.8, -7),
                                        new Vector2D(-27.8, -7),
                                        new Vector2D(-27.8, -7)

                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .build();

        depositBlock2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-27.8, -7), //with 2 blocks (-47, 10)
                                        new Vector2D(5, -20),
                                        new Vector2D(5, -20),
                                        new Vector2D(5, -20),
                                        new Vector2D(5, -20)

                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(5, -20), //with 2 blocks (-47, 10)
                                        new Vector2D(5, -31),
                                        new Vector2D(5, -31),
                                        new Vector2D(5, -31),
                                        new Vector2D(5, -31)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .build();

        grabBlock3a = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(5, -31), //with 2 blocks (-47, 10)
                                        new Vector2D(5, -27)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(5, -27),
                                        new Vector2D(-27.8, -15)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .build();

        grabBlock3b = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-27.8, -15),
                                        new Vector2D(-27.8, -7.5),
                                        new Vector2D(-27.8, -7.5),
                                        new Vector2D(-27.8, -7.5),

                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .build();

        depositBlock3 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-27.8, -7.5), //with 2 blocks (-47, 10)
                                        new Vector2D(2, -11),
                                        new Vector2D(2, -11),
                                        new Vector2D(2, -11),
                                        new Vector2D(2, -11)

                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(2, -11), //with 2 blocks (-47, 10)
                                        new Vector2D(2, -30.5),
                                        new Vector2D(2, -30.5),
                                        new Vector2D(2, -30.5),
                                        new Vector2D(2, -30.5)

                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
                        })
                )
                .build();


        park = new PathContainer.PathContainerBuilder()
                .setIncrement(0.03)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(2, -30.5), //with 2 blocks (-47, 10)
                                        new Vector2D(-30, -6)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0, 0, 0, 0, 0, 0, 0, 0
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
        pivot.processState();
        rotate.processState();

        //TODO THIS IS THE START

        waitForStart();

        //Prepare to deposit preload
        prepArm();
        raiseArm();

        //drive to sub
        followPath(startToSub, 0.15, 0.8);

        //Deposit preload
        lowerArm();

        //Push Block 1
        followPath(pushBlock1, 0.25, 0.8);
//        prepNewIntake();
//        startNewIntake();
//        finishNewIntake();


        //Push Block 2
//        prepNewIntake();
//        followPath(pushBlock2a, 0.05, 0.8);
//        startNewIntake();
//        finishNewIntake();
////        followPath(pushBlock2c, 0.05, 0.8);
//        //Push Block 3
//        prepNewIntake();
//        rotateNewIntake();
//        followPath(pushBlock3a, 0.05, 0.8);
//        startNewIntake();
//        finishNewIntake();
//        followPath(pushBlock3b, 0.05, 0.8);


        //prepIntake();
//        followPath(pushBlock3c, 0.05, 0.8);

//
        //Grab Block 1
        //prepIntake();
//        followPath(grabBlock1a, 0.15, 0.8);
//        followPath(grabBlock1b, 0.15, 0.8);
       // finishIntake();
//        prepArm();
//        raiseArm();



//        //Deposit Block 1
//        followPath(depositBlock1, 0.25, 0.8);
//        lowerArm();
//
//        //Grab Block 2
//        prepIntake();
//        followPath(grabBlock2a, 0.15, 0.8);
//        followPath(grabBlock2b, 0.15, 0.8);
//        finishIntake();
//        prepArm();
//        raiseArm();
//
//        //Deposit Block 2
//        followPath(depositBlock2, 0.25, 0.8);
//        lowerArm();
//
//        //Grab Block 3
//        prepIntake();
//        followPath(grabBlock3a, 0.15, 0.8);
//        followPath(grabBlock3b, 0.15, 0.8);
//        finishIntake();
//        prepArm();
//        raiseArm();
//
//        //Deposit Block 2
//        followPath(depositBlock3, 0.25, 0.8);
//        lowerArm();
//
//        followPath(park, 0.25, 0.8);


//        //Sweep Block 1
//        followPath(sweepBlock1a, 1, 0.8);
//        sweepA();
//        sweepB();
//        followPath(sweepBlock1b, 0, 0.8);
//        sweepC();
//
//        //Push Block 2
//        followPath(sweepBlock2a, 0.75, 0.8);
//        sweepA();
//        sweepB();
//        followPath(sweepBlock2b, 0, 0.8);
//        sweepC();

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
//    /**
//     *  Prepares intake for sweep
//     */
//
//    public void sweepA(){
//        horizontal.setState(HorizSubsystem.State.INTAKINGEXTENDED);
//        pivot.setState(PivotSubsystem.State.PREPAREINTAKE);
//        rotate.setState(RotateSubsystem.State.SPECIMEN);
//        claw.open();
//        updateCommands(0.75);
//    }
//    public void sweepB(){
//        pivot.setState(PivotSubsystem.State.INTAKE);
//        updateCommands(0.25);
//
//        claw.close();
//        updateCommands(0.25);
//        if(claw.isBlockInClaw()){
//            claw.setColor(ClawSubsystem.ColorState.GREEN);
//        }
//
//        pivot.setState(PivotSubsystem.State.PREPAREINTAKE);
//        updateCommands();
//    }
//    public void sweepC(){
//        claw.open();
//        claw.setColor(ClawSubsystem.ColorState.OFF);
//        updateCommands();
//    }
//
//    public void sweepThree(){
//        horizontal.setState(HorizSubsystem.State.AUTOINTAKE);
//        updateCommands(0.25);
//    }

    /** Prepares the intake for wall, opens claw */
    public void prepIntake(){
        //prep intake
        pivot.setState(PivotSubsystem.State.AUTO_INTAKE);
        horizontal.setState(HorizSubsystem.State.DRIVING);
        updateCommands(0.25);
        rotate.setState(RotateSubsystem.State.PICKUP);
        claw.open();
        updateCommands();
    }
    /** Grabs specimen and returns to driving mode */
    public void finishIntake(){
        updateCommands(0.55);
        horizontal.setState(WALL_PICKUP_AUTO);
        updateCommands(0.05);

        claw.close();
        updateCommands(0.15);

        pivot.setState(PivotSubsystem.State.DRIVING);
        rotate.setState(RotateSubsystem.State.NEUTRAL);
        updateCommands(0.2);
    }

    /** Raises the pivot */
    public void prepArm(){
        horizontal.setState(HorizSubsystem.State.DRIVING);
        pivot.setState(PivotSubsystem.State.AUTO_SPEC);
        updateCommands(0.1);

        rotate.setState(RotateSubsystem.State.NEUTRAL);
        updateCommands();
    }
    /** Raises the Vertical */
    public void raiseArm(){
        vertical.setDepositState(VerticalSubsystem.State.AUTO_HIGH_BAR);
        vertical.setState(VerticalSubsystem.State.DEPOSITING);
        updateCommands(0.25);
    }

    /** Deposits, and lowers arm */
    public void lowerArm(){
        vertical.setState(VerticalSubsystem.State.BOTTOM);
        updateCommands(0.75);

        claw.open();
        updateCommands(0.2);

        claw.close();
        vertical.setState(VerticalSubsystem.State.BOTTOM);
        pivot.setState(PivotSubsystem.State.DRIVING);
        updateCommands();
    }


    /** Prepares the intake sample */
    public void prepNewIntake(){
        //prep intake
        horizontal.setState(HorizSubsystem.State.PREP_AUTO_INTAKE);
        pivot.setState(PivotSubsystem.State.PREPARE_INTAKE);
        rotate.setState(RotateSubsystem.State.INTAKE);
        updateCommands(0.25);
        claw.open();
        horizontal.setState(HorizSubsystem.State.INTAKING_EXTENDED);
        updateCommands(1);
    }

    /** Prepares the intake sample */
    public void rotateNewIntake(){
        //prep intake
        rotate.setState(RotateSubsystem.State.SAMPLE);
        updateCommands();
    }
    /** Grabs sample */
    public void startNewIntake(){
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
    public void finishNewIntake(){
        rotate.setState(RotateSubsystem.State.NEUTRAL);
        pivot.setState(PivotSubsystem.State.DRIVING);
        horizontal.setState(HorizSubsystem.State.DRIVING);
        updateCommands(0.25); //removed wait
        claw.setColor(ClawSubsystem.ColorState.OFF);
        updateCommands();
    }
}