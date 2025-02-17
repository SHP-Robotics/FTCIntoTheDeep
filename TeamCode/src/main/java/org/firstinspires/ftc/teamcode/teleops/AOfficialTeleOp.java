package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.PINK;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;

import org.firstinspires.ftc.teamcode.commands.BuckettoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetoBucketCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetoHumanCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetoPassiveCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetoSpecimenCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetoSubCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetoWallCommand;
import org.firstinspires.ftc.teamcode.commands.HumantoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.SpecimentoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.SubtoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.WalltoDriveCommand;
import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;

@TeleOp(name = "A Official Teleop")
public class AOfficialTeleOp extends BaseRobot {
    private double driveBias;
    private boolean LTTrigger, crossTrigger;

    GamepadInterface gamepadInterface1, gamepadInterface2;

    public enum State {
        EXTENDED,
        COMPLETE

    }
    private State cageState;
    private State intakeState;

    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-driveBias*gamepad1.left_stick_y, driveBias*gamepad1.left_stick_x, driveBias*gamepad1.right_stick_x)
                )
        );
        LTTrigger = true;
        crossTrigger = true;

        cageState = State.COMPLETE;
        intakeState = State.COMPLETE;

        gamepadInterface1 = new GamepadInterface(gamepad1);
        gamepadInterface2 = new GamepadInterface(gamepad2);

//        vision.limelight.start();

    }
    @Override
    public void start(){
        super.start();
        driveBias = vertical.getDriveBias(gamepad1.right_stick_button);
    }

    @Override
    public void loop(){
        super.loop();
        driveBias = vertical.getDriveBias(gamepad1.right_stick_button);
        gamepadInterface1.update();
        drive.update(gamepad1);

        gamepadInterface2.update();
        drive.update(gamepad2);


        //collect from sub
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.RIGHT_BUMPER), new RunCommand(() -> {
            if (cageState == State.COMPLETE) {
                CommandScheduler.getInstance().scheduleCommand(
                    new DrivetoSubCommand(rotate, claw, pivot, horizontal)
                );
                cageState = State.EXTENDED;
                claw.setColor(PINK);
            }
           else if (cageState == State.EXTENDED) {
                CommandScheduler.getInstance().scheduleCommand(
                    new SubtoDriveCommand(rotate, claw, pivot, horizontal)
                    .then(new WaitCommand(0.25))
                    .then(new RunCommand(()->{
                        if (!claw.isBlockInClaw()) {
                            CommandScheduler.getInstance().scheduleCommand(
                                    new DrivetoSubCommand(rotate, claw, pivot, horizontal)
                            );
                            andrewWompWomp++;
                        }
                        else {
                            claw.setColor(GREEN);
                            CommandScheduler.getInstance().scheduleCommand(
                                new RunCommand(()->{
                                    rotate.setState(RotateSubsystem.State.NEUTRAL);
                                    pivot.setState(PivotSubsystem.State.SUBTODRIVING);
                                    cageState = State.COMPLETE;
                                })
                                .then(new WaitCommand(0.05))
                                .then(new RunCommand(() -> {
                                    horizontal.setState(HorizSubsystem.State.DRIVING);
                                }))
                                .then(new WaitCommand(0.5))
                                .then(new RunCommand(() -> {
                                    pivot.setState(PivotSubsystem.State.DRIVING);
                                    claw.setColor(OFF);
                                }))
                            );
                        }
                    }))
                );
            }

        }));
        new Trigger(gamepad1.right_trigger > 0.0 && cageState == State.EXTENDED, new RunCommand(() -> {
            horizontal.setTriggerPos(gamepad1.right_trigger);
        }));

        //abort
        new Trigger(gamepad1.dpad_up, new RunCommand(() -> {
            claw.close();
            rotate.setState(RotateSubsystem.State.NEUTRAL);
            pivot.setState(PivotSubsystem.State.DRIVING);
            horizontal.setState(HorizSubsystem.State.DRIVING);
            cageState = State.COMPLETE;
            intakeState = State.COMPLETE;
            claw.setColor(OFF);
        }));

        //TODO ADD ABORT FOR SUB ON SHORTSIDE

        //intake specimen from wall
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_BUMPER), new RunCommand(() -> {
            if (intakeState == State.COMPLETE) {
                CommandScheduler.getInstance().scheduleCommand(
                        new DrivetoWallCommand(rotate, claw, pivot, horizontal)
                );
                intakeState = State.EXTENDED;
                claw.setColor(PINK);
            }
            else if (intakeState == State.EXTENDED) {
                CommandScheduler.getInstance().scheduleCommand(
                    new WalltoDriveCommand(rotate, claw, pivot, horizontal)
                    .then(new RunCommand(()->{
                    if (!claw.isBlockInClaw()) {
                        CommandScheduler.getInstance().scheduleCommand(
                                new RunCommand(()->{
                                    claw.open();
                                })
                        );
                        andrewWompWomp++;
                    }
                    else {
                        claw.setColor(GREEN);
                        intakeState = State.COMPLETE;
                        claw.setColor(OFF);
                        LTTrigger = false;
                        CommandScheduler.getInstance().scheduleCommand(
                                new DrivetoPassiveCommand(rotate, claw, pivot, horizontal, vertical));

//                        horizontal.setState(HorizSubsystem.State.DRIVING);
//                        claw.setColor(GREEN);
//                        CommandScheduler.getInstance().scheduleCommand(
//                            new RunCommand(()->{
//                                pivot.setState(PivotSubsystem.State.PICKUP2);
//                                rotate.setState(RotateSubsystem.State.NEUTRAL);
//                                horizontal.setState(HorizSubsystem.State.DRIVING);
//                                intakeState = State.COMPLETE;
//                            })
//                            .then(new WaitCommand(0.25))
//                            .then(new RunCommand(() -> {
//                                horizontal.setState(HorizSubsystem.State.DRIVING);
//                                pivot.setState(PivotSubsystem.State.DRIVING);
//                                claw.setColor(OFF);
//                            }))
//                        );
                    }
                })));
            }
        }));


        //deposit specimen
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_TRIGGER) && LTTrigger,
                new DrivetoPassiveCommand(rotate, claw, pivot, horizontal, vertical)
                .then(new RunCommand(()->{
                    LTTrigger = false;
                }))

//TODO THIS IS THE ACTIVE
//                new DrivetoSpecimenCommand(rotate, claw, pivot, horizontal, vertical)
//                .then(new RunCommand(()->{
//                    LTTrigger = false;
//                }))
        );
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_TRIGGER) && !LTTrigger,
                new RunCommand(()->{
                   claw.open();
                   horizontal.setState(HorizSubsystem.State.DRIVING);
                   pivot.setState(PivotSubsystem.State.FINISHPASSIVE);
                   LTTrigger = true;
                })
                        .then(new WaitCommand(0.5))
                        .then(new RunCommand(()->{
                            vertical.setState(VerticalSubsystem.State.BOTTOM);
                        }))
                        .then(new WaitCommand(0.5))
                        .then(new RunCommand(()->{
                            claw.close();
                            pivot.setState(PivotSubsystem.State.DRIVING);
                            intakeState = State.EXTENDED;
                        }))
                        .then(new DrivetoWallCommand(rotate, claw, pivot, horizontal))




//TODO THIS IS THE ACTIVE
//                new RunCommand(()->{
//                    vertical.setState(VerticalSubsystem.State.BOTTOM); //down to bottom
//                })
//                    .then(new WaitCommand(0.5))
//                    .then(new RunCommand(()->{
//                        claw.open();
//                    }))
//                    .then(new DrivetoWallCommand(rotate, claw, pivot, horizontal)
//                    .then(new RunCommand(()->{
//                        vertical.setState(VerticalSubsystem.State.BOTTOM);
//                        intakeState = State.EXTENDED;
//                        claw.setColor(PINK);
//                        LTTrigger = true;
//                    }))




//                new SpecimentoDriveCommand(rotate, claw, pivot, horizontal, vertical)
//                .then(new RunCommand(()->{
//                    LTTrigger = true;
//                }))
//                .then(new WaitCommand(0.5))
//                .then(new RunCommand(()->{
//                    claw.close();
//                }))
        );

        //deposit bucket
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.A) && crossTrigger,
                new DrivetoBucketCommand(rotate, claw, pivot, horizontal, vertical)
                .then(new RunCommand(()->{
                    crossTrigger = false;
                }))

        );
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.A) && !crossTrigger,
                new BuckettoDriveCommand(rotate, claw, pivot, horizontal, vertical)
                        .then(new RunCommand(()->{
                            crossTrigger = true;
                        }))
                        .then(new WaitCommand(0.5))
                        .then(new RunCommand(()->{
                            pivot.setState(PivotSubsystem.State.DRIVING);
                            vertical.setState(VerticalSubsystem.State.BOTTOM);
                            claw.close();
                        }))
        );

        //Claw
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.DPAD_LEFT), new RunCommand(() -> {
                rotate.rotateCW();
        }));
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.DPAD_RIGHT), new RunCommand(()->{
                rotate.rotateCCW();
        }));

        //give sample to human player
        new Trigger(gamepad1.square,
                new DrivetoHumanCommand(rotate, claw, pivot, horizontal)
                .then(new WaitCommand(0.5))
                .then(new HumantoDriveCommand(rotate, claw, pivot, horizontal)));


        //resetIMU
        new Trigger(gamepad1.triangle, new RunCommand(()->{
            drive.resetIMUAngle();
        }));


        //reset Slide Zero Pos
        new Trigger(gamepad2.square, new RunCommand(()->{
            vertical.resetZeroPosition();
        }));

        //Reset encoders
        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.DPAD_UP), new RunCommand(() -> {
            vertical.incrementSlide();
        }));
        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.DPAD_DOWN), new RunCommand(()->{
            vertical.emergencyDecrementSlide();
        }));

        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.DPAD_LEFT), new RunCommand(()->{
            vertical.endReset();
        }));

        //EMERGENCY BLOCK IN BOT
        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.A), new RunCommand(()->{ //CROSS
            horizontal.setState(HorizSubsystem.State.BLOCKINBOT);
            })
                .then(new WaitCommand(0.5))
                .then(new RunCommand(()-> {
                    horizontal.setState(HorizSubsystem.State.DRIVING);
                })
        ));

        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.Y), new RunCommand(()->{ //TRIANGLE
            drive.toggleIMU();
        }));

    }
}
