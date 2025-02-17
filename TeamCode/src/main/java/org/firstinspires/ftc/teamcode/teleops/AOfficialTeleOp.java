package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.PINK;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;

import org.firstinspires.ftc.teamcode.commands.BucketToDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToBucketCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToHumanCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPassiveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToSubCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToWallCommand;
import org.firstinspires.ftc.teamcode.commands.SubToDriveCommand;
import org.firstinspires.ftc.teamcode.commands.WallToDriveCommand;
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
                    new DriveToSubCommand(rotate, claw, pivot, horiz)
                );
                cageState = State.EXTENDED;
                claw.setColor(PINK);
            }
           else if (cageState == State.EXTENDED) {
                CommandScheduler.getInstance().scheduleCommand(
                    new SubToDriveCommand(rotate, claw, pivot, horiz)
                    .then(new WaitCommand(0.25))
                    .then(new RunCommand(()->{
                        if (!claw.isBlockInClaw()) {
                            CommandScheduler.getInstance().scheduleCommand(
                                    new DriveToSubCommand(rotate, claw, pivot, horiz)
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
                                    horiz.setState(HorizSubsystem.State.DRIVING);
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
            horiz.setTriggerPos(gamepad1.right_trigger);
        }));

        //abort
        new Trigger(gamepad1.dpad_up, new RunCommand(() -> {
            claw.close();
            rotate.setState(RotateSubsystem.State.NEUTRAL);
            pivot.setState(PivotSubsystem.State.DRIVING);
            horiz.setState(HorizSubsystem.State.DRIVING);
            cageState = State.COMPLETE;
            intakeState = State.COMPLETE;
            claw.setColor(OFF);
        }));

        //intake specimen from wall
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_BUMPER) && intakeState == State.COMPLETE,
                new DriveToWallCommand(rotate, claw, pivot, horiz)
                .then(new RunCommand(()->{
                    intakeState = State.EXTENDED;
                    claw.setColor(PINK);
                }))
        );

        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_BUMPER)&& intakeState == State.EXTENDED,
                new WallToDriveCommand(rotate, claw, pivot, horiz, vertical)
                .then(new RunCommand(()-> intakeState = State.COMPLETE)
        ));


        //deposit specimen
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_TRIGGER) && vertical.getState() == VerticalSubsystem.State.BOTTOM,
                new DriveToPassiveCommand(rotate, claw, pivot, horiz, vertical)
        );
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_TRIGGER) && vertical.getState() != VerticalSubsystem.State.BOTTOM,
                new RunCommand(()-> intakeState = State.EXTENDED)
        );

        //deposit bucket
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.A) && vertical.getState() == VerticalSubsystem.State.BOTTOM,
                new DriveToBucketCommand(rotate, claw, pivot, horiz, vertical)
        );
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.A) && vertical.getState() != VerticalSubsystem.State.DEPOSITING,
                new BucketToDriveCommand(rotate, claw, pivot, horiz, vertical)
        );

        //Claw
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.DPAD_LEFT),
                new RunCommand(() -> rotate.rotateCW()));
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.DPAD_RIGHT),
                new RunCommand(()-> rotate.rotateCCW()));

        //give sample to human player
        new Trigger(gamepad1.square,
                new DriveToHumanCommand(rotate, claw, pivot, horiz));

        //resetIMU
        new Trigger(gamepad1.triangle,
                new RunCommand(()-> drive.resetIMUAngle()));

        //reset Slide Zero Pos
        new Trigger(gamepad2.square,
                new RunCommand(()-> vertical.resetZeroPosition()));

        //Reset encoders
        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.DPAD_UP),
                new RunCommand(() -> vertical.incrementSlide()));
        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.DPAD_DOWN),
                new RunCommand(()-> vertical.emergencyDecrementSlide()));

        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.DPAD_LEFT),
                new RunCommand(()-> vertical.endReset()));

        //EMERGENCY BLOCK IN BOT
        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.A), new RunCommand(()->{ //CROSS
            horiz.setState(HorizSubsystem.State.BLOCK_IN_BOT);
            })
                .then(new WaitCommand(0.5))
                .then(new RunCommand(()-> {
                    horiz.setState(HorizSubsystem.State.DRIVING);
                })
        ));

        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.Y), new RunCommand(()->{ //TRIANGLE
            drive.toggleIMU();
        }));

    }
}
