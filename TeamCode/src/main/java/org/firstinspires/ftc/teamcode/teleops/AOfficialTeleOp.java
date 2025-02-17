package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem.State.INTAKE;
import static org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem.State.PICKUP;
import static org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem.State.PREPARE_INTAKE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;

import org.firstinspires.ftc.teamcode.commands.BlockInBotCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;

@TeleOp(name = "A Official Teleop")
public class AOfficialTeleOp extends BaseRobot {
    private double driveBias;
    GamepadInterface gamepadInterface1, gamepadInterface2;

    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-driveBias*gamepad1.left_stick_y, driveBias*gamepad1.left_stick_x, driveBias*gamepad1.right_stick_x)
                )
        );

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
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.RIGHT_BUMPER) && pivot.getState() != PREPARE_INTAKE,
                new DriveToSubCommand(rotate, claw, pivot, horiz));
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.RIGHT_BUMPER) && pivot.getState() == PREPARE_INTAKE,
                new SubToDriveCommand(rotate, claw, pivot, horiz));

        if(gamepad1.right_trigger >= 0.0 && (pivot.getState() == PREPARE_INTAKE || pivot.getState() == INTAKE))
            horiz.setTriggerPos(gamepad1.right_trigger);

        //abort
        if(gamepad1.dpad_up){
            claw.close();
            rotate.setState(RotateSubsystem.State.NEUTRAL);
            pivot.setState(PivotSubsystem.State.DRIVING);
            horiz.setState(HorizSubsystem.State.DRIVING);
            claw.setColor(OFF);
        };

        //intake specimen from wall
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_BUMPER) && pivot.getState() != PICKUP,
                new DriveToWallCommand(rotate, claw, pivot, horiz, vertical));
        //
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_BUMPER) && pivot.getState() == PICKUP,
                new WallToDriveCommand(rotate, claw, pivot, horiz, vertical)
                    .then(new RunCommand(()-> {
                        if (claw.isBlockInClaw()) {
                            CommandScheduler.getInstance().scheduleCommand(new DriveToPassiveCommand(rotate, claw, pivot, horiz, vertical));
                        }
                    }
        )));

        //deposit bucket
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.A) && vertical.getState() == VerticalSubsystem.State.BOTTOM,
                new DriveToBucketCommand(rotate, claw, pivot, horiz, vertical)
        );
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.A) && vertical.getState() != VerticalSubsystem.State.DEPOSITING,
                new BucketToDriveCommand(rotate, claw, pivot, horiz, vertical)
        );

        //Claw
        if(gamepadInterface1.isKeyDown(GamepadKey.DPAD_LEFT)) rotate.rotateCW();
        if(gamepadInterface1.isKeyDown(GamepadKey.DPAD_RIGHT)) rotate.rotateCCW();

        //give sample to human player
        new Trigger(gamepad1.square, new DriveToHumanCommand(rotate, claw, pivot, horiz));

        //resetIMU
        if(gamepad1.triangle) drive.resetIMUAngle();

        //reset Slide Zero Pos
        if(gamepad2.square) vertical.resetZeroPosition();

        //Reset encoders
        if(gamepadInterface2.isKeyDown(GamepadKey.DPAD_UP)) vertical.incrementSlide();
        if(gamepadInterface2.isKeyDown(GamepadKey.DPAD_DOWN)) vertical.emergencyDecrementSlide();
        if(gamepadInterface2.isKeyDown(GamepadKey.DPAD_LEFT)) vertical.endReset();

        //EMERGENCY BLOCK IN BOT
        if(gamepadInterface2.isKeyDown(GamepadKey.A)) new BlockInBotCommand(horiz); //CROSS?
        if(gamepadInterface2.isKeyDown(GamepadKey.Y)) drive.toggleIMU(); //TRIANGLE
    }
}