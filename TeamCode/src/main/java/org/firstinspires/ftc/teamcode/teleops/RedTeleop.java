package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem.State.INTAKE;
import static org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem.State.PICKUP;
import static org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem.State.PREPARE_INTAKE;
import static org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem.State.PREPARE_INTAKE_HIGHER;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.commands.BlockInBotCommand;
import org.firstinspires.ftc.teamcode.commands.BucketToDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToBucketCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToHumanCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToSubCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToWallCommand;
import org.firstinspires.ftc.teamcode.commands.FlashCommand;
import org.firstinspires.ftc.teamcode.commands.SubToDriveCommand;
import org.firstinspires.ftc.teamcode.commands.WallToDriveCommand;
import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;

import java.util.ArrayList;

@TeleOp(name = "*** Red Teleop ***")
public class RedTeleop extends BaseRobot {
    private double driveBias;
    private boolean bucketExtended, autoRotation;
    GamepadInterface gamepadInterface1, gamepadInterface2;
    private int r, g, b;

    Pose2D lastDetection = new Pose2D(0,0,0);
    ElapsedTime clawAlignment;

    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-driveBias*gamepad1.left_stick_y, driveBias*gamepad1.left_stick_x, driveBias*gamepad1.right_stick_x)
                )
        );
        r = 0;
        g = 0;
        b = 0;


        clawAlignment = new ElapsedTime();

        gamepadInterface1 = new GamepadInterface(gamepad1);
        gamepadInterface2 = new GamepadInterface(gamepad2);
//        vision.limelight.start();
        bucketExtended = false;
        vertical.setSlidePower(false);

        autoRotation = true;
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
                new DriveToSubCommand(rotate, claw, pivot, horiz)
                        .then(new RunCommand(()-> clawAlignment.reset())));
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.RIGHT_BUMPER) && pivot.getState() == PREPARE_INTAKE,
                new SubToDriveCommand(rotate, claw, pivot, horiz)
        );

        //extend horizontal slides
        if(gamepad1.right_trigger >= 0.0 && (pivot.getState() == PREPARE_INTAKE || pivot.getState() == INTAKE))
            horiz.setTriggerPos(gamepad1.right_trigger);


        if(autoRotation) {
            //rotation detection
            detectSamples();

            //rotation movement
            new Trigger(pivot.getState() == PREPARE_INTAKE, new RunCommand(() -> {
                if (!rotate.aligned) {
                    clawAlignment.reset();
                    claw.open();
                } else if (clawAlignment.seconds() > 1 && sampleCentered()) {
                    CommandScheduler.getInstance().scheduleCommand(
                            new SubToDriveCommand(rotate, claw, pivot, horiz)
                                    .then(new RunCommand(() -> clawAlignment.reset())));
                }
            }));
//            telemetry.addData("clawAlignment", clawAlignment.seconds());
//            telemetry.addData("CENTERED?", sampleCentered());
        }

        //turn off auto rotation
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.DPAD_DOWN), new RunCommand(()->{
            autoRotation = !autoRotation;
            })
                .then(new FlashCommand(claw, BLUE)));

        if(gamepad1.touchpad){
            if(detectSample.cycleColorsRed()){
                r = 255;
                g = 255;
                b = 0;
            }
            else{
                r = 255;
                g = 0;
                b = 0;
            }
        }
        gamepad1.setLedColor(r,g,b,1000);

        //abort
        if(gamepad1.dpad_up){
            claw.close();
            rotate.setState(RotateSubsystem.State.NEUTRAL);
            pivot.setState(PivotSubsystem.State.DRIVING);
            horiz.setState(HorizSubsystem.State.DRIVING);
            claw.setColor(OFF);
        };

        //intake prepare to intake spec from wall
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_BUMPER) && pivot.getState() != PICKUP,
                new DriveToWallCommand(rotate, claw, pivot, horiz, vertical));
        //attempt to grab spec from wall
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_BUMPER) && pivot.getState() == PICKUP,
                new WallToDriveCommand(rotate, claw, pivot, horiz, vertical));

        //deposit bucket
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.A) && !bucketExtended,
                new DriveToBucketCommand(rotate, claw, pivot, horiz, vertical)
                        .then(new RunCommand(()->bucketExtended = true))
        );
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.A) && bucketExtended,
                new BucketToDriveCommand(rotate, claw, pivot, horiz, vertical)
                        .then(new RunCommand(()->bucketExtended = false))

        );

        //Claw
        if(gamepadInterface1.isKeyDown(GamepadKey.DPAD_LEFT)) rotate.rotateCW();
        if(gamepadInterface1.isKeyDown(GamepadKey.DPAD_RIGHT)) rotate.rotateCCW();

        //give sample to human player
        new Trigger(gamepad1.square, new DriveToHumanCommand(rotate, claw, pivot, horiz));

        //send rails out to park
        new Trigger(gamepad1.circle, new RunCommand(()->{
            horiz.setState(HorizSubsystem.State.INTAKING_EXTENDED);
            pivot.setState(PREPARE_INTAKE_HIGHER);
        }));

        //resetIMU
        if(gamepad1.triangle) drive.resetIMUAngle();

        //Reset encoders
        if(gamepadInterface2.isKeyDown(GamepadKey.DPAD_UP)) vertical.incrementSlide();
        if(gamepadInterface2.isKeyDown(GamepadKey.DPAD_DOWN)) vertical.emergencyDecrementSlide();

        //EMERGENCY BLOCK IN BOT
        if(gamepadInterface2.isKeyDown(GamepadKey.A)) new BlockInBotCommand(horiz); //CROSS?
        if(gamepadInterface2.isKeyDown(GamepadKey.Y)) drive.toggleIMU(); //TRIANGLE

        //disables break beam
        if(gamepadInterface2.isKeyDown(GamepadKey.X)) claw.toggleBreakBeam(); //SQUARE

        telemetry.addData("AUTO ROTATION ON? ", autoRotation);

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

    public void detectSamples(){
        positions = detectSample.getPositions();
        lastDetection = selectPos(positions);
        if (lastDetection == null) {
            lastDetection = new Pose2D(0, 0, 0);
            clawAlignment.reset();
        }

        if(pivot.getState() == PREPARE_INTAKE){
//            telemetry.addData("Detected rotation", lastDetection.getHeadingRadians());
            rotate.turn(lastDetection.getHeadingRadians());
            rotate.processState();
        }
    }

    public boolean sampleCentered(){
        return Math.abs(lastDetection.getY()) < 300 && Math.abs(lastDetection.getX()) < 200;
    }

}