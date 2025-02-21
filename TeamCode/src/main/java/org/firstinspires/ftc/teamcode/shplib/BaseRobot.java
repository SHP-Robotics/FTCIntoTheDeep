package org.firstinspires.ftc.teamcode.shplib;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.shprobotics.pestocore.geometries.Pose2D;

import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DetectSample;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;

import java.util.ArrayList;

public class BaseRobot extends OpMode {
    // Declare subsystems and devices
    public DriveSubsystem drive;
    public HorizSubsystem horiz;
    public VerticalSubsystem vertical;
    public PivotSubsystem pivot;
    public RotateSubsystem rotate;
    public ClawSubsystem claw;

    public double previousTime = 0;
    public double andrewWompWomp = 0;

    public ArrayList<Pose2D> positions;
    public DetectSample detectSample;

    // Called when you press the init button
    @Override
    public void init() {
        CommandScheduler.resetInstance();
        // Configures universal clock and scheduler - DO NOT DELETE!
        configure();

        // Initialize your subsystems and devices
        drive = new DriveSubsystem(hardwareMap);
        vertical = new VerticalSubsystem(hardwareMap);
        rotate = new RotateSubsystem(hardwareMap);
        pivot = new PivotSubsystem(hardwareMap);
        pivot.setState(PivotSubsystem.State.DRIVING);
        pivot.periodic(telemetry);
        claw = new ClawSubsystem(hardwareMap);
        horiz = new HorizSubsystem(hardwareMap);

        detectSample = new DetectSample(hardwareMap, telemetry);
    }

    // Called when you press the start button
    @Override
    public void start() {
    }

    // Called repeatedly while an OpMode is running
    @Override
    public void loop() {
        telemetry.addData("Loop Time (ms): ", Clock.elapsed(previousTime) * 1000);
        previousTime = Clock.now();
        telemetry.addData("ANDREW WOMP WOMP", andrewWompWomp);

        // Handles all subsystem and command execution - DO NOT DELETE!
        CommandScheduler.updateCommands();
    }

    // Called when you press the stop button
    @Override
    public void stop() {
        // Flushes any cached subsystems and commands - DO NOT DELETE!
        CommandScheduler.resetInstance();
    }

    public void configure() {
        // Starts universal clock - DO NOT DELETE!
        Clock.start();
        // Assigns telemetry object for Subsystem.periodic - DO NOT DELETE!
        CommandScheduler.getInstance().setTelemetry(telemetry);
        // Turn on bulk reads to help optimize loop times
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}
