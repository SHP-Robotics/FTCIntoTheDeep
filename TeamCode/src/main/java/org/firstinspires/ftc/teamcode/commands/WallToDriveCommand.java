package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.OFF;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;

public class WallToDriveCommand extends Command {
    RotateSubsystem rotate;
    ClawSubsystem claw;
    PivotSubsystem pivot;
    HorizSubsystem horiz;
    VerticalSubsystem vertical;
    private double startTime, endTime;

    public WallToDriveCommand(RotateSubsystem rotate, ClawSubsystem claw, PivotSubsystem pivot, HorizSubsystem horiz, VerticalSubsystem vertical) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(rotate, claw, pivot, horiz);
        this.rotate = rotate;
        this.claw = claw;
        this.pivot = pivot;
        this.horiz = horiz;
        this.vertical = vertical;
        endTime = 0.25;
    }


    // Called once when the command is initially schedule

    public void init() {
        startTime = Clock.now();
    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        claw.close();
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        if (!claw.isBlockInClaw())
            claw.open();
        else {
            claw.setColor(GREEN);
            claw.setColor(OFF);
            CommandScheduler.getInstance().scheduleCommand(
                    new DriveToPassiveCommand(rotate, claw, pivot, horiz, vertical));
        }
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, endTime);
    }
}
