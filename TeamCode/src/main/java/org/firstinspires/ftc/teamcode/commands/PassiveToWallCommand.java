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
import org.firstinspires.ftc.teamcode.teleops.AOfficialTeleOp;

public class PassiveToWallCommand extends Command {
    RotateSubsystem rotate;
    ClawSubsystem claw;
    PivotSubsystem pivot;
    HorizSubsystem horiz;
    VerticalSubsystem vertical;
    private double startTime;
    private double endTime;

    public PassiveToWallCommand(RotateSubsystem rotate, ClawSubsystem claw, PivotSubsystem pivot, HorizSubsystem horiz, VerticalSubsystem vertical) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(rotate, claw, pivot, horiz);
        this.rotate = rotate;
        this.claw = claw;
        this.pivot = pivot;
        this.horiz = horiz;
        this.vertical = vertical;
        endTime = 1.5;
    }


    // Called once when the command is initially schedule

    public void init() {
        startTime = Clock.now();

        claw.open();
        horiz.setState(HorizSubsystem.State.DRIVING);
        pivot.setState(PivotSubsystem.State.FINISHPASSIVE);
    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        if(Clock.hasElapsed(startTime,0.5)){
            vertical.setState(VerticalSubsystem.State.BOTTOM);
        }
        if(Clock.hasElapsed(startTime, 0.6)){
            claw.close();
            pivot.setState(PivotSubsystem.State.DRIVING);
        }
        if(Clock.hasElapsed(startTime,1)){
            pivot.setState(PivotSubsystem.State.PREPAREPICKUP);
            horiz.setState(HorizSubsystem.State.INTAKE_WALL);
        }
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        rotate.setState(RotateSubsystem.State.PICKUP);
        pivot.setState(PivotSubsystem.State.PICKUP);
        claw.open();
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, endTime);
    }
}
