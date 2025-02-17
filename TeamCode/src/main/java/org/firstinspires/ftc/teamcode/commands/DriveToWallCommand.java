package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.PINK;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;

public class DriveToWallCommand extends Command {
    RotateSubsystem rotate;
    ClawSubsystem claw;
    PivotSubsystem pivot;
    HorizSubsystem horiz;
    VerticalSubsystem vertical;
    double trigger, startTime, endTime;

    public DriveToWallCommand(RotateSubsystem rotate, ClawSubsystem claw, PivotSubsystem pivot, HorizSubsystem horiz, VerticalSubsystem vertical) {
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
        super.init();
        startTime = Clock.now();
    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        if(vertical.getState() == VerticalSubsystem.State.BOTTOM){
            endTime = 0.5;
            pivot.setState(PivotSubsystem.State.PREPARE_PICKUP);
            horiz.setState(HorizSubsystem.State.INTAKE_WALL);
            return;
        }

        if(Clock.hasElapsed(startTime,1)){
            pivot.setState(PivotSubsystem.State.PREPARE_PICKUP);
            horiz.setState(HorizSubsystem.State.INTAKE_WALL);
        }
        else if(Clock.hasElapsed(startTime, 0.6)){
            claw.close();
            pivot.setState(PivotSubsystem.State.DRIVING);
        }
        else if(Clock.hasElapsed(startTime,0.5)){
            vertical.setState(VerticalSubsystem.State.BOTTOM);
        }
        else if (Clock.hasElapsed(startTime, 0)){
            horiz.setState(HorizSubsystem.State.DRIVING);
            pivot.setState(PivotSubsystem.State.FINISH_PASSIVE);
        }
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        rotate.setState(RotateSubsystem.State.PICKUP);
        pivot.setState(PivotSubsystem.State.PICKUP);
        claw.open();
        claw.setColor(PINK);
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, endTime);
    }
}
