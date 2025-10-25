package frc.robot.commands.drive;

import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.Command;

public class ZeroHeading extends Command {
    public ZeroHeading() {
        super(Drive.getInstance(), "Drive", "ZeroHeading");
    }

     // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Drive.getInstance().zeroHeading();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}

