package frc.robot.commands.manipulators;

import frc.robot.RobotContainer;
import frc.robot.subsystems.elevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class Manual_OR extends Command {
    public Manual_OR () {
    }
    @Override
    public void initialize() {
    }
    @Override
    public void execute() {
        RobotContainer.manipManual();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
