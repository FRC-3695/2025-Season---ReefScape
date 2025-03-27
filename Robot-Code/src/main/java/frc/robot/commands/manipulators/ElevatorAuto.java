package frc.robot.commands.manipulators;

import frc.robot.subsystems.elevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorAuto extends Command {
    private final elevatorSubsystem elevatorSystem;
    public ElevatorAuto (elevatorSubsystem subsystem) {
        elevatorSystem = subsystem;
        addRequirements(elevatorSystem);
    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        
    }
    @Override
    public boolean isFinished() {
        return true;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
