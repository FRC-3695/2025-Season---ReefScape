package frc.robot.commands.manipulators;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.algaeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeAutoFeed extends Command {
    private final algaeSubsystem    algaeSystem;
    private final Timer             runTimer        = new Timer();
    public AlgaeAutoFeed (algaeSubsystem subsystem) {
        algaeSystem = subsystem;
        addRequirements(algaeSystem);
    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        if (!runTimer.isRunning()) {
            runTimer.reset();
            if (!Robot.algaeSensor_Capture.get()) {
                runTimer.start();
                algaeSubsystem.intake();
            } else if (Robot.algaeSensor_Capture.get()) {
                runTimer.start();
                algaeSubsystem.eject();
            }
        } else {
            if (Robot.algaeMotor_Intake.get() < 0) {
                if (Robot.algaeSensor_Capture.get() || runTimer.get() >= Constants.config.algae.autoFeed_TO) {
                    algaeSubsystem.stop();
                    runTimer.stop();
                }
            } else {
                if (runTimer.get() >= Constants.config.algae.autoEject_TO) {
                    algaeSubsystem.stop();
                    runTimer.stop();

                }
            }
        }
    }
    @Override
    public boolean isFinished() {
        if (runTimer.isRunning()) {
            return false;
        } else {
            return true;
        }
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
