package frc.robot.commands.manipulators;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.coralSubsystem;
import frc.robot.tools.utils;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class CoralAutoFeed extends Command {
    private final coralSubsystem coralSystem;
    private static Timer        runTimer                = new Timer();
    public CoralAutoFeed (coralSubsystem subsystem) {
        coralSystem = subsystem;
        addRequirements(coralSystem);
    }
    @Override
    public void initialize() {
        utils.Logging(4, "Coral AutoFeed Called");

    }
    @Override
    public void execute() {                                                                                        // 
        if (!runTimer.isRunning()) {
            utils.Logging(4, "Intake Started");
            runTimer.reset();
            if (!Robot.elevatorSensor_CorLoad.get()) {
                runTimer.start();
                coralSubsystem.intake();
            } else if (Robot.elevatorSensor_CorLoad.get()) {
                runTimer.start();
                coralSubsystem.eject();
            }
        }
        else {
            utils.Logging(4, "Intake Running");
            if (Robot.coralMotor.get() < 0) {
                if (Robot.elevatorSensor_CorLoad.get() || runTimer.get() > 5.0) {
                    coralSubsystem.stop();
                    utils.Logging(4, "Intake Stopped");
                    runTimer.stop();
                }
            } else {
                if (!Robot.elevatorSensor_CorEmpty.get() || runTimer.get() > 1.5) {
                    coralSubsystem.stop();
                    utils.Logging(4, "Intake Stopped");
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
