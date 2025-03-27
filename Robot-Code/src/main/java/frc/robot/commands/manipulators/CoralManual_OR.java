// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulators;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.coralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralManual_OR extends Command {
  private static      coralSubsystem        coralSystem;
  private             boolean               reverse             = false;
  private             double                speed               = 0;
  /** Creates a new CoralManual_OR. */
  public CoralManual_OR(double speed, boolean reverse, coralSubsystem subsystem) {
    coralSystem = subsystem;
    speed = this.speed;
    reverse = this.reverse;
    addRequirements(coralSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!reverse) {
      coralSystem.speedAssign(speed);
    } else {
      coralSystem.speedAssign(-speed);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (speed < -Constants.operatorManip.Trigger_DeadBand || Constants.operatorManip.Trigger_DeadBand > speed) {
      coralSystem.stop();
      return true;
    } else {
      return false;
    }
  }
}
