// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulators;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.algaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeManual_OR extends Command {
  private final     algaeSubsystem        algaeSystem;
  private           double                speed               = 0;
  private           boolean               reverse             = false;
  /** Creates a new AlgaeManual_OR. */
  public AlgaeManual_OR(double speed, boolean reverse, algaeSubsystem subsystem) {
    algaeSystem = subsystem;
    speed = this.speed;
    reverse = this.reverse;
    addRequirements(algaeSystem);
  }
  @Override
  public void initialize() {}
  @Override
  public void execute() {
    if (!reverse) {
      algaeSystem.speedAsign(speed);
    } else {
      algaeSystem.speedAsign(-speed);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (speed < -Constants.operatorManip.Trigger_DeadBand || Constants.operatorManip.Trigger_DeadBand > speed) {
      algaeSystem.stop();
      return true;
    } else {
      return false;
    }
  }
}
