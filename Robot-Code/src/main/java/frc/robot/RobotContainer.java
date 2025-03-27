// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.manipulators.AlgaeAutoFeed;
import frc.robot.commands.manipulators.AlgaeManual_OR;
import frc.robot.commands.manipulators.CoralAutoFeed;
import frc.robot.commands.manipulators.CoralManual_OR;
import frc.robot.commands.manipulators.Manual_OR;
import frc.robot.subsystems.algaeSubsystem;
import frc.robot.subsystems.coralSubsystem;
import frc.robot.subsystems.elevatorSubsystem;
import frc.robot.subsystems.swervedrive.swerveSubsystem;
import frc.robot.tools.utils;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  public static final swerveSubsystem     drivebase    = new swerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/"+utils.RoboRIOid()));                                                  // The robot's swerve drive subsystems and commands are defined here
  public static final coralSubsystem      coral        = new coralSubsystem();
  public static final algaeSubsystem      algae        = new algaeSubsystem();
  public static final elevatorSubsystem   elevator     = new elevatorSubsystem();
  private static      Boolean             manipManual  = true;
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(                                                                        // Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
    drivebase.getSwerveDrive(),
    () -> Robot.operatorDriver.getLeftY() * -1,
    () -> Robot.operatorDriver.getLeftX() * -1)
    .withControllerRotationAxis(Robot.operatorDriver::getRightX)
    .deadband(Constants.operatorManip.RIGHT_X_DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(                                           // Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
    Robot.operatorDriver::getRightX, 
    Robot.operatorDriver::getRightY)
    .headingWhile(true);
  // ----------------------------------------------------------------------------------//    Robot Cntr    //------------------------------------------------------------------------------------
  public RobotContainer() {                                                                                                             // The container for the robot. Contains subsystems, OI devices, and commands.
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I am alive, I know things!"));
  }
  // -----------------------------------------------------------------------------------//    Bindings    //-------------------------------------------------------------------------------------
  private void configureBindings() {                                                                                                    //  Use this method to define your trigger->command mappings.
//    Command driveFieldOrientedDirectAngle           = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity       = drivebase.driveFieldOriented(driveAngularVelocity);
//    Command driveSetpointGen                        = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    if (DriverStation.isTest()) {                                                                                                       // During Driver Station Test mode set controller binding to
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

      Robot.operatorDriver.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      Robot.operatorDriver.y().whileTrue(drivebase.driveToDistance(1.0, 0.2));
      Robot.operatorDriver.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      Robot.operatorDriver.back().whileTrue(drivebase.modulesCenter());
      Robot.operatorDriver.leftBumper().onTrue(Commands.none());
      Robot.operatorDriver.rightBumper().onTrue(Commands.none());
    } else {                                                                                                                            // During Driver Station Teleop mode set controller binding to
      Robot.operatorDriver.a().onTrue((Commands.none()));
      Robot.operatorDriver.b().onTrue((Commands.none()));
      Robot.operatorDriver.x().onTrue((Commands.none()));
      Robot.operatorDriver.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      Robot.operatorDriver.back().onTrue((Commands.none()));
      Robot.operatorDriver.leftBumper().onTrue((Commands.none()));
      Robot.operatorDriver.rightBumper().onTrue((Commands.none()));
      
      elevator.setDefaultCommand(elevator.core());
      Robot.operatorManip.x().onTrue(new CoralAutoFeed(coral));
      Robot.operatorManip.y().onTrue(new AlgaeAutoFeed(algae));
      
      Robot.operatorManip.start()
        .onTrue(new Manual_OR())
        .debounce(.125)
        .onTrue(driveFieldOrientedAngularVelocity);

      manipManual();
    }
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  // -------------------------------------------------------------------------------//    Auto Pass Through    //--------------------------------------------------------------------------------
  public void setMotorBrake(boolean brake) {
    drivebase.modulesBraking(brake);
  }
  // -----------------------------------------------------------------------------------//    Functions    //------------------------------------------------------------------------------------
    public void dashboardUpdate() {
    SmartDashboard.putData("subsystems/swerve/scheduled",       RobotContainer.drivebase.getCurrentCommand());
    SmartDashboard.putData("subsystems/swerve/default",         RobotContainer.drivebase.getDefaultCommand());
    SmartDashboard.putData("subsystems/coral/scheduled",        RobotContainer.coral.getCurrentCommand());
    SmartDashboard.putData("subsystems/coral/default",          RobotContainer.coral.getDefaultCommand());
    SmartDashboard.putData("subsystems/elevator/scheduled",     RobotContainer.elevator.getCurrentCommand());
    SmartDashboard.putData("subsystems/elevator/default",       RobotContainer.elevator.getDefaultCommand());
    SmartDashboard.putData("subsystems/algae/scheduled",        RobotContainer.algae.getCurrentCommand());
    SmartDashboard.putData("subsystems/algae/default",          RobotContainer.algae.getDefaultCommand());
    SmartDashboard.updateValues();
  }

  public static void manipManual() {
    utils.Logging(4, "Swapped Operator to Manual");
    manipManual ^= true;
    if (!manipManual) {
      elevator.manual_OR();
      algae.core();
      ;coral.core();
    } else {
      elevator.manual_OR();
      algae.manual_OR();
      coral.manual_OR();
    }
  }
}