// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private Timer disabledTimer;
  // -----------------------------------------------------------------------------------    Controller(s)    ------------------------------------------------------------------------------------
  final static CommandXboxController operatorDriver = new CommandXboxController(Constants.operatorDriver.port);                         // Driver's xBox controller
  final static CommandXboxController operatorManip  = new CommandXboxController(Constants.operatorManip.port);                          // Operator's xBox controller
  // --------------------------------------------------------------------------------------    Motor(s)    --------------------------------------------------------------------------------------
  
  // Front Left Motors :3
  SparkFlex FL_Stearing = new SparkFlex(Constants.CANnet.swerve.FL_Stearing, MotorType.kBrushless);
  SparkFlex FL_Drive = new SparkFlex(Constants.CANnet.swerve.FL_Drive, MotorType.kBrushless);
  SparkFlex FL_Absolute = new SparkFlex(Constants.CANnet.swerve.FL_Absolute, MotorType.kBrushless);
  // Front Right Motors :3
  SparkFlex FR_Absolute = new SparkFlex(Constants.CANnet.swerve.FR_Absolute, MotorType.kBrushless);
  SparkFlex FR_Drive = new SparkFlex(Constants.CANnet.swerve.FR_Drive, MotorType.kBrushless);
  SparkFlex FR_Stearing = new SparkFlex(Constants.CANnet.swerve.FR_Drive, MotorType.kBrushless);
  // Back Right Motors :3
  SparkFlex BR_Absolute = new SparkFlex(Constants.CANnet.swerve.BR_Absolute, MotorType.kBrushless);
  SparkFlex BR_Drive = new SparkFlex(Constants.CANnet.swerve.BR_Drive, MotorType.kBrushless);
  SparkFlex BR_Stearing = new SparkFlex(Constants.CANnet.swerve.BR_Drive, MotorType.kBrushless);
  // Back Left Motors :3
  SparkFlex BL_Absolute = new SparkFlex(Constants.CANnet.swerve.BL_Absolute, MotorType.kBrushless);
  SparkFlex BL_Drive = new SparkFlex(Constants.CANnet.swerve.BL_Drive, MotorType.kBrushless);
  SparkFlex BL_Stearing = new SparkFlex(Constants.CANnet.swerve.BL_Stearing, MotorType.kBrushless);





 
  // -------------------------------------------------------------------------------------    Sensor(s)    --------------------------------------------------------------------------------------

  // ----------------------------------------------------------------------------------    Other Device(s)    -----------------------------------------------------------------------------------
  public static PowerDistribution powerHub = new PowerDistribution(Constants.CANnet.core_PowerHub, ModuleType.kRev);                    // {@param - powerHub} Power Distribution Hub

  // ----------------------------------------------------------------------------------//    Robot Init    //------------------------------------------------------------------------------------
  @Override
  public void robotInit() {                                                                                                             // Called on robot startup, best use is for anything needed to prep robot and ensure std. start state
    m_robotContainer = new RobotContainer();
    disabledTimer = new Timer();                                                                                                        // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop immediately when disabled, but then also let it be pushed
    if (isSimulation()) {                                                                                                               // Checks to see if code is running in simulation
      DriverStation.silenceJoystickConnectionWarning(true);                                                                     // -> Silences Joystick Connection Warning
    }
  }
  // ---------------------------------------------------------------------------------//    Robot Prdic    //------------------------------------------------------------------------------------
  @Override
  public void robotPeriodic() {                                                                                                         // Called every (20ms) in all modes, this is triggered after mode specific periodics
    CommandScheduler.getInstance().run();
  }
  // -----------------------------------------------------------------------------------//    Stop Init    //------------------------------------------------------------------------------------
  @Override
  public void disabledInit() {                                                                                                          // Run at moment of start of disable
    m_robotContainer.setMotorBrake(true);                                                                                           // Sets motor controllers to brake mode on swerve
    disabledTimer.reset();
    disabledTimer.start();
  }
  // ----------------------------------------------------------------------------------//    Stop Prdic    //------------------------------------------------------------------------------------
  @Override
  public void disabledPeriodic() {                                                                                                      // Periodic run during robot disabled state

  }
  // -----------------------------------------------------------------------------------//    Stop Exit    //------------------------------------------------------------------------------------
  @Override
  public void disabledExit() {                                                                                                          // Run on exit of robot disabled state

  }
  // -----------------------------------------------------------------------------------//    Auto Init    //------------------------------------------------------------------------------------
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    
  }
  // ----------------------------------------------------------------------------------//    Auto Prdic    //------------------------------------------------------------------------------------
  @Override
  public void autonomousPeriodic() {}
  // -----------------------------------------------------------------------------------//    Auto Exit    //------------------------------------------------------------------------------------
  @Override
  public void autonomousExit() {}
  // -----------------------------------------------------------------------------------//    Tele Init    //------------------------------------------------------------------------------------
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
  // ----------------------------------------------------------------------------------//    Tele Prdic    //------------------------------------------------------------------------------------
  @Override
  public void teleopPeriodic() {}
  // -----------------------------------------------------------------------------------//    Tele Exit    //------------------------------------------------------------------------------------
  @Override
  public void teleopExit() {}
  // -----------------------------------------------------------------------------------//    Test Init    //------------------------------------------------------------------------------------
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
  // ----------------------------------------------------------------------------------//    Test Prdic    //------------------------------------------------------------------------------------
  @Override
  public void testPeriodic() {}
  // -----------------------------------------------------------------------------------//    Test Exit    //------------------------------------------------------------------------------------
  @Override
  public void testExit() {}
}
