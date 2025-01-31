// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private Timer disabledTimer;
  // -----------------------------------------------------------------------------------    Controller(s)    ------------------------------------------------------------------------------------
  final static CommandXboxController operatorDriver = new CommandXboxController(Constants.operatorDriver.port);                         // Driver's xBox controller
  final static CommandXboxController operatorManip  = new CommandXboxController(Constants.operatorManip.port);                          // Operator's xBox controller
  // --------------------------------------------------------------------------------------    Motor(s)    --------------------------------------------------------------------------------------
  final static SparkMax elevatorMotor_Lead = new SparkMax(Constants.CANnet.elevator.Lift_Master, MotorType.kBrushless);
  final static SparkMax elevatorMotor_Follower = new SparkMax(Constants.CANnet.elevator.Lift_Follower, MotorType.kBrushless);
  final static SparkFlex climberMotor_Paw = new SparkFlex(Constants.CANnet.climber.Paw, MotorType.kBrushless);
  // -------------------------------------------------------------------------------------    Sensor(s)    --------------------------------------------------------------------------------------
  final static AbsoluteEncoder elevatorEncoder_Lead = elevatorMotor_Lead.getAbsoluteEncoder();
  final static AbsoluteEncoder elevatorEncoder_Follower = elevatorMotor_Follower.getAbsoluteEncoder();
  final static RelativeEncoder elevatorEncoder_LeadAlt = elevatorMotor_Lead.getAlternateEncoder();
  final static AbsoluteEncoder climberCodoer_PawEncoder = climberMotor_Paw.getAbsoluteEncoder();
  // ----------------------------------------------------------------------------------    Other Device(s)    -----------------------------------------------------------------------------------
  public static PowerDistribution powerHub = new PowerDistribution(Constants.CANnet.core_PowerHub, ModuleType.kRev);                    // {@param - powerHub} Power Distribution Hub
  // ----------------------------------------------------------------------------------//    Robot Init    //------------------------------------------------------------------------------------
  private static SparkMaxConfig elevatorMotorConfig_Global = new SparkMaxConfig();
  private static SparkMaxConfig elevatorMotorConfig_Lead = new SparkMaxConfig();
  private static SparkMaxConfig elevatorMotorConfig_Follower = new SparkMaxConfig();
  @Override
  public void robotInit() {                                                                                                             // Called on robot startup, best use is for anything needed to prep robot and ensure std. start state
    m_robotContainer = new RobotContainer();
    disabledTimer = new Timer();                                                                                                        // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop immediately when disabled, but then also let it be pushed
    if (isSimulation()) {                                                                                                               // Checks to see if code is running in simulation
      DriverStation.silenceJoystickConnectionWarning(true);                                                                     // -> Silences Joystick Connection Warning
    }
    motorConfig();
    sensorConfig();
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
  // ----------------------------------------------------------------------------------//    Function(s)   //------------------------------------------------------------------------------------
  public void motorConfig() {
    elevatorMotorConfig_Global                                                                                                          //
      .inverted(false)
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);
    elevatorMotorConfig_Global.closedLoop                                                                                               //
      .pid(                                                                                                                             //
        Constants.config.elevator.PIDF_P,
        Constants.config.elevator.PIDF_I,
        Constants.config.elevator.PIDF_D,
        ClosedLoopSlot.kSlot0
      )
      .velocityFF(Constants.config.elevator.velocityFF, ClosedLoopSlot.kSlot0)                                                          //
      .outputRange(0, 1, ClosedLoopSlot.kSlot0);                                                                                        //
    elevatorMotorConfig_Global.closedLoop.maxMotion                                                                                     //
      .maxAcceleration(Constants.config.elevator.accelerationMax, ClosedLoopSlot.kSlot0)                                                //
      .maxVelocity(Constants.config.elevator.velocityMax, ClosedLoopSlot.kSlot0)                                                        //
      .allowedClosedLoopError(1, ClosedLoopSlot.kSlot0);                                                                                //
    elevatorMotorConfig_Lead.apply(elevatorMotorConfig_Global);                                                                         //
    elevatorMotorConfig_Lead.alternateEncoder                                                                                           //
        .countsPerRevolution(Constants.config.elevator.altReltEncoder);
    elevatorMotorConfig_Follower                                                                                                        //
      .apply(elevatorMotorConfig_Global)
      .follow(Constants.CANnet.elevator.Lift_Master);
    elevatorMotor_Lead.clearFaults();                                                                                                   //
    elevatorMotor_Lead.configure(elevatorMotorConfig_Lead, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);             //
    elevatorMotor_Follower.clearFaults();                                                                                               //
    elevatorMotor_Follower.configure(elevatorMotorConfig_Follower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);     //
  }
  public void sensorConfig() {
    
  }
}