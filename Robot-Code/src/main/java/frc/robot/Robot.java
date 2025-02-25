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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.tools.utils;
import swervelib.math.SwerveMath;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private Timer disabledTimer;
  // -----------------------------------------------------------------------------------    Controller(s)    ------------------------------------------------------------------------------------
  public final static CommandXboxController operatorDriver = new CommandXboxController(Constants.operatorDriver.port);                  // Driver's xBox controller
  public final static CommandXboxController operatorManip  = new CommandXboxController(Constants.operatorManip.port);                   // Operator's xBox controller
  // --------------------------------------------------------------------------------------    Motor(s)    --------------------------------------------------------------------------------------
  
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
    if (utils.RoboRIOid() == "compBot2024") {
      elevatorInit();
    }
    utils.GitInfo();                                                                                                                    // Displays current git status build on robot start
  }
  // ---------------------------------------------------------------------------------//    Robot Prdic    //------------------------------------------------------------------------------------
  @Override
  public void robotPeriodic() {                                                                                                         // Called every (20ms) in all modes, this is triggered after mode specific periodics
    CommandScheduler.getInstance().run();
  }
  // -----------------------------------------------------------------------------------//    Stop Init    //------------------------------------------------------------------------------------
  @Override
  public void disabledInit() {                                                                                                          // Run at moment of start of disable
    m_robotContainer.setMotorBrake(true);                                                                                         // Sets motor controllers to brake mode on swerve
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
  public void testPeriodic() {
    
  }
  // -----------------------------------------------------------------------------------//    Test Exit    //------------------------------------------------------------------------------------
  @Override
  public void testExit() {}
  // ----------------------------------------------------------------------------------//    Function(s)   //------------------------------------------------------------------------------------
  public void climberInit() {
    final SparkFlex         climberMotor_Paw              = new SparkFlex(Constants.CANnet.climber.Paw, MotorType.kBrushless);
    final AbsoluteEncoder   climberCodoer_PawEncoder      = climberMotor_Paw.getAbsoluteEncoder();
  }
  public void elevatorInit() {
    final SparkMax          elevatorMotor_Lead            = new SparkMax(Constants.CANnet.elevator.Lift_Master, MotorType.kBrushless);
    final SparkMax          elevatorMotor_Follower        = new SparkMax(Constants.CANnet.elevator.Lift_Follower, MotorType.kBrushless);
    final AbsoluteEncoder   elevatorEncoder_Lead          = elevatorMotor_Lead.getAbsoluteEncoder();
    final AbsoluteEncoder   elevatorEncoder_Follower      = elevatorMotor_Follower.getAbsoluteEncoder();
    final RelativeEncoder   elevatorEncoder_LeadAlt       = elevatorMotor_Lead.getAlternateEncoder();
    final DigitalInput      elevatorSensor_3sZero         = new DigitalInput(Constants.sensor.elevator0);                               // Reed Switch for when Elevator is Home
    final DigitalInput      elevatorSensor_CarZero        = new DigitalInput(Constants.sensor.elevatorCar0);                            // Optical Sensor to Check that Manipulator is in expected location
    final DigitalInput      elevatorSensor_CorLoad        = new DigitalInput(Constants.sensor.coralLoaded);                             // Optical Sensor for Coral Loaded
    final DigitalInput      elevatorSensor_CorEmpty       = new DigitalInput(Constants.sensor.coralEmpty);                              // Optical Sensor for Confirming Coral has been Deposited
    final SparkMaxConfig    elevatorMotorConfig_Global    = new SparkMaxConfig();
    final SparkMaxConfig    elevatorMotorConfig_Lead      = new SparkMaxConfig();
    final SparkMaxConfig    elevatorMotorConfig_Follower  = new SparkMaxConfig();
    elevatorMotorConfig_Global                                                                                                          // Global Config for Elevator Motors
      .inverted(false)                                                                                                         // Inverts Motors Motion if Needed
      .smartCurrentLimit(Constants.config.elevator.stallAmp)                                                                            // Sets Stall Amperage for Motor
      .idleMode(IdleMode.kBrake);                                                                                                       // Sets Braking Mode
    elevatorMotorConfig_Global.closedLoop                                                                                               // Global Config for Elevator Motors Closed Loop
      .pid(                                                                                                                             // PID Config `Slot 0`
        Constants.config.elevator.PIDF_P,
        Constants.config.elevator.PIDF_I,
        Constants.config.elevator.PIDF_D,
        ClosedLoopSlot.kSlot0
      )
      .velocityFF(Constants.config.elevator.velocityFF, ClosedLoopSlot.kSlot0)                                                          // Velocity Feed Forward Config `Slot 0`
      .outputRange(0, 1, ClosedLoopSlot.kSlot0);                                                                                        // Output Range Config `Slot 0`
    elevatorMotorConfig_Global.closedLoop.maxMotion                                                                                     // Global Config for Elevator MAX Motion by RevRobotics
      .maxAcceleration(Constants.config.elevator.accelerationMax, ClosedLoopSlot.kSlot0)                                                // Acceleration Mac `Slot 0`
      .maxVelocity(Constants.config.elevator.velocityMax, ClosedLoopSlot.kSlot0)                                                        // Velocity Max `Slot 0`
      .allowedClosedLoopError(1, ClosedLoopSlot.kSlot0);                                                                                // Error Lovel `Slot 0`
    elevatorMotorConfig_Lead.apply(elevatorMotorConfig_Global);                                                                         // Applies config from Global Config to Lead Motor Config
    elevatorMotorConfig_Lead.alternateEncoder                                                                                           // Sets config for Alternate Encoder Connected to Lead Motor Controller
    .positionConversionFactor(Constants.config.elevator.climbRatio)
    .countsPerRevolution(Constants.config.elevator.altReltEncoder);                                                                     // Steps per Revolution
    elevatorMotorConfig_Follower                                                                                                        // Elevator Motor Config for Follower
      .apply(elevatorMotorConfig_Global)                                                                                                // Applies config from Global Config to Follwer Motor Config
      .follow(Constants.CANnet.elevator.Lift_Master);                                                                                   // Sets CAN ID for Lead Motor and sets Follower Motor to Follow
    elevatorMotor_Lead.clearFaults();                                                                                                   // Clears Sticky Faults from Lead Elevator Motor Controller
    elevatorMotor_Lead.configure(elevatorMotorConfig_Lead, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);             // Loads config to Elevator Lead Motor
    elevatorMotor_Follower.clearFaults();                                                                                               // Clears Sticky Faults from Secondary Elevator Motor Controller
    elevatorMotor_Follower.configure(elevatorMotorConfig_Follower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);     // Loads config to Elevator Follower Motor
  
    }
  public void sensorConfig() {
    
  }
}