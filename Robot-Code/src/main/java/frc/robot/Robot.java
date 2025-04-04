// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.algaeSubsystem;
import frc.robot.tools.utils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.*;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import limelight.Limelight;

import edu.wpi.first.wpilibj.DigitalInput;
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
  // --------------------------------------------------------------------------------    Vision Processor(s)    ---------------------------------------------------------------------------------
  //public final static Limelight         cameraForward                 = new Limelight("limelight-forward");                             // 
  //public final static Limelight         cameraRear                    = new Limelight("rearCamera");                                    // 
  // -----------------------------------------------------------------------------------    Controller(s)    ------------------------------------------------------------------------------------
  public final static CommandXboxController operatorDriver = new CommandXboxController(Constants.operatorDriver.port);                  // Driver's xBox controller
  public final static CommandXboxController operatorManip  = new CommandXboxController(Constants.operatorManip.port);                   // Operator's xBox controller
  // --------------------------------------------------------------------------------------    Motor(s)    --------------------------------------------------------------------------------------
  public final static SparkMax          elevatorMotor_Lead            = new SparkMax(Constants.CANnet.elevator.Lift_Master, MotorType.kBrushless);
  public final static SparkMax          elevatorMotor_Follower        = new SparkMax(Constants.CANnet.elevator.Lift_Follower, MotorType.kBrushless);
  public final static SparkMaxConfig    elevatorMotorConfig_Global    = new SparkMaxConfig();
  public final static SparkMaxConfig    elevatorMotorConfig_Lead      = new SparkMaxConfig();
  public final static SparkMaxConfig    elevatorMotorConfig_Follower  = new SparkMaxConfig();
  public final static SparkFlex         coralMotor                    = new SparkFlex(Constants.CANnet.manipulators.Coral_Feed, MotorType.kBrushless);
  public final static SparkFlexConfig   coralMotorConfig              = new SparkFlexConfig();
  public final static SparkFlex         algaeMotor_Position           = new SparkFlex(Constants.CANnet.manipulators.Algae_Intake, MotorType.kBrushless);
  public final static SparkFlex         algaeMotor_Intake             = new SparkFlex(Constants.CANnet.manipulators.Algae_Feed, MotorType.kBrushless);
  public final static SparkFlexConfig   algaeMotorConfig_Position     = new SparkFlexConfig();
  public final static SparkFlexConfig   algaeMotorConfig_Intake     = new SparkFlexConfig();
  // -------------------------------------------------------------------------------------    Sensor(s)    --------------------------------------------------------------------------------------
  public final static RelativeEncoder   elevatorEncoder_Lead          = elevatorMotor_Lead.getEncoder();
  public final static SparkLimitSwitch  elevatorSensor_Zero           = elevatorMotor_Lead.getReverseLimitSwitch();                     // Reed Switch for when Elevator is Home
  public final static SparkLimitSwitch  elevatorSensor_Max            = elevatorMotor_Lead.getForwardLimitSwitch();                     // Reed Switch for when Elevator is Fully Deployed
  public final static DigitalInput      elevatorSensor_CorLoad        = new DigitalInput(Constants.sensor.coralLoaded);                 // Optical Sensor for Coral Loaded
  public final static DigitalInput      elevatorSensor_CorEmpty       = new DigitalInput(Constants.sensor.coralEmpty);                  // Optical Sensor for Confirming Coral has been Deposited
  public final static RelativeEncoder   algaeMotorIntakeEncoder       = algaeMotor_Position.getEncoder();
  public final static DigitalInput      algaeSensor_Zero              = new DigitalInput(Constants.sensor.algaeZero);
  public final static DigitalInput      algaeSensor_Capture           = new DigitalInput(Constants.sensor.algaeLoaded);

  // ----------------------------------------------------------------------------------    Other Device(s)    -----------------------------------------------------------------------------------
  public static PowerDistribution powerHub = new PowerDistribution(Constants.CANnet.core_PowerHub, ModuleType.kRev);                    // {@param - powerHub} Power Distribution Hub
  // ----------------------------------------------------------------------------------//    Robot Init    //------------------------------------------------------------------------------------
  @Override
  public void robotInit() {                                                                                                             // Called on robot startup, best use is for anything needed to prep robot and ensure std. start state
    utils.GitInfo();                                                                                                                    // Displays current git status build on robot start
    m_robotContainer = new RobotContainer();                                                                                            // Setup RobotContainer
    disabledTimer = new Timer();                                                                                                        // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop immediately when disabled, but then also let it be pushed
    if (isSimulation()) {                                                                                                               // Checks to see if code is running in simulation
      DriverStation.silenceJoystickConnectionWarning(true);                                                                     // -> Silences Joystick Connection Warning
    }
    switch (utils.RoboRIOid()) {                                                                                                        // Gets roboRio Identity {@param defaultSwitchCase}
      case "compBot2024":                                                                                                               // *> Runs for Competition Robot then >> default
        elevatorInit();
        coralInit();
        algaeInit();
      case "devBot2024":                                                                                                                // *> Runs for Development Robot then >> default
      default:                                                                                                                          // *> Runs if Switch Case's not met or for Case's that are not broken on call

        break;
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
  public void autonomousPeriodic() {
    algaeSubsystem.algaeManip_deployToZero();
  }
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
  public void teleopPeriodic() {

  }
  // -----------------------------------------------------------------------------------//    Tele Exit    //------------------------------------------------------------------------------------
  @Override
  public void teleopExit() {}
  // -----------------------------------------------------------------------------------//    Test Init    //------------------------------------------------------------------------------------
  @Override
  public void testInit() {
    utils.debugEnable(true);                                                                                                            // Debug Enabled Automaticly during Test
    CommandScheduler.getInstance().cancelAll();
  }
  // ----------------------------------------------------------------------------------//    Test Prdic    //------------------------------------------------------------------------------------
  @Override
  public void testPeriodic() {
    
  }
  // -----------------------------------------------------------------------------------//    Test Exit    //------------------------------------------------------------------------------------
  @Override
  public void testExit() {
    
  }
  // ----------------------------------------------------------------------------------//    Function(s)   //------------------------------------------------------------------------------------
  private void elevatorInit() {
    elevatorMotorConfig_Global                                                                                                          // Global Config for Elevator Motors
      .inverted(false)                                                                                                                  // Inverts Motors Motion if Needed
      .smartCurrentLimit(Constants.config.elevator.stallAmp)                                                                            // Sets Stall Amperage for Motor
      .idleMode(IdleMode.kBrake);                                                                                                       // Sets Braking Mode
    elevatorMotorConfig_Global.closedLoop 
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)                                                                                              // Global Config for Elevator Motors Closed Loop
      .pid(                                                                                                                             // PID Config `Slot 0`
        Constants.config.elevator.PIDF_P,
        Constants.config.elevator.PIDF_I,
        Constants.config.elevator.PIDF_D,
        ClosedLoopSlot.kSlot0
      )
      .velocityFF(Constants.config.elevator.velocityFF, ClosedLoopSlot.kSlot0)                                                          // Velocity Feed Forward Config `Slot 0`
      .outputRange(Constants.config.elevator.output_Min, Constants.config.elevator.output_Max, ClosedLoopSlot.kSlot0);                                                                                        // Output Range Config `Slot 0`
    elevatorMotorConfig_Global.closedLoop.maxMotion                                                                                     // Global Config for Elevator MAX Motion by RevRobotics
      .maxAcceleration(Constants.config.elevator.accelerationMax, ClosedLoopSlot.kSlot0)                                                // Acceleration Mac `Slot 0`
      .maxVelocity(Constants.config.elevator.velocityMax, ClosedLoopSlot.kSlot0)                                                        // Velocity Max `Slot 0`
      .allowedClosedLoopError(2, ClosedLoopSlot.kSlot0);                                                                                // Error Lovel `Slot 0`
    elevatorMotorConfig_Lead.apply(elevatorMotorConfig_Global);                                                                         // Applies config from Global Config to Lead Motor Config
    elevatorMotorConfig_Lead.encoder
      .positionConversionFactor(Constants.config.elevator.conversionFact);
    elevatorMotorConfig_Lead.limitSwitch                                                                                                // Configuration of Limit Switches
      .forwardLimitSwitchType(Type.kNormallyOpen)                                                                                       // Sets NC or NO status of Switch
      .forwardLimitSwitchEnabled(true)                                                                                          // Enables Limit Switch
      .reverseLimitSwitchType(Type.kNormallyOpen)                                                                                       // Sets NC or NO status of Switch
      .reverseLimitSwitchEnabled(true);                                                                                         // Enables Limit Switch
    elevatorMotorConfig_Follower                                                                                                        // Elevator Motor Config for Follower
      .apply(elevatorMotorConfig_Global)                                                                                                // Applies config from Global Config to Follwer Motor Config
      .follow(Constants.CANnet.elevator.Lift_Master);                                                                                   // Sets CAN ID for Lead Motor and sets Follower Motor to Follow
    elevatorMotor_Lead.clearFaults();                                                                                                   // Clears Sticky Faults from Lead Elevator Motor Controller
    elevatorMotor_Lead.configure(elevatorMotorConfig_Lead, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);             // Loads config to Elevator Lead Motor
    elevatorMotor_Follower.clearFaults();                                                                                               // Clears Sticky Faults from Secondary Elevator Motor Controller
    elevatorMotor_Follower.configure(elevatorMotorConfig_Follower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);     // Loads config to Elevator Follower Motor
  }
  private void coralInit() {
   coralMotorConfig
      .inverted(false)
      .smartCurrentLimit(Constants.config.coral.stallAmp)
      .voltageCompensation(11.8)
      .idleMode(IdleMode.kCoast);
    coralMotor.clearFaults();
    coralMotor.configure(coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  private void algaeInit() {
    algaeMotorConfig_Position
      .inverted(true)
      .smartCurrentLimit(Constants.config.algae.stallAmp)
      .idleMode(IdleMode.kBrake);
    algaeMotor_Position.clearFaults();
    algaeMotor_Position.configure(algaeMotorConfig_Position, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeMotorConfig_Intake
      .inverted(false)
      .smartCurrentLimit(Constants.config.algae.stallAmp)
      .idleMode(IdleMode.kBrake);
    algaeMotor_Intake.clearFaults();
    algaeMotor_Intake.configure(algaeMotorConfig_Intake, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}