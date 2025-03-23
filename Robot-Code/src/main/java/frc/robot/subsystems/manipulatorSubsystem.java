package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.operatorDriver;
import frc.robot.Constants.operatorManip;
import frc.robot.tools.utils;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;





public class manipulatorSubsystem extends SubsystemBase {
    private static Timer lastZerod                                              = new Timer();                                          // {@param elevatorZero_lapsedTime} Time Elapsed Since Last Zero'd
    private static Timer intakeRunTimer                                         = new Timer();
    private static int elevatorZero                                             = 0;                                                    // {@param elevatorZero_Count} ++ for every zero reset
    private static SparkClosedLoopController elevatorController_Motion          = Robot.elevatorMotor_Lead.getClosedLoopController();   // Retrievs Rev MaxMotion Closed Loop Controller                   
    // ------------------------------------------------------------------------------------    Command(s)    ------------------------------------------------------------------------------------
    public Command idle() {                                                                                                      // Holds Parts Idle
        return run(() ->{
            Robot.coralMotor.set(0);
        });
    }
    public Command autoFeederIntake() {                                                                                                 // 
        return run(() ->{
            intakeRunTimer.reset();
            utils.Logging(4, "Intake");
            if (!Robot.elevatorSensor_CorLoad.get()) {
                intakeRunTimer.start();
                while(!Robot.elevatorSensor_CorLoad.get() && intakeRunTimer.get() < 5.0) {
                    Robot.coralMotor.set(Constants.config.coral.autoFeed);
                }
                Robot.coralMotor.set(0);
            } else if (Robot.elevatorSensor_CorLoad.get()) {
                intakeRunTimer.start();
                while(Robot.elevatorSensor_CorEmpty.get() && intakeRunTimer.get() < 1.5) {
                    Robot.coralMotor.set(Constants.config.coral.autoEject);
                }
                Robot.coralMotor.set(0);
            } else {
                Robot.coralMotor.set(0);
            }
            intakeRunTimer.stop();
        });
    }
    public Command autoScore(int level) {                                                                                               // 
        return run(() ->{
            utils.Logging(4, "Elevator Floor: "+level);
            switch (level) {
                case 1:                                                                                                                 // 
                    runToTarget(Constants.config.elevator.reef_L1 - Constants.config.elevator.heightAtZero);
                    utils.Logging(3, "Height: "+ (Constants.config.elevator.reef_L1 - Constants.config.elevator.heightAtZero));
                    break;
                case 2:                                                                                                                 // 
                    runToTarget(Constants.config.elevator.reef_L2 - Constants.config.elevator.heightAtZero);
                    utils.Logging(3, "Height: "+ (Constants.config.elevator.reef_L2 - Constants.config.elevator.heightAtZero));
                    break;
                case 3:                                                                                                                 // 
                    runToTarget(Constants.config.elevator.reef_L3 - Constants.config.elevator.heightAtZero);
                    utils.Logging(3, "Height: "+ (Constants.config.elevator.reef_L3 - Constants.config.elevator.heightAtZero));
                    break;
                case 4:                                                                                                                 // 
                    runToTarget(Constants.config.elevator.reef_L4 - Constants.config.elevator.heightAtZero);
                    utils.Logging(3, "Height: "+ (Constants.config.elevator.reef_L4 - Constants.config.elevator.heightAtZero));
                    break;
                default:                                                                                                                // 
                    runToTarget(0);
                    break;
            }
        });
    }
    public Command manualElevator(double motionFeed) {                                                                                                 // Manually raises elevator
        return run(() ->{
            if (motionFeed > 0 && !Robot.elevatorSensor_Max.isPressed()) {

            } else if (motionFeed < 0 && !Robot.elevatorSensor_Zero.isPressed()) {

            } else {

            }
        });
    }
    public Command manualFeed(double motionFeed) {                                                                                                       // Manually feeds coral into coral manipulator
        return run(() ->{
        });
    }
    public Command manualScore() {                                                                                                      // Manually scores coral from coral manipulator
        return run(() ->{
        });
    }
    public Command algaeAuto() {
        return run(() ->{
            if (Robot.algaeSensor_Capture.get()) {

            } else {
                
            }
        });
    }
    // -----------------------------------------------------------------------------------    Periodic(s)    ------------------------------------------------------------------------------------
    public manipulatorSubsystem() {                                                                                                     // Creates Manipulator Subsystem AKA: Initialization
        
    }
    @Override
    public void periodic () {                                                                                                           // Scheduled every 20ms by Subsystem Base
        dashboardUpdate();
        if (DriverStation.isTestEnabled()) {                                                                                            // Runs if Driverstation is in Test and Enabled
            dashboardTest();
        }
        encoderZero();

        //algaeManip();
        
    }

    
    // ------------------------------------------------------------------------------------    Functions    -------------------------------------------------------------------------------------
    private static void dashboardUpdate() {                                                                                             // Updates Dashboard Data
        SmartDashboard.putNumber("manipulator/elevator/height", Robot.elevatorEncoder_Lead.getPosition());                                   // Gives callout of current height
        SmartDashboard.putNumber("manipulator/elevator/motionSpeed", Robot.elevatorMotor_Lead.get());                                           // Gives speed of elevator currently running
        SmartDashboard.putNumber("manipulator/elevator/lastUpdatedZero", lastZerod.get());                                                      // Counts time from last zeroing event
        SmartDashboard.putNumber("manipulator/elevator/zeroingEvents", elevatorZero);                                                           // Count of times zeroing has occured
        SmartDashboard.putBoolean("manipulator/coral/load", Robot.elevatorSensor_CorLoad.get());
        SmartDashboard.putBoolean("manipulator/coral/clear", Robot.elevatorSensor_CorEmpty.get());
        SmartDashboard.putNumber("manipulator/algae/arm",  Robot.algaeMotorIntakeEncoder.getPosition());
        SmartDashboard.updateValues();

    }
    private static void dashboardTest() {                                                                                               // Starts and Updates Values if in `Test Function`
        utils.Logging(0, "Elevator Height :"+Robot.elevatorEncoder_Lead.getPosition());
    }
    private static void encoderZero() {                                                                                                 // function to watch for zeroing event
        if(Robot.elevatorSensor_Zero.isPressed()) {                                                                                         // if reed switch is engaged runs
            Robot.elevatorEncoder_Lead.setPosition(0);                                                                               //updates encoder to zero
            if (lastZerod.isRunning()) {                                                                                                // checks if timer is running
                lastZerod.restart();                                                                                                    // starts timeer on zeroing event
            } else {
                lastZerod.reset();                                                                                                      // sets timer to zero
                lastZerod.start();                                                                                                      // starts timer
            }                                                                                                          
        } else {                                                                                                                        // if elevator reed switch is not engaged
            
        }
    }
    private static void runToTarget(double targetHeight) {                                                                              // Runs to height from floor
        targetHeight = targetHeight - Constants.config.elevator.heightAtZero;                                                           // Updates target run height to match height from floor              
        elevatorController_Motion.setReference(targetHeight, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);                                    // Calls Rev Motion MaxMotion with set height Position
    }

    private static void algaeManip() {
        double manip_Speed = Robot.operatorManip.getLeftY();
        Robot.algaeMotorIntake.set(manip_Speed);
        double kicker_Speed = Robot.operatorManip.getRightTriggerAxis();
        Robot.algaeMotorFeed.set(kicker_Speed);
    }

        // 360 degree 
        // 90 degree rotation 
        // that would be 25% of 360 
        // 100 encoder rotations is 1 rotation
        // 

}