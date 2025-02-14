package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.tools.utils;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class manipulatorSubsystem extends SubsystemBase {
    private static Timer lastZerod                                              = new Timer();                                          // {@param elevatorZero_lapsedTime} Time Elapsed Since Last Zero'd
    private static int elevatorZero                                             = 0;                                                    // {@param elevatorZero_Count} ++ for every zero reset
    private static SparkClosedLoopController elevatorController_Motion          = Robot.elevatorMotor_Lead.getClosedLoopController();   // Retrievs Rev MaxMotion Closed Loop Controller                   
    // ------------------------------------------------------------------------------------    Command(s)    ------------------------------------------------------------------------------------
    public Command autoFeederIntake() {                                                                                                 // 
        return run(() ->{
        });
    }
    public Command autoScore(int level) {                                                                                               // 
        return run(() ->{
            switch (level) {
                case 1:                                                                                                                 // 
                    runToTarget(Constants.config.elevator.reef_L1 - Constants.config.elevator.heightAtZero);
                    break;
                case 2:                                                                                                                 // 
                    runToTarget(Constants.config.elevator.reef_L2 - Constants.config.elevator.heightAtZero);
                    break;
                case 3:                                                                                                                 // 
                    runToTarget(Constants.config.elevator.reef_L3 - Constants.config.elevator.heightAtZero);
                    break;
                case 4:                                                                                                                 // 
                    runToTarget(Constants.config.elevator.reef_L4 - Constants.config.elevator.heightAtZero);
                    break;
                default:                                                                                                                // 
                    runToTarget(0);
                    break;
            }
        });
    }
    public Command manualElevatorUp() {                                                                                                 // Manually raises elevator
        return run(() ->{
        });
    }
    public Command manualElevatorDown() {                                                                                               // Manually lowers elevator
        return run(() ->{
        });
    }
    public Command manualFeed() {                                                                                                       // Manually feeds coral into coral manipulator
        return run(() ->{
        });
    }
    public Command manualScore() {                                                                                                      // Manually scores coral from coral manipulator
        return run(() ->{
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
    }
    // ------------------------------------------------------------------------------------    Functions    -------------------------------------------------------------------------------------
    private static void dashboardUpdate() {                                                                                             // Updates Dashboard Data
        SmartDashboard.putNumber("elevator/height", Robot.elevatorEncoder_LeadAlt.getPosition());                                   // Gives callout of current height
        SmartDashboard.putNumber("elevator/motionSpeed", Robot.elevatorMotor_Lead.get());                                           // Gives speed of elevator currently running
        SmartDashboard.putNumber("elevator/lastUpdatedZero", lastZerod.get());                                                      // Counts time from last zeroing event
        SmartDashboard.putNumber("elevator/zeroingEvents", elevatorZero);                                                           // Count of times zeroing has occured
        SmartDashboard.updateValues();
    }
    private static void dashboardTest() {                                                                                               // Starts and Updates Values if in `Test Function`
        utils.Logging(0, "Elevator Height :"+Robot.elevatorEncoder_LeadAlt.getPosition());
    }
    private static void encoderZero() {                                                                                                 // function to watch for zeroing event
        if(Robot.elevatorSensor_3sZero.get()) {                                                                                         // if reed switch is engaged runs
            Robot.elevatorEncoder_LeadAlt.setPosition(0);                                                                               //updates encoder to zero
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
        elevatorController_Motion.setReference(targetHeight, ControlType.kMAXMotionPositionControl);                                    // Calls Rev Motion MaxMotion with set height Position
    }
}
