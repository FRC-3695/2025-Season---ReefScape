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

public class elevatorSubsystem extends SubsystemBase {
    private static Timer lastZerod                                              = new Timer();                                          // {@param elevatorZero_lapsedTime} Time Elapsed Since Last Zero'd
    private static int elevatorZero                                             = 0;                                                    // {@param elevatorZero_Count} ++ for every zero reset
    private static SparkClosedLoopController elevatorMotionController           = Robot.elevatorMotor_Lead.getClosedLoopController();   // Retrievs Rev MaxMotion Closed Loop Controller                   
    // ------------------------------------------------------------------------------------    Command(s)    ------------------------------------------------------------------------------------
    public Command idle() {                                                                                                      // Holds Parts Idle
        return run(() ->{
            Robot.elevatorMotor_Lead.set(0);
           
        });
    }
    public Command autoScore(int level) {                                                                                               // 
        return run(() ->{
            utils.Logging(4, "Elevator Floor: "+level);
            switch (level) {
                case 1:                                                                                                                 // 
                    runToTarget(Constants.config.elevator.reef_L1);
                    utils.Logging(3, "Height: "+ (Constants.config.elevator.reef_L1 - Constants.config.elevator.heightAtZero));
                    break;
                case 2:                                                                                                                 // 
                    runToTarget(Constants.config.elevator.reef_L2);
                    utils.Logging(3, "Height: "+ (Constants.config.elevator.reef_L2 - Constants.config.elevator.heightAtZero));
                    break;
                case 3:                                                                                                                 // 
                    runToTarget(Constants.config.elevator.reef_L3);
                    utils.Logging(3, "Height: "+ (Constants.config.elevator.reef_L3 - Constants.config.elevator.heightAtZero));
                    break;
                case 4:                                                                                                                 // 
                    runToTarget(Constants.config.elevator.reef_L4);
                    utils.Logging(3, "Height: "+ (Constants.config.elevator.reef_L4 - Constants.config.elevator.heightAtZero));
                    break;
                default:                                                                                                                // 
                    runToTarget(0);
                    break;
            }
        });
    }
    public Command manualElevator(double input) {                                                                                                 // Manually raises elevator
        return run(() ->{
            utils.Logging(4, "Elevator Manual Power:"+input);
        });
    }
    // -----------------------------------------------------------------------------------    Periodic(s)    ------------------------------------------------------------------------------------
    public elevatorSubsystem() {                                                                                                     // Creates Manipulator Subsystem AKA: Initialization
        
    }
    @Override
    public void periodic () {                                                                                                           // Scheduled every 20ms by Subsystem Base
        dashboardUpdate();
        if (DriverStation.isTestEnabled()) {                                                                                            // Runs if Driverstation is in Test and Enabled
            dashboardTest();
        }
        encoderZero();
        elevatorManual();
    }
    // ------------------------------------------------------------------------------------    Functions    -------------------------------------------------------------------------------------
    private static void dashboardUpdate() {                                                                                             // Updates Dashboard Data
        SmartDashboard.putNumber("manipulator/elevator/height", Robot.elevatorEncoder_Lead.getPosition());                                   // Gives callout of current height
        SmartDashboard.putNumber("manipulator/elevator/runSpeed", Robot.elevatorMotor_Lead.get());                                           // Gives speed of elevator currently running
        SmartDashboard.putNumber("manipulator/elevator/lastUpdatedZero", lastZerod.get());                                                      // Counts time from last zeroing event
        SmartDashboard.putNumber("manipulator/elevator/zeroingEvents", elevatorZero);                                                           // Count of times zeroing has occured
        SmartDashboard.putBoolean("manipulator/elevator/sensor/Max", Robot.elevatorSensor_Max.isPressed());
        SmartDashboard.putBoolean("manipulator/elevator/sensor/Zero", Robot.elevatorSensor_Zero.isPressed());
        SmartDashboard.updateValues();

    }
    private static void dashboardTest() {                                                                                               // Starts and Updates Values if in `Test Function`
        utils.Logging(0, "Elevator Height :"+Robot.elevatorEncoder_Lead.getPosition());
    }
    private static void encoderZero() {                                                                                                 // function to watch for zeroing event
        if(Robot.elevatorSensor_Zero.isPressed() && (Robot.elevatorEncoder_Lead.getPosition() > 2 || Robot.elevatorEncoder_Lead.getPosition() < 0)) {                                                                                         // if reed switch is engaged runs
            Robot.elevatorEncoder_Lead.setPosition(0);
            elevatorZero ++;                                                                               //updates encoder to zero
            if (lastZerod.isRunning()) {                                                                                                // checks if timer is running
                lastZerod.restart();                                                                                                    // starts timeer on zeroing event
            } else {
                lastZerod.reset();                                                                                                      // sets timer to zero
                lastZerod.start();                                                                                                      // starts timer
            }                                                                                                          
        }
    }
    public static void runToTarget(double targetHeight, boolean adjusted) {                                                                              // Runs to height from floor
        if (!adjusted) {
            targetHeight = targetHeight - Constants.config.elevator.heightAtZero;                                                           // Updates target run height to match height from floor              
        }
        elevatorMotionController.setReference(targetHeight, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);                                    // Calls Rev Motion MaxMotion with set height Position
    }
    public static void runToTarget(double targetHeight) {
        runToTarget(targetHeight, false);
    }
    public static void elevatorManual() {
        if (Robot.operatorManip.getLeftY() >= Constants.config.elevator.manualDeadBand || Robot.operatorManip.getLeftY() <= -Constants.config.elevator.manualDeadBand) {
            if (!Robot.elevatorSensor_Zero.isPressed() && -Robot.operatorManip.getLeftY() <= 0) {
                Robot.elevatorMotor_Lead.set((Constants.config.elevator.manualSpeed * -Robot.operatorManip.getLeftY())/4);
            } else if (!Robot.elevatorSensor_Max.isPressed() && -Robot.operatorManip.getLeftY() >= 0) {
                Robot.elevatorMotor_Lead.set((Constants.config.elevator.manualSpeed * -Robot.operatorManip.getLeftY()));
            }
            utils.Logging(4, "Elevator Manual Power:"+Robot.elevatorMotor_Lead.get());
        }  else {
            Robot.elevatorMotor_Lead.set(0);
        }
    }
}
