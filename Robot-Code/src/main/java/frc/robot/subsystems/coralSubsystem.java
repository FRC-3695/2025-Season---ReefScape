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





public class coralSubsystem extends SubsystemBase {
    private static Timer coral_RunTimer                                         = new Timer();
    private static String state                                                 = null;
    // ------------------------------------------------------------------------------------    Command(s)    ------------------------------------------------------------------------------------
    public Command idle() {                                                                                                      // Holds Parts Idle
        return run(() ->{
            Robot.coralMotor.set(0);
           
        });
    }
    public Command autoFeederIntake() {                                                                                                 // 
        return run(() ->{
            coral_RunTimer.reset();
            utils.Logging(4, "Intake");
            if (!Robot.elevatorSensor_CorLoad.get()) {
                coral_RunTimer.start();
                while(!Robot.elevatorSensor_CorLoad.get() && coral_RunTimer.get() < 5.0) {
                    Robot.coralMotor.set(Constants.config.coral.autoFeed);
                }
                Robot.coralMotor.set(0);
            } else if (Robot.elevatorSensor_CorLoad.get()) {
                coral_RunTimer.start();
                while(Robot.elevatorSensor_CorEmpty.get() && coral_RunTimer.get() < 1.5) {
                    Robot.coralMotor.set(Constants.config.coral.autoEject);
                }
                Robot.coralMotor.set(0);
            } else {
                Robot.coralMotor.set(0);
            }
            coral_RunTimer.stop();
        });
    }
    public Command manualScore() {                                                                                                      // Manually scores coral from coral manipulator
        return run(() ->{
        });
    }
    // -----------------------------------------------------------------------------------    Periodic(s)    ------------------------------------------------------------------------------------
    public coralSubsystem() {                                                                                                     // Creates Manipulator Subsystem AKA: Initialization
        
    }
    @Override
    public void periodic () {                                                                                                           // Scheduled every 20ms by Subsystem Base
        dashboardUpdate();
        if (DriverStation.isTestEnabled()) {                                                                                            // Runs if Driverstation is in Test and Enabled
            dashboardTest();
        }       
    }
    // ------------------------------------------------------------------------------------    Functions    -------------------------------------------------------------------------------------
    private static void dashboardUpdate() {                                                                                             // Updates Dashboard Data
        SmartDashboard.putBoolean("manipulator/coral/load", Robot.elevatorSensor_CorLoad.get());
        SmartDashboard.putBoolean("manipulator/coral/clear", Robot.elevatorSensor_CorEmpty.get());
        SmartDashboard.updateValues();
    }
    private static void dashboardTest() {                                                                                               // Starts and Updates Values if in `Test Function`
    }
}