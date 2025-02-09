package frc.robot.subsystems;
import frc.robot.Robot;

import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class manipulatorSubsystem extends SubsystemBase {
    private static Timer lastZerod                          = new Timer();                                                              // Time Elapsed Since Last Zero'd
    //private SparkClosedLoopController elevatorControl       = Robot.elevatorMotor_Lead.getClosedLoopController();                       // Retrievs Rev MaxMotion Closed Loop Controller
    // ------------------------------------------------------------------------------------    Command(s)    ------------------------------------------------------------------------------------
    public Command autoFeederIntake() {
        return run(() ->{
        });
    }
    public Command autoScore(int level) {
        return run(() ->{
            switch (level) {
                case 1:
                    
                    break;
                case 2:
                    
                    break;
                case 3:
                    
                    break;
                case 4:
                    
                    break;
                default:
                    break;
            }
        });
    }
    public Command manualElevatorUp() {
        return run(() ->{
        });
    }
    public Command manualElevatorDown() {
        return run(() ->{
        });
    }
    public Command manualScoreIn() {
        return run(() ->{
        });
    }
    public Command manualScoreOut() {
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
        SmartDashboard.putNumber("height", 0);                                                                                          //
        SmartDashboard.putNumber("motionSpeed", 0);                                                                                     //
        SmartDashboard.putNumber("lastUpdatedZero", 0);                                                                                 //
        SmartDashboard.updateValues();
    }
    private static void dashboardTest() {                                                                                               // Starts and Updates Values if in `Test Function`

    }
    private static void encoderZero() {
        /*if(Robot.elevatorSensor_3sZero.get()) {
            Robot.elevatorEncoder_LeadAlt.setPosition(0);
        } else {

        }*/
    }
    private static void runToTarget(double target) {
        
    }
}
