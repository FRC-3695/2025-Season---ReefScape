package frc.robot.subsystems;
import frc.robot.Robot;

import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class manipulatorSubsystem extends SubsystemBase {
    private SparkClosedLoopController elevatorControl       = Robot.elevatorMotor_Lead.getClosedLoopController();                       // Retrievs Rev MaxMotion Closed Loop Controller
    // ------------------------------------------------------------------------------------    Command(s)    ------------------------------------------------------------------------------------
    public manipulatorSubsystem() {                                                                                                     // Creates Manipulator Subsystem AKA: Initialization
        
    }
    // -----------------------------------------------------------------------------------    Periodic(s)    ------------------------------------------------------------------------------------
    @Override
    public void periodic () {                                                                                                           // Scheduled every 20ms by Subsystem Base
        dashboardUpdate();
        if (DriverStation.isTestEnabled()) {                                                                                            // Runs if Driverstation is in Test and Enabled
            dashboardTest();
        }
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

    }
}
