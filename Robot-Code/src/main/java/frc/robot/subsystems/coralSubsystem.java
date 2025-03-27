package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.tools.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;





public class coralSubsystem extends SubsystemBase {
    // ------------------------------------------------------------------------------------    Command(s)    ------------------------------------------------------------------------------------
    public Command core() {
        return run(() ->{
        });
    }
    public Command manual_OR() {
        return run(() ->{
            utils.Logging(4, "-");
            if (!Robot.operatorManip.rightBumper().getAsBoolean()) {
                speedAssign(Robot.operatorDriver.getLeftTriggerAxis());
              } else {
                speedAssign(-Robot.operatorDriver.getLeftTriggerAxis());
              }
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
    public static void speedAssign(double speed) {
        Robot.coralMotor.set(speed);
    }
    public static void intake() {
        Robot.coralMotor.set(Constants.config.coral.autoFeed);
    }
    public static void eject() {
        Robot.coralMotor.set(Constants.config.coral.autoEject);
    }
    public static void stop() {
        Robot.coralMotor.set(0);
    }
}