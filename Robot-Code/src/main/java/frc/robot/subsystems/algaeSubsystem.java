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

public class algaeSubsystem extends SubsystemBase {
    private static Timer algaeFeed_RunTimer                                     = new Timer();
    // ------------------------------------------------------------------------------------    Command(s)    ------------------------------------------------------------------------------------
    public Command core() {
        return run(() ->{

        });
    }
    public Command manual_OR() {
        return run(() ->{
            utils.Logging(4, "-");
            if (!Robot.operatorManip.leftBumper().getAsBoolean()) {
            speedAsign(Robot.operatorManip.getLeftTriggerAxis());
            } else {
            speedAsign(-Robot.operatorManip.getLeftTriggerAxis());
            }
        });
    }
    // -----------------------------------------------------------------------------------    Periodic(s)    ------------------------------------------------------------------------------------
    public algaeSubsystem() {                                                                                                     // Creates Manipulator Subsystem AKA: Initialization
        
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
        SmartDashboard.putNumber("manipulator/algae/Pose",  Robot.algaeMotorIntakeEncoder.getPosition());
        SmartDashboard.putBoolean("manipulator/algae/sensor/poseZero", Robot.algaeSensor_Zero.get());
        SmartDashboard.putBoolean("manipulator/algae/sensor/algaeCapture", Robot.algaeSensor_Capture.get());
        SmartDashboard.putNumber("manipulator/algae/sensor/poseMotorCurrent", Robot.algaeMotor_Position.getOutputCurrent());
        SmartDashboard.updateValues();

    }
    private static void dashboardTest() {                                                                                               // Starts and Updates Values if in `Test Function`
    }
    public static void algaeManip_runToAngle(int Angle) {

    }
    public static void algaeManip_deployToZero() {
        double ampDraw = 0;
        utils.Logging(4, "Auto Deploying Algae & Zero");
        if (Robot.algaeSensor_Zero.get()) {
            while (Robot.algaeSensor_Zero.get()) {
                Robot.algaeMotor_Position.set(Constants.config.algae.autoDeploy_SPD);
                ampDraw = Robot.algaeMotor_Position.getOutputCurrent();
            }
        }
        if (!Robot.algaeSensor_Zero.get()) {
            while (Robot.algaeMotor_Position.getOutputCurrent() <= (ampDraw * Constants.config.algae.varAmpDeploy) || Robot.algaeMotor_Position.getOutputCurrent() < Constants.config.algae.maxAmpDeploy) {
                Robot.algaeMotor_Position.set(Constants.config.algae.autoDeploy_SPD / 4);
            }
        }
        Robot.algaeMotor_Position.set(0);
    }
    public static void speedAsign(double speed) {
        Robot.algaeMotor_Intake.set(speed);
    }
    public static void intake() {
        Robot.algaeMotor_Intake.set(Constants.config.algae.autoFeed);
    }
    public static void eject() {
        Robot.algaeMotor_Intake.set(Constants.config.algae.autoEject);
    }
    public static void stop() {
        Robot.algaeMotor_Intake.set(0);
    }
}
