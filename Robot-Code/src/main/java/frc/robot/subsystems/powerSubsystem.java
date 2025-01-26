package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.swerveSubsystem;
import frc.robot.tools.utils;

public class powerSubsystem extends SubsystemBase {
    static boolean startupRun = false;
    static Boolean[] powerHubPorts;
    static String[] swModuleName = {"FL", "FR", "BL", "BR"};                                                                            // {@Param - }Swerve Module Naming according to YAGSL Order
    // ------------------------------------------------------------------------------------    Functions    -------------------------------------------------------------------------------------
    private void hubChannelPopulated() {                                                                                                // Serves to create map of active power hub channels prior to risk of tripped breakers not counting in dead shorts
        powerHubPorts = new Boolean[Robot.powerHub.getNumChannels()];                                                                   // {@param - powerHubPorts} an array set to document active ports on the hub
        for (int channel = 0; channel < powerHubPorts.length; channel++) {                                                              // Scns through power hub slots to find active powerHub Channels
            if (Robot.powerHub.getCurrent(channel) > 0) {
                powerHubPorts[channel] = true;                                                                                          // Documents channel as in use if current draw is sensed
                utils.Logging(0, "Device sensed on power hub port :" + channel);
            } else {
                powerHubPorts[channel] = false;                                                                                         // Documents channel as not used if current draw is not sensed
                utils.Logging(0, "Device sensed on power hub port :" + channel);
            }
        }
    }
    private void driveTrain() {                                                                                                         // Checks power health of drive train
         for (int module = 0; module <4; module++) {
            utils.Power.voltageDrop(
                swModuleName[module] + " Drive Vortex", 
                Robot.powerHub.getVoltage(), 
                RobotContainer.drivebase.getSwerveModVoltage_Drive(module), 
                Constants.power.managment.sw_vtDrop_tol
            );
            utils.Power.voltageDrop(
                swModuleName[module] + " Steering Vortex",
                Robot.powerHub.getVoltage(),
                RobotContainer.drivebase.getSwerveModVoltage_Steer(module),
                Constants.power.managment.sw_vtDrop_tol
            );
            utils.Power.amperageCheck(
                swModuleName[module] + " Drive Vortex",
                0,
                Constants.power.managment.swD_SurgeAmp,
                Constants.power.managment.swD_CritAmp,
                Constants.power.managment.swD_BreakAmp,
                Constants.power.managment.swD_BreakTime
            );
            utils.Power.amperageCheck(
                swModuleName[module] + " Steering Vortex",
                0,
                Constants.power.managment.swD_SurgeAmp,
                Constants.power.managment.swD_CritAmp,
                Constants.power.managment.swD_BreakAmp,
                Constants.power.managment.swD_BreakTime
            );
            RobotContainer.drivebase.checkModuleConnectivity(module);                                                                   // Checks all devices on swerve module for connectivity
        }
    }
    private void startUp() {                                                                                                            // Run only within first 10s for RoboRIO if not completed
        if (startupRun) {return;};                                                                                                      // If start up code has been run escape the function to prevent duplicate run
        Robot.powerHub.clearStickyFaults();                                                                                             // Clear recorded faults on power hub so diagnostics can be stored for later download that are relevent
        Robot.powerHub.resetTotalEnergy();                                                                                              // Reset energy consumption as to give an ide to energy consumed per match or run
        hubChannelPopulated();
    }
    // -----------------------------------------------------------------------------------    Periodic(s)    ------------------------------------------------------------------------------------
    @Override
    public void periodic () {                                                                                                           // Scheduled roughly once every 20ms to run
        
    }
}