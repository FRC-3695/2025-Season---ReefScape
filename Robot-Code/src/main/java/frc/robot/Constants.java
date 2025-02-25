package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public class Constants {
    public static final class robot {
        public static final double ROBOT_MASS = (148 - 20.3) * 0.453592;                                                                    // 32lbs * kg per pound
        public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);        // Frame Size
    }
    public static final class swerve {
        public static final double          LOOP_TIME               = 0.13;                                                                 //ms, 20ms + 110ms sprk max velocity lag {Loop time to get the updated data for swerve system}
        public static final double          MAX_SPEED               = Units.feetToMeters(44.5);                                        //ft,  Maximum speed of the robot in feet per second conv. to meters per second, used to limit acceleration.
        public static final PIDConstants    AUTO_Translation_PID    = new PIDConstants(0.7, 0, 0);                                 //
        public static final PIDConstants    AUTO_Angle_PID          = new PIDConstants(0.4, 0, 0.01);                              //
        public static final double          WHEEL_LOCK_TIME         = 10;                                                                   //s, Time till wheels return to breaking mode and stop PID's after x seconds
        public static final double          wheelDiameter           = 4;                                                                    //in, Diameter of wheels in the swerve modules
        public static final double          gearRatio_Stearing      = 12.8;                                                                 // Ratio's upper number for swerve modules steering
        public static final class PathFinding {                                                                                             // Constraints placed on drive train during PathPlanner Autonomous Actions
            public static final double      velocity                = Units.feetToMeters(12.5);                                        //ft, Max Velocity Allowed
            public static final double      acceleration            = 4.0;                                                                  // Max Acceleration
            public static final double      angularVeloc            = Units.feetToMeters(10.2);                                        // Max Angular Velocity Allowed
            public static final double      angularAccel            = Units.degreesToRadians(720);                                  //deg, Max Acceleration Rate during angular G's
        }
    }
    public static final class operatorDriver {
        public static final int    port             = 0;                                                                                    // USB position assignment for DriverStation
        public static final double LEFT_X_DEADBAND  = 0.1;                                                                                  //%, Left Stick Horizontal Deadband Percentage of Travel from center
        public static final double LEFT_Y_DEADBAND  = 0.1;                                                                                  //%, Left Stick Vertical Deadband Percentage of Travel from center
        public static final double RIGHT_X_DEADBAND = 0.1;                                                                                  //%, Right Stick Horizontal Deadband Percentage of Travel from center
        public static final double RIGHT_Y_DEADBAND = 0.1;                                                                                  //%, Right Stick Vertical Deadband Percentage of Travel from center
        public static final double turn             = 6.0;                                                                                  // ???? Brough over from YAGSL Example (Unkown action)
    }
    public static final class operatorManip {
        public static final int    port             = 1;                                                                                    // USB position assignment for DriverStation
        public static final double LEFT_X_DEADBAND  = 0.1;                                                                                  //%, Left Stick Horizontal Deadband Percentage of Travel from center
        public static final double LEFT_Y_DEADBAND  = 0.1;                                                                                  //%, Left Stick Vertical Deadband Percentage of Travel from center
        public static final double RIGHT_X_DEADBAND = 0.1;                                                                                  //%, Right Stick Horizontal Deadband Percentage of Travel from center
        public static final double RIGHT_Y_DEADBAND = 0.1;                                                                                  //%, Right Stick Vertical Deadband Percentage of Travel from center
    }
    public static final class vision {

    }
    public static final class power {
        public static final int    port_core_RIO        = 20;                                                                               // Port on the Power Hub for the RIO Controller
        public static final int   fused_core_RIO        = 5;                                                                                //amps, Fused for
        public static final int    port_core_Radio      = 21;                                                                               // Port on the Power Hub for the Radio Module (Vivid Hosting or OpenMesh)
        public static final int   fused_core_Radio      = 15;                                                                               //amps, Fused for
        public static final int    port_core_LL_VRM     = 22;                                                                               // Port on the Power Hub for the Voltage Regulation Module dedicated for LimeLight Cameras
        public static final int   fused_core_LL_VRM     = 10;                                                                               //amps, Fused for
        public static final int    port_sw_FL_Stearing  = 0;                                                                                // Port on the Power Hub for the Front Left Swerve Stearing Motor
        public static final int   fused_sw_FL_Stearing  = 30;                                                                               //amps, Fused for
        public static final int    port_sw_FL_Drive     = 1;                                                                                // Port on the Power Hub for the Front Left Swerve Drive Motor
        public static final int   fused_sw_FL_Drive     = 40;                                                                               //amps, Fused for
        public static final int    port_sw_FR_Stearing  = 19;                                                                               // Port on the Power Hub for the Front Right Swerve Stearing Motor
        public static final int   fused_sw_FR_Stearing  = 30;                                                                               //amps, Fused for
        public static final int    port_sw_FR_Drive     = 18;                                                                               // Port on the Power Hub for the Front Right Swerve Drive Motor
        public static final int   fused_sw_FR_Drive     = 40;                                                                               //amps, Fused for
        public static final int    port_sw_F_Encoders   = 4;                                                                                // Port on the Power Hub for the Front two Absolute Encoders on the Swerve Modules
        public static final int   fused_sw_F_Encoders   = 10;                                                                               //amps, Fused for
        public static final int    port_sw_BL_Stearing  = 2;                                                                                // Port on the Power Hub for the Back Left Swerve Stearing Motor
        public static final int   fused_sw_BL_Stearing  = 30;                                                                               //amps, Fused for
        public static final int    port_sw_BL_Drive     = 3;                                                                                // Port on the Power Hub for the Back Left Swerve Drive Motor
        public static final int   fused_sw_BL_Drive     = 40;                                                                               //amps, Fused for
        public static final int    port_sw_BR_Stearing  = 17;                                                                               // Port on the Power Hub for the Back Right Swerve Stearing Motor
        public static final int   fused_sw_BR_Stearing  = 30;                                                                               //amps, Fused for
        public static final int    port_sw_BR_Drive     = 16;                                                                               // Port on the Power Hub for the Back Right Swerve Drive Motor
        public static final int   fused_sw_BR_Drive     = 40;                                                                               //amps, Fused for
        public static final int    port_sw_B_Encoders   = 15;                                                                               // Port on the Power Hub for the Back two Absolute Encoders on the Swerve Modules
        public static final int   fused_sw_B_Encoders   = 10;                                                                               //amps, Fused for
        public static final int    port_sw_IMU          = 23;                                                                               // Port on the Power Hub for the CTRE Pigeon2 IMU
        public static final int   fused_sw_IMU          = 5;                                                                                //amps, Fused for
        public static final class managment{
            public static final double  sw_vtDrop_tol   = 0.095;                                                                            // Swerve Drive Percent Voltage Drop Tolerated Before Alert
            public static final double  swD_SurgeAmp    = 50;                                                                               //amps, 
            public static final double  swD_CritAmp     = 75;                                                                               //amps, 
            public static final int     swD_BreakAmp    = 60;                                                                               //amps, 
            public static final double  swD_BreakTime   = 1000;                                                                             //ms, 
        } 
    }
    public static final class CANnet {                                                                                                      // Can Devices and their associated ID assignments
        public static final int         core_RIO        = 0;
        public static final int         core_PowerHub   = 1;                                                                                // RevRobotics PowerHub
        public static final int         core_IMU        = 2;                                                                               // CrossTheRoadElectronics Pigeon 2.0
        public static final class swerve {
            public static final int     FL_Stearing     = 11;                                                                               // RevRobotics Vortex Motor w/ Flex Controller
            public static final int     FL_Drive        = 12;                                                                               // RevRobotics Vortex Motor w/ Flex Controller
            public static final int     FL_Absolute     = 13;                                                                               // CrossTheRoadElectronics CanCoder
            public static final int     FR_Stearing     = 21;                                                                               // RevRobotics Vortex Motor w/ Flex Controller
            public static final int     FR_Drive        = 22;                                                                               // RevRobotics Vortex Motor w/ Flex Controller
            public static final int     FR_Absolute     = 23;                                                                               // CrossTheRoadElectronics CanCoder
            public static final int     BL_Stearing     = 31;                                                                               // RevRobotics Vortex Motor w/ Flex Controller
            public static final int     BL_Drive        = 32;                                                                               // RevRobotics Vortex Motor w/ Flex Controller
            public static final int     BL_Absolute     = 33;                                                                               // CrossTheRoadElectronics CanCoder
            public static final int     BR_Stearing     = 41;                                                                               // RevRobotics Vortex Motor w/ Flex Controller
            public static final int     BR_Drive        = 42;                                                                               // RevRobotics Vortex Motor w/ Flex Controller
            public static final int     BR_Absolute     = 43;                                                                               // CrossTheRoadElectronics CanCoder
        }
        public static final class elevator {
            public static final int     Lift_Master     = 15;                                                                               // RevRobotics NEO 750 w/ SparkMax Controller and throughbore encoder
            public static final int     Lift_Follower   = 16;                                                                               // RevRobotics NEO 750 w/ SparkMax Controller
        }
        public static final class manipulators {
            public static final int     Coral_Feed      = 35;                                                                               // RevRobotics NEO 750 w/ SparkMax Controller
            public static final int     Coral_Tilt      = 36;                                                                               // RevRobotics NEO 750 w/ SparkMax Controller
            public static final int     Algae_kicker    = 37;                                                                               // RevRobotics NEO 750 w/ SparkMax Controller
            public static final int     Algae_Left      = 38;                                                                               // RevRobotics NEO 550 w/ SparkMax Controller
            public static final int     Algae_Right     = 39;                                                                               // RevRobotics NEO 550 w/ SparkMax Controller
        }
        public static final class climber {
            public static final int     Paw             = 25;                                                                               // RevRobotics Vortex Motor w/ Flex Controller
        }
    }
    public static final class sensor {
        public static final int         elevator0       = 0;                                                                                // Reed Switch Sensor on 3rd Stage (on) when at bottom
        public static final int         elevatorCar0    = 1;                                                                                // Optical Sensor on Car (on) when at bottom
        public static final int         coralLoaded     = 2;                                                                                // Optical to catch when coral is completely loaded in manipulator
        public static final int         coralEmpty      = 3;                                                                                // Optical to catch when coral is completely unloaded in manipulator
        public static final int         climbCageIn     = 4;                                                                                // Optical Sensor to know when cage has settled into climber
        public static final int         climbCageDown   = 5;                                                                                // 
    }
    public static final class config {
        public static final class elevator {
            public static final int         stallAmp        = 50;                                                                           // Stall Amps
            public static final int         altReltEncoder  = 8192;                                                                         // Encoder Steps per Revolution for Throughbore Encoder
            public static final double      ratioGearBox    = 6;                                                                            // Single Stage gear box with 6:1 Ratio by RevRobotics
            public static final double      climbRatio      = 0;                                                                            // Conversion Factor
            public static final double      PIDF_P          = 0.001;                                                                        // Proportional
            public static final double      PIDF_I          = 0.000;                                                                        // Integral Factor
            public static final double      PIDF_D          = 0.000;                                                                        // Derivative
            public static final double      velocityFF      = 0.000;                                                                        // Feed-Forward
            public static final double      velocityMax     = 6000.0;                                                                       // Max Velocity Allowed
            public static final double      accelerationMax = 0.000;                                                                        // Max Acceleration Allowed
        }
        public static final class climber {

        }
    }          
}
