package frc.robot.tools;
import java.text.DecimalFormat;

import com.acidmanic.consoletools.terminal.Terminal;

//WPILib Libraries
import edu.wpi.first.hal.HALUtil;
import frc.robot.BuildConstants;

public class utils {
    private static double lastTripRisk;
    private static Terminal Console = new Terminal();
    private utils() {}
    // ------------------------------------------------------------------------------------------    Math    ------------------------------------------------------------------------------------------
    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    // ----------------------------------------------------------------------------------------    Logging    -----------------------------------------------------------------------------------------
    static DecimalFormat ft = new DecimalFormat("###0000.000"); 
    public static void Logging(int level, String event) {
        System.out.print("*** ");
        double time = HALUtil.getFPGATime();
        time = time/1000000;
        System.out.print(ft.format(time));
        switch (level) {
            case 0:
                Console.setScreenAttributes(Terminal.Constants.FOREGROUND_MAGENTA, Terminal.Constants.BACKGROUND_BLACK);
                System.out.print(" - Debug - ");
                break;
            case 1:
                Console.setScreenAttributes(Terminal.Constants.FOREGROUND_BLUE, Terminal.Constants.BACKGROUND_BLACK);
                System.out.print(" - Info - ");
                break;
            case 2:
                Console.setScreenAttributes(Terminal.Constants.FOREGROUND_MAGENTA, Terminal.Constants.BACKGROUND_BLACK);
                System.out.print(" - Error - ");
                break;
            case 3:
                Console.setScreenAttributes(Terminal.Constants.FOREGROUND_YELLOW, Terminal.Constants.BACKGROUND_BLACK);
                System.out.print(" - ! Alert ! - ");
                break;
            case 4:
                Console.setScreenAttributes(Terminal.Constants.FOREGROUND_RED, Terminal.Constants.BACKGROUND_BLACK);
                System.out.print(" - (/) Warning (/) - ");
                break;
            case 5:
                Console.setScreenAttributes(Terminal.Constants.FOREGROUND_BLACK, Terminal.Constants.BACKGROUND_RED);
                System.out.print(" - </> Critical </> - ");
                break;
            default:
                break;
        }
        Console.setScreenAttributes(Terminal.Constants.FOREGROUND_MAGENTA, Terminal.Constants.BACKGROUND_BLACK);
        System.out.println(event);
    }
    // ----------------------------------------------------------------------------------------    GitData    -----------------------------------------------------------------------------------------
    public static void GitInfo() {
        Console.setScreenAttributes(Terminal.Constants.FOREGROUND_MAGENTA, Terminal.Constants.BACKGROUND_BLACK);
        System.out.println("|");
        System.out.println("****************************************************");
        System.out.println("Build Branch: "+ BuildConstants.MAVEN_NAME);
        System.out.println("GIT Revision: "+ BuildConstants.GIT_REVISION);
        System.out.println("Built on: "+ BuildConstants.BUILD_DATE + " @ " + BuildConstants.BUILD_UNIX_TIME);
        System.out.println("GIT_SHA: "+ BuildConstants.GIT_SHA);
        if(BuildConstants.DIRTY != 0) {       
            Console.setScreenAttributes(Terminal.Constants.FOREGROUND_RED, Terminal.Constants.BACKGROUND_BLACK);                                                                                                                                    // Warning of uncommited changes in deployed build
            System.out.println("|\n****************************************************\n****************************************************\n    ********** Fruit of the Poisonous Tree *************");
        }
        Console.setScreenAttributes(Terminal.Constants.FOREGROUND_MAGENTA, Terminal.Constants.BACKGROUND_BLACK);
        System.out.println("****************************************************");
        System.out.println("|");
    }
    // ---------------------------------------------------------------------------------    RoboRIO Identification    ---------------------------------------------------------------------------------
    public static String RoboRIOid() {
        switch (System.getenv("serialnum")) {
            case "2":
                return "Comp_Bot-2022";
            case "4":
                return "Comp_Bot-2023";
            case "0":
                return "Dev_Bot-2024";
            case "1":
                return "Comp_Bot-2024";
            case "32238C5":
                return "Caroline_Rogue-One";
            case "3":
                return "Caroline_Bench";
            default:
                Logging(4, "RoboRIO Serial Number Not in Utils Listing : " + System.getenv("serialnum"));
                return null;
        }
    }
    // -------------------------------------------------------------------------------------    RoboRIO Serial    -------------------------------------------------------------------------------------
    public static String RoboRIOSerial() {
        return System.getenv("serialnum");
    }
    // -------------------------------------------------------------------------------    Power Reporting Functions    --------------------------------------------------------------------------------
    public static final class Power {
        public static void voltageDrop(String name, double supplyVolt, double deviceVolt, double percentLossCritical, double percentLossAlert) {                            // Check the voltage drop betweek given supply and device and using tolerance to identify bad connections
            String logReport = "";
            if (deviceVolt/supplyVolt+percentLossCritical < 1) {
                Logging(5, logReport);
            } else if (deviceVolt/supplyVolt+percentLossAlert < 1) {
                Logging(3, logReport);
            } else {
                Logging(0, logReport);
            }
        }
        public static void voltageDrop(String name, double supplyVolt, double deviceVolt, double percentLossCritical) {                                                     // Check the voltage drop betweek given supply and device and using tolerance to identify bad connections
            voltageDrop(name, supplyVolt, deviceVolt, percentLossCritical, 0.5);
        }
        public static void amperageCheck(String name, double deviceAmpDraw, double surgeAmps, double criticalAmps, double breakerTrip, double breakerTimeMS) {              // 60amp 1s for 40am
            if (deviceAmpDraw > criticalAmps) {                                                                                                                             // 
                Logging(4, name + "is Exceeding Critical Amp(s) \"" + criticalAmps + "\"" + " by drawing : " + deviceAmpDraw + "Amp(s)");
            } else if (deviceAmpDraw > surgeAmps) {                                                                                                                         //
                Logging(3, name + "is Exceeding Max Surge Amp(s) \"" + surgeAmps + "\"" + " by drawing : " + deviceAmpDraw + "Amp(s)");
            } else {                                                                                                                                                        //
                Logging(0, name + " current draw is (" + deviceAmpDraw + ")Amp(s)");
            }
            if (deviceAmpDraw > breakerTrip && lastTripRisk >= breakerTimeMS) {                                                                                             //
                Logging(4, name + " has reached breaker trip limit of " + breakerTrip + " Amp(s) over " + breakerTimeMS + "ms");
            } else if (lastTripRisk > 150) {
                lastTripRisk = System.currentTimeMillis();
            }
        }
    }
}