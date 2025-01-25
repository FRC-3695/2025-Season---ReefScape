package frc.robot.tools;
import java.text.DecimalFormat;

//WPILib Libraries
import edu.wpi.first.hal.HALUtil;

public class utils {
    private utils() {}
    // ------------------------------------------------------------------------------------------    Math    ------------------------------------------------------------------------------------------
    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    static DecimalFormat ft = new DecimalFormat("###0000.000"); 
    public static void Logging(int level, String event) {
        System.out.print("*** ");
        double time = HALUtil.getFPGATime();
        time = time/1000000;
        System.out.print(ft.format(time));
        switch (level) {
            case 0:
                System.out.print(" - Debug - ");
                break;
            case 1:
                System.out.print(" - Info - ");
                break;
            case 2:
                System.out.print(" - Error - ");
                break;
            case 3:
                System.out.print(" - ! Alert ! - ");
                break;
            case 4:
                System.out.print(" - (/) Warning (/) - ");
                break;
            case 5:
                System.out.print(" - </> Critical </> - ");
                break;
            default:

                break;
        }
        System.out.println(event);
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
}