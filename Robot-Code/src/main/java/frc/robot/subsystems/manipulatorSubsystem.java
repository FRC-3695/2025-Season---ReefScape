package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;




public class manipulatorSubsystem extends SubsystemBase {


    static CommandXboxController operatorDriver = new CommandXboxController(Constants.operatorDriver.port);
    public static final CANSparkMax Coral_Feed = new CANSparkMax(Constants.CANnet.manipulators.Coral_Feed, MatorType.kbrushless);


    public void periodic() {
        double speed = operatorDriver.getLeftY(); 
        Coral_Feed.set(speed);
    }
}
