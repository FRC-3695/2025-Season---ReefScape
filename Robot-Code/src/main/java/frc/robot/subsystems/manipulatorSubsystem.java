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
    private final SparkFlex climber_Paw = new SparkFlex(Constants.CANnet.climber.Paw, MotorType.kBrushless);

    public void periodic() {
        double speed = operatorDriver.getLeftY(); 
        climber_Paw.set(speed);
    }
}
