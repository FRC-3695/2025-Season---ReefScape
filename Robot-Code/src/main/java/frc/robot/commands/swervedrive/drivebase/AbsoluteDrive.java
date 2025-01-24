package frc.robot.commands.swervedrive.drivebase;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.swerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class AbsoluteDrive extends Command {                                                                                            //- Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
    private final swerveSubsystem swerve;                                                                                               // {@Param - swerve} The swerve drivebase subsystem.
    private final DoubleSupplier vX, vY;                                                                                                // {@Param - vX} DoubleSupplier that supplies the x-translation joystick input. Should be in the range -1 to 1 with deadband already accounted for.  Positive X is away from the alliance wall. {@Param - vX} DoubleSupplier that supplies the y-translation joystick input. Should be in the range -1 to 1 with deadband already accounted for. Positive Y is towards the left wall when looking through the driver station glass.
    private final DoubleSupplier headingHorizontal, headingVertical;                                                                    // {@Param - headingHorizontal} DoubleSupplier that supplies the horizontal component of the robot's heading angle. In the robot coordinate system, this is along the same axis as vY. Should range from -1 to 1 with no deadband. Positive is towards the left wall when looking through the driver station glass. {@Param - headingVertical} DoubleSupplier that supplies the vertical component of the robot's heading angle. In the robot coordinate system, this is along the same axis as vX. Should range from -1 to 1 with no deadband. Positive is away from the alliance wall.
    private boolean initRotation = false;                                                                                               // {@Param - initRotation} 
    
    public AbsoluteDrive(swerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingHorizontal, DoubleSupplier headingVertical) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingHorizontal = headingHorizontal;
        this.headingVertical = headingVertical;
        addRequirements(swerve);
    }
    @Override
    public void initialize() {                                                                                                          //-
        initRotation = true;
    }
    @Override
    public void execute() {                                                                                                             //- Called every time the scheduler runs while the command is scheduled.
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(                                                                           // Get the desired chassis speeds based on a 2 joystick module.
            vX.getAsDouble(),
            vY.getAsDouble(),
            headingHorizontal.getAsDouble(),
            headingVertical.getAsDouble()
        );
        if (initRotation) {                                                                                                             // Prevent Movement After Auto
            if (headingHorizontal.getAsDouble() == 0 && headingVertical.getAsDouble() == 0) {
                Rotation2d firstLoopHeading = swerve.getHeading();                                                                      // Get the curretHeading
                desiredSpeeds = swerve.getTargetSpeeds(                                                                                 // Set the Current Heading to the desired Heading
                    0,
                    0,
                    firstLoopHeading.getSin(),
                    firstLoopHeading.getCos()
                );
            }
            initRotation = false;                                                                                                       // Dont Init Rotation Again
        }
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);                                                   // Limit velocity to prevent tippieness
        translation = SwerveMath.limitVelocity(
            translation,
            swerve.getFieldVelocity(),
            swerve.getPose(),
            Constants.swerve.LOOP_TIME,
            Constants.robot.ROBOT_MASS,
            List.of(Constants.robot.CHASSIS),
            swerve.getSwerveDriveConfiguration()
        );
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());
        swerve.drive(                                                                                                                   // Make the robot move
            translation,
            desiredSpeeds.omegaRadiansPerSecond,
            true
        );
    }
    @Override
    public void end(boolean interrupted) {                                                                                              //- Called once the command ends or is interrupted.

    }
    public boolean isFinished() {                                                                                                       //- Returns true when the command should end.
        return false;
    }
}
