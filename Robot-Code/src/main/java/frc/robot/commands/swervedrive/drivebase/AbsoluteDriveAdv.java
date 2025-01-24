package frc.robot.commands.swervedrive.drivebase;

import java.util.List;
import java.util.function.BooleanSupplier;
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

public class AbsoluteDriveAdv extends Command {
    private final swerveSubsystem swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier headingAdjust;
    private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight;
    private boolean resetHeading = false;

    public AbsoluteDriveAdv(swerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust, BooleanSupplier lookAway, BooleanSupplier lookTowards, BooleanSupplier lookLeft, BooleanSupplier lookRight) {
        this.swerve = swerve;                                                                                                           // {@param - swerve}        The swerve drivebase subsystem.
        this.vX = vX;                                                                                                                   // {@param - vX}            DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1 with deadband already accounted for. Positive X is away from the alliance wall.
        this.vY = vY;                                                                                                                   // {@param - vY}            DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1 with deadband already accounted for. Positive Y is towards the left wall when looking through the driver station glass.
        this.headingAdjust = headingAdjust;                                                                                             // {@param - headingAdjust} DoubleSupplier that supplies the component of the robot's heading angle that should be adjusted. Should range from -1 to 1 with deadband already accounted for.
        this.lookAway = lookAway;                                                                                                       // {@param - lookAway}      Face the robot towards the opposing alliance's wall in the same direction the driver is facing
        this.lookTowards = lookTowards;                                                                                                 // {@param - lookTowards}   Face the robot towards the driver
        this.lookLeft = lookLeft;                                                                                                       // {@param - lookLeft}      Face the robot left
        this.lookRight = lookRight;                                                                                                     // {@param - lookRight}     Face the robot right
        addRequirements(swerve);                                                                                                        
    }
    @Override
    public void initialize() {
        resetHeading = true;
    }
    @Override
    public void execute() {                                                                                                             // Called every time the scheduler runs while the command is scheduled.
        double headingX = 0;
        double headingY = 0;
                                                                                                                                        // These are written to allow combinations for 45 angles
        if (lookAway.getAsBoolean()) {                                                                                                  // Face Away from Drivers
            headingY = -1;
        }
        if (lookRight.getAsBoolean()) {                                                                                                 // Face Right
            headingX = 1;
        }
        if (lookLeft.getAsBoolean()) {                                                                                                  // Face Left
            headingX = -1;
        }
        if (lookTowards.getAsBoolean()) {                                                                                               // Face Towards the Drivers
            headingY = 1;
        }
        if (resetHeading) {                                                                                                             // Prevent Movement After Auto
            if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) == 0) {
                Rotation2d currentHeading = swerve.getHeading();                                                                        // Get the curret Heading
                headingX = currentHeading.getSin();                                                                                     // Set the Current Heading to the desired Heading
                headingY = currentHeading.getCos();                                                                                     // Set the Current Heading to the desired Heading
            }
            resetHeading = false;                                                                                                       // Dont reset Heading Again
        }
        ChassisSpeeds desirSpeeds = swerve.getTargetSpeeds(                                                                             // 
            vX.getAsDouble(),
            vY.getAsDouble(),
            headingX,
            headingY
        );
        Translation2d translation = SwerveController.getTranslation2d(desirSpeeds);                                                     // Limit velocity to prevent tippieness
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
        if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0) {                                              // Make the robot move                                                                                         //
            resetHeading = true;
            swerve.drive(
                translation,
                (Constants.operatorDriver.turn * -headingAdjust.getAsDouble()),
                true
            );
        } else {
            swerve.drive(
                translation,
                desirSpeeds.omegaRadiansPerSecond,
                true
            );
        }
    }
    @Override
    public void end(boolean interrupted) {                                                                                              // Called once the command ends or is interrupted.

    }
    @Override
    public boolean isFinished() {                                                                                                       // Returns true when the command should end.
        return false;
    }
}