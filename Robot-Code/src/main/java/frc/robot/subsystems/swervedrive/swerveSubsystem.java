package frc.robot.subsystems.swervedrive;

import frc.robot.Constants;
import frc.robot.tools.utils;

import java.io.File;
import java.io.IOException;
import java.text.ParseException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.SwerveModule;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class swerveSubsystem extends SubsystemBase {
    private final SwerveDrive                           swDrive;                                                                        // Swerve Drive Declared

    public swerveSubsystem (File directory) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;                                                                       // Sets Verbosity of SwerveDrive System on Network Table (Important for Network Speeds)
        try {
            swDrive = new SwerveParser(directory).createSwerveDrive(
                Constants.swerve.MAX_SPEED,
                new Pose2d(new Translation2d(
                        Meter.of(1),
                        Meter.of(4)
                    ),
                    Rotation2d.fromDegrees(0)
                )
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swDrive.setHeadingCorrection(false);                                                                                      // To be set to true when controlling the swerve drive via angle
        swDrive.setAngularVelocityCompensation(true, true, 0.1);                             // Corrects for skew that occurs when angular velocity increases on drivetrain
        swDrive.setModuleEncoderAutoSynchronize(false, 1);                                                             // This enable the synchronization of motor encoder and absolute encode when drive train comes to a stop for a period
        setupPathPlanner();                                                                                                             // Calls on function echoed below to setup pathplanner requirements for drivetrain
    }
    public swerveSubsystem (SwerveDriveConfiguration driveCFG, SwerveControllerConfiguration contrlCFG) {
        swDrive = new SwerveDrive(
            driveCFG, 
            contrlCFG, 
            Constants.swerve.MAX_SPEED, 
            new Pose2d(
                new Translation2d(
                    Meter.of(2),
                    Meter.of(0)
                ),
                Rotation2d.fromDegrees(0)
            )
        );
    }
    // -----------------------------------------------------------------------------------    Periodic(s)    ------------------------------------------------------------------------------------
    @Override
    public void periodic () {
        
    }
    // ----------------------------------------------------------------------------------    Module Control    ----------------------------------------------------------------------------------
    public Command modulesCenter() {                                                                                                    // Command Serves to Zero Wheels to Home/ Known 0
        return run(() -> {
            Arrays.asList(swDrive.getModules()).forEach(it -> it.setAngle(0.0));                                                  // Iterates through each module as array running each module to zero
        });
    }
    public void lock() {                                                                                                                // Lock the swerve drive to prevent it from moving.
        swDrive.lockPose();
    }
    // -----------------------------------------------------------------------------------    PathPlanner    ------------------------------------------------------------------------------------
    public void setupPathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            final boolean enableFeedForward = true;

            AutoBuilder.configure(
                this::getPose, 
                this::resetOdometry, 
                this::getRobotVelocity, 
                (speedsRobotRelative, moduleFeedForwards) -> {
                    if (enableFeedForward) {
                        swDrive.drive(
                            speedsRobotRelative,
                            swDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                            moduleFeedForwards.linearForces()
                        );
                    } else {
                        swDrive.setChassisSpeeds(speedsRobotRelative);
                    }
                }, 
                new PPHolonomicDriveController(                                                                                         // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0),                                                                           // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0)                                                                            // Rotation PID constants
                ), 
                config, 
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
            );

        } catch (Exception e) {
            e.printStackTrace();
        }
        PathfindingCommand.warmupCommand().schedule();                                                                                  // Preload PathPlanner Path finding
    }
    public Command driveToDistance(double distanceToCover, double speedSet) {                                                           // Returns a Command that drives the swerve drive to a specific distance at a given speed.
        return run(() -> drive(new ChassisSpeeds(Units.feetToMeters(speedSet), 0, 0)))
            .until(() -> swDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) > distanceToCover);
    }
    public void modulesBraking (boolean brake) {                                                                                        // Public Function to enable and disable braking
        swDrive.setMotorIdleMode(brake);
    }
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotX) {                 // Command to drive the robot using translative values and heading as angular velocity.
        return run(() -> {
            swDrive.drive(
                SwerveMath.scaleTranslation(
                    new Translation2d(
                        translationX.getAsDouble() * swDrive.getMaximumChassisVelocity(),
                        translationY.getAsDouble() * swDrive.getMaximumChassisVelocity()
                    ), 
                0.8
                ),
                Math.pow(angularRotX.getAsDouble(), 3) * swDrive.getMaximumChassisAngularVelocity(),
                true,
                false);
        });
    }
    public Command getAutonomousCommand(String pathName) {
        return new PathPlannerAuto(pathName);
    }
    public Command driveToPose(Pose2d pose) {                                                                                           // Use of PathPlanner path planning tool to navigate to a point on the field
        PathConstraints constraints = new PathConstraints(
            Constants.swerve.PathFinding.velocity,
            Constants.swerve.PathFinding.acceleration,
            Constants.swerve.PathFinding.angularVeloc,
            Constants.swerve.PathFinding.angularAccel
        );
        return AutoBuilder.pathfindToPose(
            pose, 
            constraints,
            MetersPerSecond.of(0)
        );
    }
    public Command sysIdDriveMotorCommand() {                                                                                           // Command to characterize the robot drive motors using SysId
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(
                new Config(),                                                                                                           // Blank config for testing
                this,
                swDrive,                                                                                                                // Swerve Drive Decleration
                12,
                true                                                                                                                    // Test with Spinning (true) or test with driving in straight line (false)                                                                                                                      // Max Battery Voltage
            ),
            3.0,                                                                                                                        // delay
            5.0,                                                                                                                        // quasi Timeout
            3.0                                                                                                                         // Dynamic Timeout
        );
    }
    public Command sysIdAngleMotorCommand() {                                                                                           // Command to characterize the robot angle motors using SysId
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setAngleSysIdRoutine(
                new Config(),
                this,
                swDrive
            ), 
            3.0,                                                                                                                        // delay
            5.0,                                                                                                                        // quasi Timeout
            3.0                                                                                                                         // Dynamic Timeout
        );
    }
    private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)                                       // Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
    throws IOException, ParseException, org.json.simple.parser.ParseException {
        SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
            RobotConfig.fromGUISettings(), 
            swDrive.getMaximumChassisAngularVelocity()
        );
        AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(new SwerveSetpoint(
            swDrive.getRobotVelocity(),
            swDrive.getStates(),
            DriveFeedforwards.zeros(
                swDrive.getModules().length
            )
        ));
        AtomicReference<Double> previousTime = new AtomicReference<>();
        return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
            () -> {
                double newTime = Timer.getFPGATimestamp();
                SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(
                    prevSetpoint.get(),
                    robotRelativeChassisSpeed.get(),
                    newTime - previousTime.get()
                );
                swDrive.drive(
                    newSetpoint.robotRelativeSpeeds(),
                    newSetpoint.moduleStates(),
                    newSetpoint.feedforwards().linearForces()
                );
                prevSetpoint.set(newSetpoint);
                previousTime.set(newTime);
            }
        );
    }
    public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {                               // Drive with 254's Setpoint generator; port written by PathPlanner.
        try {
            return driveWithSetpointGenerator(() -> {
                return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());
            });
        } catch (Exception e) {
            DriverStation.reportError(e.toString(), true);
        }

        return Commands.none();
    }
    // ------------------------------------------------------------------------------------    Functions    -------------------------------------------------------------------------------------
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swDrive.drive(
            translation,
            rotation,
            fieldRelative,
            false
        );
    }
    public void drive(ChassisSpeeds velocity) {                                                                                         // Drive according to the chassis robot oriented velocity.
        swDrive.drive(velocity);
    }
    public SwerveDriveKinematics getKinematics() {                                                                                      // Get the swerve drive kinematics object.
        return swDrive.kinematics;
    }
    public void driveFieldOriented(ChassisSpeeds velocity) {                                                                            // Drive the robot given a chassis field oriented velocity.
        swDrive.driveFieldOriented(velocity);
    }
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {                                                               // Drive the robot given a chassis field oriented velocity.
        return run(() -> {
            swDrive.driveFieldOriented(velocity.get());
        });
    }
    public void resetOdometry(Pose2d initialHolonomicPose) {                                                                            // Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this method. However, if either gyro angle or module position is reset, this must be called in order for odometry to keep working.
        swDrive.resetOdometry(initialHolonomicPose);
    }
    public Pose2d getPose() {                                                                                                           // Gets the current pose (position and rotation) of the robot, as reported by odometry.
        return swDrive.getPose();
    }
    public void setChassisSpeeds(ChassisSpeeds chassisSpeed) {                                                                          // Set chassis speeds with closed-loop velocity control.
        swDrive.setChassisSpeeds(chassisSpeed);
    }
    public void postTrajectory(Trajectory trajectory) {                                                                                 // Post the trajectory to the field.
        swDrive.postTrajectory(trajectory);
    }
    public void zeroGyro() {                                                                                                            // Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
        swDrive.zeroGyro();
        utils.Logging(1, "Gyro Zero'd");
    }
    private boolean isRedAlliance() {                                                                                                   // Checks if the alliance is red, defaults to false if alliance isn't available.
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }
    public void zeroGyroWithAlliance() {                                                                                                // This will zero (calibrate) the robot to assume the current position is facing forward, and if red alliance rotate the robot 180 after the drviebase zero command
        if (isRedAlliance()) {                                                                                                          // Check if Alliance is Red (true) -> 
            zeroGyro();                                                                                                                 // Zero Gyro
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));                                 // Rotates Pose 180 degrees in system
        } else {                                                                                                                        // if Alliance is Blue (false) ->
            zeroGyro();                                                                                                                 // Zero Gyro
        }
    }
    public Rotation2d getHeading() {                                                                                                    // Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase. Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
        return getPose().getRotation();
    }
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double xHeading, double yHeading) {                              // Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for the angle of the robot.
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(
            xInput,                                                                                                                     // @param xInput X joystick input for the robot to move in the X direction.
            yInput                                                                                                                      // @param yInput Y joystick input for the robot to move in the Y direction.
        ));
        return swDrive.swerveController.getTargetSpeeds(
            scaledInputs.getX(),
            scaledInputs.getY(),
            xHeading,                                                                                                                   // @param headingX X joystick which controls the angle of the robot.
            yHeading,                                                                                                                   // @param headingY Y joystick which controls the angle of the robot.
            getHeading().getRadians(),
            Constants.swerve.MAX_SPEED
        );
    }
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {                                              // Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of 90deg.
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(
            xInput,                                                                                                                     // @param xInput X joystick input for the robot to move in the X direction.
            yInput                                                                                                                      // @param yInput Y joystick input for the robot to move in the Y direction.
        ));
        return swDrive.swerveController.getTargetSpeeds(
            scaledInputs.getX(), 
            scaledInputs.getY(), 
            angle.getRadians(),                                                                                                         // @param angle  The angle in as a {@link Rotation2d}.
            getHeading().getRadians(), 
            Constants.swerve.MAX_SPEED
        );
    }
    public ChassisSpeeds getFieldVelocity() {                                                                                           // Gets the current field-relative velocity (x, y and omega) of the robot
        return swDrive.getFieldVelocity();
    }
    public ChassisSpeeds getRobotVelocity() {                                                                                           // Gets the current velocity (x, y and omega) of the robot
        return swDrive.getRobotVelocity();
    }
    public SwerveController getSwerveController() {                                                                                     // Get the {@link SwerveController} in the swerve drive.
        return swDrive.swerveController;
    }
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {                                                                     // Get the {@link SwerveDriveConfiguration} object.
        return swDrive.swerveDriveConfiguration;
    }
    public Rotation2d getPitch() {                                                                                                      // Gets the current pitch angle of the robot, as reported by the imu.
        return swDrive.getPitch();
    }
    public SwerveDrive getSwerveDrive() {                                                                                               // Gets the swerve drive object.
        return swDrive;
    }
     public double SwerveModVoltage_Drive(int module) {                                                                                 // Pulls Voltage from Drive Motor on Select Swerve Module
        SwerveModule[] modules = swDrive.getModules();                                                                                  // > Looks up module data and stores it to {@param - module}
        SwerveMotor motor = modules[module].getDriveMotor();                                                                            // > Gets Drive motor from {@param - module} storing SwerveMotor as {@param - motor}
        return motor.getVoltage();                                                                                                      // > {@param - motor} is pulled to get voltage
    }
    public double SwerveModVoltage_Steer(int module) {                                                                                  // Pulls Voltage from Angle Motor on Select Swerve Module
        SwerveModule[] modules = swDrive.getModules();                                                                                  // > Looks up module data and stores it to {@param - module}
        SwerveMotor motor = modules[module].getAngleMotor();                                                                            // > Gets Steering motor from {@param - module} storing SwerveMotor as {@param - motor}
        return motor.getVoltage();                                                                                                      // > {@param - motor} is pulled to get voltage
    }
}
