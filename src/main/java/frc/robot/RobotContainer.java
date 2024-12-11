// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.auto.PathFindToLowZone;
import frc.robot.commands.swervedrive.auto.PathFindToTote;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.PIDToVisionPose;
import frc.robot.commands.swervedrive.drivebase.PathFindToTag;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final CommandPS5Controller driverController = new CommandPS5Controller(0);
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/neo"));

    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        registerNamedCommands();

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the rotational velocity
        // buttons are quick rotation positions to different ways to face
        // WARNING: default buttons are on the same buttons as the ones defined in
        // configureBinding
        final AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                () -> -MathUtil.applyDeadband(driverController.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> -MathUtil.applyDeadband(driverController.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> -MathUtil.applyDeadband(driverController.getRightX(),
                        OperatorConstants.RIGHT_X_DEADBAND),

                driverController.getHID()::getTriangleButtonPressed,
                driverController.getHID()::getCrossButtonPressed,
                driverController.getHID()::getCircleButtonPressed,
                driverController.getHID()::getSquareButtonPressed);

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation
        final Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
                () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> driverController.getRightX(),
                () -> driverController.getRightY());

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot
        final Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(
                () -> MathUtil.applyDeadband(-driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(-driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> driverController.getRightX() * -0.5);

        final Command driveFieldOrientedAngularVelocitySim = drivebase.simDriveCommand(
                () -> MathUtil.applyDeadband(-driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(-driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> driverController.getRightX() * -0.5);

        drivebase.setDefaultCommand(
                RobotBase.isSimulation() ? driveFieldOrientedAngularVelocitySim : driveFieldOrientedAngularVelocity);

        //auto config
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.addOption("Do nothing", Commands.none());
        autoChooser.addOption("Low Zone Left", new PathPlannerAuto("Low Zone Left"));
        autoChooser.addOption("Low Zone Right", new PathPlannerAuto("Low Zone Right"));
        autoChooser.addOption("Low Zone Middle", new PathPlannerAuto("Low Zone Middle"));
        autoChooser.addOption("Path Find to Close Tote", new PathPlannerAuto("Middle Tote Left"));
        autoChooser.addOption("Path Find to Middle Tote", new PathPlannerAuto("Middle Tote Left"));
        autoChooser.addOption("Path Find to Close Far", new PathPlannerAuto("Middle Tote Left"));
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("LEFT: Path Find to Low Zone", new PathFindToLowZone(drivebase, PathFindToLowZone.Position.LEFT));
        NamedCommands.registerCommand("MIDDLE: Path Find to Low Zone", new PathFindToLowZone(drivebase, PathFindToLowZone.Position.MIDDLE));
        NamedCommands.registerCommand("RIGHT: Path Find to Low Zone", new PathFindToLowZone(drivebase, PathFindToLowZone.Position.RIGHT));

        NamedCommands.registerCommand("Path Find to Close Tote", new PathFindToTote(drivebase, PathFindToTote.Position.CLOSE));
        NamedCommands.registerCommand("Path Find to Middle Tote", Commands.deferredProxy(() -> new PathFindToTote(drivebase, PathFindToTote.Position.MIDDLE)));
        NamedCommands.registerCommand("Path Find to Far Tote", new PathFindToTote(drivebase, PathFindToTote.Position.FAR));


    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the
     * named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for
     * {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        //
        driverController.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        // driverController.circle().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
//        driverController.square().whileTrue(
//                 drivebase.driveToPose(
//                         new Pose2d(new Translation2d(4, 7), Rotation2d.fromDegrees(40))));

        // driverController.triangle().whileTrue(
        // Commands.deferredProxy(() -> {
        // if (drivebase.getBestTagTransform().isPresent()) {
        // Transform2d tagTransform = drivebase.getBestTagTransform().get();
        //
        // return drivebase.driveToPose(drivebase.getPose().plus(new
        // Transform2d(tagTransform.getTranslation(),
        // tagTransform.getRotation().plus(Rotation2d.fromDegrees(180)))));
        // }
        // return null;
        // })
        // );
        //
        // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock,
        // drivebase).repeatedly());

        // driverController.square().whileTrue(PathFindToTag.pathFindToTag(drivebase));
        driverController.triangle().whileTrue(new PIDToVisionPose(drivebase, drivebase::getToteDestinationPose));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return drivebase.getAutonomousCommand("Drive Backward");
        return autoChooser.getSelected();
    }

    public void setDriveMode() {
        // drivebase.setDefaultCommand();
    }

    public void setMotorBrake(final boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
