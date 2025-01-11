// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommandDouble;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();


    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystickDriver = new CommandXboxController(0);
    private final CommandXboxController joystickOperator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeCommand m_IntakeCommandDriver = new IntakeCommand(m_IntakeSubsystem, joystickDriver);
    private final IntakeCommand m_IntakeCommandOperator = new IntakeCommand(m_IntakeSubsystem, joystickOperator);

    private final IntakeCommandDouble m_IntakeCommandOne = new IntakeCommandDouble(m_IntakeSubsystem, 0.25);
    private final IntakeCommandDouble m_IntakeCommandZero = new IntakeCommandDouble(m_IntakeSubsystem, 0.0);
    private final IntakeCommandDouble m_IntakeCommandNeg = new IntakeCommandDouble(m_IntakeSubsystem, -0.25);

    private double Limiter = 0;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        NamedCommands.registerCommand("act_intake_forward", new InstantCommand(m_IntakeSubsystem::motorFwd));
        NamedCommands.registerCommand("act_intake_reverse", new InstantCommand(m_IntakeSubsystem::motorRev));
        NamedCommands.registerCommand("act_intake_stop"   , new InstantCommand(m_IntakeSubsystem::motorOff));


        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);



        configureBindings();
    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
        
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystickDriver.getLeftY() * MaxSpeed * 0.4 * (1-(joystickDriver.getLeftTriggerAxis() * 0.25))) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystickDriver.getLeftX() * MaxSpeed * 0.4 * (1-(joystickDriver.getLeftTriggerAxis() * 0.25))) // Drive left with negative X (left)
                    .withRotationalRate(-joystickDriver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        

        joystickDriver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystickDriver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystickDriver.getLeftY(), -joystickDriver.getLeftX()))
        ));

        joystickDriver.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystickDriver.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        joystickDriver.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityY(0.5).withVelocityY(0))
        );
        joystickDriver.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityY(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystickDriver.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


//        joystickDriver.leftTrigger().onTrue(m_IntakeCommandDriver);
//        joystickDriver.rightTrigger().onTrue(m_IntakeCommandDriver);
//        joystickDriver.leftTrigger().onFalse(m_IntakeCommandDriver);
//        joystickDriver.rightTrigger().onFalse(m_IntakeCommandDriver);




        joystickDriver.leftBumper().onTrue(m_IntakeCommandOne);
        joystickDriver.rightBumper().onTrue(m_IntakeCommandNeg);
        joystickDriver.leftBumper().onFalse(m_IntakeCommandZero);
        joystickDriver.rightBumper().onFalse(m_IntakeCommandZero);

        joystickOperator.leftTrigger().onTrue(m_IntakeCommandOperator);
        joystickOperator.rightTrigger().onTrue(m_IntakeCommandOperator);
        joystickOperator.leftTrigger().onFalse(m_IntakeCommandOperator);
        joystickOperator.rightTrigger().onFalse(m_IntakeCommandOperator);        

        //drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
