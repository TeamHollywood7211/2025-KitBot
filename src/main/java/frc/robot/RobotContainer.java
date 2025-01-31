// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.ImportantPositions;
import frc.robot.Constants.autoConfigConstants;
import frc.robot.commands.Auto_IntakeWaitForPiece;
import frc.robot.commands.ClimberCommand;

import frc.robot.commands.IntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MusicSubsystem;

public class RobotContainer {

    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    //private final MusicSubsystem m_MusicSubsystem = new MusicSubsystem();
    private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    public static double OriginalMaxSpeed = MaxSpeed*0.5;
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                             // second max angular
                                                                                             // velocity



    /* Setting up bindings for necessary control of the swerve drive platform */
    public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();
    private final static SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    /*
     * public final static SwerveRequest.RobotCentric autonMovement = new
     * SwerveRequest.RobotCentric()
     * .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
     * .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.3);
     */
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverStick   = new CommandXboxController(0); // For driver
    private final CommandXboxController operatorStick = new CommandXboxController(1); // For operator
    private final CommandXboxController autoStick     = new CommandXboxController(2); // For handling the autonomous "drive to positions"

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeCommand m_IntakeCommand = new IntakeCommand(m_IntakeSubsystem, operatorStick);

    private final ClimberCommand m_ClimberCommand = new ClimberCommand(m_ClimberSubsystem, operatorStick);
    private final Auto_IntakeWaitForPiece a_waitForPiece = new Auto_IntakeWaitForPiece(m_IntakeSubsystem);
    // PID
    public final static PIDController ll_rotatePID = new PIDController(0.3, 0, 0.005);

   


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public void createFrontUsbCamera() {
        CameraServer.startAutomaticCapture(); // Camera stuff :3
    }

    public RobotContainer() {

        NamedCommands.registerCommand("act_intake_eject", new InstantCommand(m_IntakeSubsystem::motorRev));
        NamedCommands.registerCommand("act_intake_off"   , new InstantCommand(m_IntakeSubsystem::motorOff));
        NamedCommands.registerCommand("act_wait_for_piece", a_waitForPiece);

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        createFrontUsbCamera();
        configureBindings();
        LimelightHelpers.outputToSmartDashboard(); //This is a quick function that just outputs the X and Y to SmartDashboard
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // (Wow I've never heard of any program that says X is forward lol -Noah)
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverStick.getLeftY() * MaxSpeed) // Drive forward
                                                                                                      // with negative Y
                                                                                                      // (forward)
                        .withVelocityY(-driverStick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverStick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                       // negative X (left)
                )

        );
        m_IntakeSubsystem.setDefaultCommand(m_IntakeCommand); 
        m_ClimberSubsystem.setDefaultCommand(m_ClimberCommand);

        driverStick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverStick.leftTrigger().onTrue(new InstantCommand(drivetrain::setDriveSlow));
        driverStick.leftTrigger().onFalse(new InstantCommand(drivetrain::setDriveNormal));

        driverStick.pov(0).whileTrue(drivetrain
                .applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0)));
        driverStick.pov(180).whileTrue(drivetrain
                .applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0).withRotationalRate(0))

        );
        driverStick.pov(90).whileTrue(drivetrain
                .applyRequest(() -> forwardStraight.withVelocityY(0.5).withVelocityX(0).withRotationalRate(0)));
        driverStick.pov(270).whileTrue(drivetrain
                .applyRequest(() -> forwardStraight.withVelocityY(-0.5).withVelocityX(0).withRotationalRate(0)));

        // reset the field-centric heading on left bumper press
        driverStick.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driverStick.x().whileTrue(drivetrain.run(() -> followAprilTag()));

        // AUTO STICK
        autoStick.a().onTrue(drivetrain.run(() -> autoToPosition(ImportantPositions.coral4))); // dont ask me how
                                                                                               // drivetrain.run() is
                                                                                               // used here, it just
                                                                                               // maybe works.
                                                                                               
        //autoStick.b().onTrue(m_MusicSubsystem.runOnce(() -> m_MusicSubsystem.playSong("chirp/portal.chrp")));
        // OPERATOR STICK

        operatorStick.y().onTrue(new InstantCommand(m_ClimberSubsystem::setArmHigh))     ;
        operatorStick.b().onTrue(new InstantCommand(m_ClimberSubsystem::setArmClimb))    ;
        operatorStick.povLeft().onTrue(new InstantCommand(m_ClimberSubsystem::setArmLow));
        //operatorStick.leftTrigger().onTrue(m_IntakeCommand);
        //operatorStick.rightTrigger().onTrue(m_IntakeCommand);
        //operatorStick.leftTrigger().onFalse(m_IntakeCommand);
        //operatorStick.rightTrigger().onFalse(m_IntakeCommand);

        // DUMB TELEMETRY THATS ACTUALLY REALLY USEFUL BUT IT ANNOYS ME

        drivetrain.registerTelemetry(logger::telemeterize);
        

    }


    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public static double limelight_range_proportional() {
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= MaxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }

    public static double limelight_aim_proportional() {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our
        // proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
        // rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= MaxAngularRate;

        // invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    public void followAprilTag() { // Basic method for driving towards a limelight. Good for debugging not likely
                                   // to actually use this.
        final var forward_limelight = RobotContainer.limelight_range_proportional();
        final var rot_limelight = RobotContainer.limelight_aim_proportional();

        double current_rotation = drivetrain.getGyro().getDegrees();
        double pid_output = ll_rotatePID.calculate(current_rotation, current_rotation + rot_limelight);

        SmartDashboard.putNumber("Limelight Rot", rot_limelight);
        SmartDashboard.putNumber("Limelight Fwd", forward_limelight);

        drivetrain.setControl(
                forwardStraight.withVelocityX(MathUtil.clamp(forward_limelight, -.5, .5))
                        .withVelocityY(0)
                        .withRotationalRate(MathUtil.clamp(pid_output, -1, 1)));

    }

    public void autoToPosition(double[] positions) // Should in theory move us to a pose of x, y. Waiting till we have
                                                   // more of the field to tell
    {
        // stolen from https://pathplanner.dev/pplib-pathfinding.html#pathfind-to-pose
        String stupid_dumb_string = String.format("Positions are %s, $2d", positions[0], positions[1]);
        System.out.println(stupid_dumb_string);

        Pose2d targetPose = new Pose2d(positions[0], positions[1], Rotation2d.fromDegrees(positions[2]));
        PathConstraints constraints = new PathConstraints(
                autoConfigConstants.maxVelocityMPS, autoConfigConstants.maxAngularVelocitySq,
                Units.degreesToRadians(autoConfigConstants.maxAngularVelocity),
                Units.degreesToRadians(autoConfigConstants.maxAngularVelocitySq));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0);
    }
    public static double booleanToDouble(boolean b) {
        if (b) {
            return 1;
        }
        return 0;
    }
}
