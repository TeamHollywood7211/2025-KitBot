// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.IntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;


public class RobotContainer {

    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();


    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double OriginalMaxSpeed = MaxSpeed;
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
        /* Setting up bindings for necessary control of the swerve drive platform */
        public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final static SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
       /*          public final static SwerveRequest.RobotCentric autonMovement = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.3);
        */    
                private final Telemetry logger = new Telemetry(MaxSpeed);
            
                private final CommandXboxController joystick = new CommandXboxController(0);
                private final CommandXboxController operatorStick = new CommandXboxController(1);
                
                public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
                private final IntakeCommand m_IntakeCommand = new IntakeCommand(m_IntakeSubsystem, operatorStick);
            
                //PID
                public final static PIDController ll_rotatePID = new PIDController(0.3, 0, 0.005);
                
                
                
                                /* Path follower */
                                private final SendableChooser<Command> autoChooser;
                                
                                public void createFrontUsbCamera() {
                                    CameraServer.startAutomaticCapture(); //Camera stuff :3
                                }
                            
                            
                                public RobotContainer() {
                                    NamedCommands.registerCommand("act_intake_forward", new InstantCommand(m_IntakeSubsystem::motorFwd));
                                    NamedCommands.registerCommand("act_intake_reverse", new InstantCommand(m_IntakeSubsystem::motorRev));
                                    NamedCommands.registerCommand("act_intake_stop"   , new InstantCommand(m_IntakeSubsystem::motorOff));
                            
                            
                                    autoChooser = AutoBuilder.buildAutoChooser("Tests");
                                    SmartDashboard.putData("Auto Mode", autoChooser);
                            
                            
                                    createFrontUsbCamera();
                                    configureBindings();
                                }
                                
                                private void configureBindings() {
                                    // Note that X is defined as forward according to WPILib convention,
                                    // and Y is defined as to the left according to WPILib convention.
                                    drivetrain.setDefaultCommand(
                                        // Drivetrain will execute this command periodically
                                        drivetrain.applyRequest(() ->
                                            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                                        )
                                    );
                                    
                                    //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                                    /*joystick.b().whileTrue(drivetrain.applyRequest(() ->
                                        point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
                                    ));*/
                            
                                    joystick.leftTrigger().onTrue(new InstantCommand(drivetrain::setDriveSlow));
                                    joystick.leftTrigger().onFalse(new InstantCommand(drivetrain::setDriveNormal));
                                    
                                    joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
                                        forwardStraight.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
                                    );
                                    joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
                                        forwardStraight.withVelocityX(-0.5).withVelocityY(0).withRotationalRate(0))
                                        
                                    );
                                    joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
                                        forwardStraight.withVelocityY(0.5).withVelocityX(0).withRotationalRate(0))
                                    );
                                    joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
                                        forwardStraight.withVelocityY(-0.5).withVelocityX(0).withRotationalRate(0))
                                    );
                            
                                    
                            
                            
                                    // Run SysId routines when holding back/start and X/Y.
                                    // Note that each routine should be run exactly once in a single log.
                                    //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                                    //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                                    //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                                    //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
                            
                                    // reset the field-centric heading on left bumper press
                                    joystick.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                                    //joystick.x().whileTrue(new InstantCommand(drivetrain::followAprilTag));
                                    //joystick.x().whileTrue(drivetrain.run(() -> followAprilTag())); //will this run???? Lets see once the meeting is over????
                                    joystick.x().whileTrue(drivetrain.run(() -> followAprilTag()));                                    
                                
                            
                            
                                    operatorStick.leftTrigger().onTrue(m_IntakeCommand);
                                    operatorStick.rightTrigger().onTrue(m_IntakeCommand);
                                    operatorStick.leftTrigger().onFalse(m_IntakeCommand);
                                    operatorStick.rightTrigger().onFalse(m_IntakeCommand);
                            
                                    drivetrain.registerTelemetry(logger::telemeterize);
                                }
                            
                                public Command getAutonomousCommand() {
                                    /* Run the path selected from the auto chooser */
                                    return autoChooser.getSelected();
                                }
                            
                            
                                public static double limelight_range_proportional()
                                {    
                                  double kP = .1;
                                  double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
                                  targetingForwardSpeed *= MaxSpeed;
                                  targetingForwardSpeed *= -1.0;
                                  return targetingForwardSpeed;
                                }
                            
                                public static double limelight_aim_proportional()
                                {    
                                  // kP (constant of proportionality)
                                  // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
                                  // if it is too high, the robot will oscillate.
                                  // if it is too low, the robot will never reach its target
                                  // if the robot never turns in the correct direction, kP should be inverted.
                                  double kP = .035;
                              
                                  // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
                                  // your limelight 3 feed, tx should return roughly 31 degrees.
                                  double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
                              
                                  // convert to radians per second for our drive method
                                  targetingAngularVelocity *= MaxAngularRate;
                                    
                              //invert since tx is positive when the target is to the right of the crosshair
                                targetingAngularVelocity *= -1.0;
                                    
                                return targetingAngularVelocity;
                                }

                            public static void followAprilTag(){
                                final var forward_limelight = RobotContainer.limelight_range_proportional();
                                final var rot_limelight = RobotContainer.limelight_aim_proportional();
                                

                                double current_rotation = drivetrain.getGyro().getDegrees();
                                
                                double pid_output = ll_rotatePID.calculate(current_rotation, current_rotation+rot_limelight);


                                SmartDashboard.putNumber("Limelight Rot", rot_limelight);
                                SmartDashboard.putNumber("Limelight Fwd", forward_limelight);
                                
                                drivetrain.setControl(
                                    forwardStraight.withVelocityX(MathUtil.clamp(forward_limelight, -.5, .5))
                                .withVelocityY(0) 
                                .withRotationalRate(MathUtil.clamp(pid_output, -1, 1))
                                //.withRotationalRate(MathUtil.clamp(rot_limelight, -.25, .25))
                                
                                //.withRotationalRate(MathUtil.clamp())
                );
        }

}
