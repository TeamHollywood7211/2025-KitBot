// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class IntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_subsystem;
  private final CommandXboxController m_controller;


  /**
   * Creates our IntakeCommand
   *
   * @param subsystem The subsystem used by this command.
   * @param controller The controller used by this command
   */
  public IntakeCommand(IntakeSubsystem subsystem, CommandXboxController controller) {
    m_subsystem = subsystem;
    m_controller = controller;

    addRequirements(subsystem);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    m_subsystem.setMotor((m_controller.getLeftTriggerAxis() - m_controller.getRightTriggerAxis()) * IntakeConstants.teleopMotorSpeed);
    //Sets the motor speed depending on the controllers triggers. Think like this
    //Left trigger pulled, right trigger not, then (1-0 = 1), motor at full positive power.
    //Left trigger not, right trigger pulled, then (0-1 = -1), motor at full negative power.
  
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
