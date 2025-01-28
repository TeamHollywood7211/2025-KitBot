// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;



public class IntakeSubsystem extends SubsystemBase {

  /** Creates a new Intake Subsystem. */
  SparkFlex intakeMotor = new SparkFlex(Constants.IntakeConstants.intakeID, MotorType.kBrushless); //Creates the one intake motor 

  public IntakeSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {

    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }



  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake IR", getIR());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setMotor(double speed) //Just some really basic stuff for adjusting motor, running fwd, etc. 
  {
    intakeMotor.set(speed);
  }
  public void motorFwd()
  {
    intakeMotor.set(IntakeConstants.autoMotorSpeed);
  }
  public void motorRev()
  {
    intakeMotor.set(-IntakeConstants.autoMotorSpeed);
  }
  public void motorOff()
  {
    intakeMotor.set(0);
  }
  
  public boolean getIR()
  {
    return intakeIR.get();
  }


}
