// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SparkMax ArmMotor = new SparkMax(ClimberConstants.armMotorID, MotorType.kBrushless);
  PIDController ArmPID = new PIDController(0.03, 0, 0.005);

  RelativeEncoder ArmEncoder = ArmMotor.getEncoder();
  double ArmSetpoint = ArmEncoder.getPosition();
  public ClimberSubsystem() {
    

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    double currentPos = ArmEncoder.getPosition();
    SmartDashboard.putNumber("New Arm Position", currentPos);
    SmartDashboard.putNumber("New Arm Target Pos", ArmSetpoint);
    ArmMotor.set(MathUtil.clamp(ArmPID.calculate(currentPos, ArmSetpoint), -0.5, 0.5));
    SmartDashboard.putNumber("Percentage to Pos: ", (currentPos / ArmSetpoint) * 100);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setMotor(double speed)
  {
    ArmMotor.set(speed);
  }
  public void setArmHigh()
  {
    ArmSetpoint = 120;
  }
  public void setArmLow()
  {
    ArmSetpoint = 0;
  }
  public void setArmClimb()
  {
    ArmSetpoint = 100;
  }
}
