// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotateVelocityPID extends Command {
  private ArmSubsystem armSub;
  private PIDController controller;
  private DoubleSupplier setpoint;
  private DoubleSupplier tolerance;
  private double bias;

  /** Creates a new ArmRotateTo. */
  public ArmRotateVelocityPID(ArmSubsystem armsub, DoubleSupplier setpoint, DoubleSupplier tolerance) {
    this.armSub = armsub;
    this.setpoint = setpoint;
    this.tolerance = tolerance;
    controller = armsub.getVelController();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(setpoint.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.setSetpoint(setpoint.getAsDouble());
    bias = setpoint.getAsDouble()/Constants.ArmConstants.maxRPM;
    double PIDOutput = controller.calculate(armSub.getEncoderRPM());

    armSub.setVoltage(PIDOutput);
    
    SmartDashboard.putNumber("Velocity PID Output", PIDOutput);
    SmartDashboard.putNumber("Velocity PID Setpoint", setpoint.getAsDouble());
    SmartDashboard.putNumber("Velocity PID Bias", bias);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.setSpeedPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
