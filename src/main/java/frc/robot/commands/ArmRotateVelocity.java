// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotateVelocity extends Command {
  ArmSubsystem armSub;
  DoubleSupplier speed;

  /** Creates a new ArmRotateAtRPM. */
  public ArmRotateVelocity(ArmSubsystem armSub, DoubleSupplier speed) {
    this.armSub = armSub;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.setSpeedRPM(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.setSpeedPercent(0);
    
    SmartDashboard.putNumber("Velocity PID Setpoint", speed.getAsDouble());
    SmartDashboard.putNumber("Arm velocity", armSub.getEncoderVelocity());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
