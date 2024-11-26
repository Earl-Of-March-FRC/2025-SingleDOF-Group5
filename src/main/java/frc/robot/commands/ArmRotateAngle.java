// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotateAngle extends Command {
  private ArmSubsystem armSub;
  private DoubleSupplier angle;

  /** Creates a new ArmRotateTo. */
  public ArmRotateAngle(ArmSubsystem armsub, DoubleSupplier angle) {
    this.armSub = armsub;
    this.angle = angle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.rotateToAngle(angle.getAsDouble());
    SmartDashboard.putNumber("Arm PID Setpoint", angle.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSub.getEncoderAngle() == angle.getAsDouble();
  }
}
