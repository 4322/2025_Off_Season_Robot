package frc.robot.commands;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.deployer.Deployer;
import frc.robot.subsystems.elevator.Elevator;

public class CoastCommand extends Command {
  private Timer coastTimer = new Timer();
  private Deployer deployer;
  private Arm arm;
  private Elevator elevator;

  public CoastCommand(Arm arm, Elevator elevator, Deployer deployer, Superstructure superstructure) {
    this.deployer = deployer;
    this.arm = arm;
    this.elevator = elevator;
    addRequirements(arm, elevator, deployer);
  }

  @Override
  public void initialize() {
    coastTimer.start();
    arm.stop(IdleMode.kCoast);
    elevator.stop(IdleMode.kCoast);
    deployer.stop(IdleMode.kCoast);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return coastTimer.hasElapsed(10) || DriverStation.isEnabled();
  }

  @Override
  public void end(boolean interrupted) {
    arm.stop(IdleMode.kBrake);
    elevator.stop(IdleMode.kBrake);
    deployer.stop(IdleMode.kBrake);
    coastTimer.stop();
    coastTimer.reset();
  }
}
