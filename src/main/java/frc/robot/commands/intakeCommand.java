package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntake;

public class intakeCommand extends Command {
    double value;
    GroundIntake intake;
    boolean active;
    public intakeCommand(double intakeValue, GroundIntake subsystem, boolean Active){
        this.value=intakeValue;
        this.intake=subsystem;
        this.active=Active;
        addRequirements(subsystem);
    }
    @Override
    public void execute(){
        if (active) {
        intake.Intake(value);    
        } else {
        intake.Intake(.1);
        }
    }
    @Override
    public boolean isFinished(){
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            intake.Intake(0);
        }
    }
}
