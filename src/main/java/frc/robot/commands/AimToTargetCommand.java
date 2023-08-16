package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.LimelightInterface;


public class AimToTargetCommand extends CommandBase {
    private Drive drive;
    private PIDController angleController; 
    private LimelightInterface limelightInterface; 

    private double minSpeed = 0.05;

    public AimToTargetCommand(Drive drive, LimelightInterface limelightInterface){
        this.drive = drive;
        this.limelightInterface= limelightInterface;

        angleController = new PIDController(.18, 0, 0);

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(Units.degreesToRadians(0.2));

        addRequirements(drive);
    }

    @Override
    public void initialize(){
        angleController.reset();
        angleController.setSetpoint(0);
    }

    @Override
    public void execute() {
        double output = angleController.calculate(Units.degreesToRadians(limelightInterface.getTX()));
        output = Math.copySign(minSpeed, output) + output; 

        drive.DriveTank(-output, output);
    }

    @Override 
    public boolean isFinished() {
        return angleController.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted){
        drive.stop();
    }
}
