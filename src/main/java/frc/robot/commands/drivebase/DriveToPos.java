package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveToPos extends Command {
    private boolean bDone = false;
    //private double bHeading;
    //private double rHeading;
    private int latestID;
    private Pose2d newTarget;
    private DriveTrain m_driveTrain;
  
    

    
   
    public DriveToPos(Pose2d ntarget, DriveTrain m_driveTrain) {
        newTarget = ntarget;
         addRequirements(m_driveTrain);
         this.m_driveTrain = m_driveTrain;

        

    }
    // if fixedDist = false => stagPosition is suposed to recieve the percantage to
    // be traversed in stag, in 0.xx format

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_driveTrain.driveToPose(newTarget);
        bDone = false;
        

   
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
       
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.driveToPose(newTarget).end(interrupted);
        bDone = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_driveTrain.driveToPose(newTarget).isFinished();
        //return bDone;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }

  }