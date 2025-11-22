package frc.robot.subsystems;

import frc.robot.RobotContainer;
//import frc.robot.Constants.CanIds;
//import frc.robot.commands.drivebase.DriveToPickup.TARGETPOS;
//import frc.robot.subsystems.Coral.CoralPhase;
//import frc.utils.CommonLogic;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.annotation.Target;
import java.util.ArrayList; // Ensure this import is complete and used in the code
import java.util.EnumMap;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.function.Predicate;


public class DiverAssist extends SubsystemBase {
    // GLOBAL VARIABLES GO BELOW THIS LINE
    private DAStatus currStatus = DAStatus.INIT;
    private DriveTrain driveTrain;
    private Pose2d currRobotPose;  //current robot pose
    private Pose2d SelectedtargetPose2d;  //selected target pose
    private Pose2d prevTargetPose2d;   //previous loop's selected target pose
    private DriveState currDriveState = DriveState.STATIONARY;
    //private CoralPhase currCoralPhase;
    //private Coral coral;
    String currCmdName = "";
    private FullState currFullState = FullState.UNKOWNSTATE;

    //enum map to store distances to each target
    private EnumMap<Targets, Double> distances = new EnumMap<>(Targets.class);

    //pose tolaerance values
    private static final double positionToleranceMeters = 0.1; // 10 cm tolerance
    private static final double angleToleranceRadians = Math.toRadians(5); // 5 degrees tolerance
    private boolean bAtPrevTarget = false;

    //tactic variables
    private TACTIC_APPROACH currentTactic = TACTIC_APPROACH.T1;
    private ActionStates currentActionState = ActionStates.UNKNOWN;
    
    // GLOBAL VARIABLES GO ABOVE THIS LINE
    // SYSTEM METHODS GO BELOW THIS LINE
    public DiverAssist() {
        

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        switch (currStatus)  {
            case RUNNING:
                getSubsystemState();
                housekeepingTasks();
                break;
            case INIT:
                getSubsystems();
                getSubsystemState();
                currStatus = DAStatus.RUNNING;
                break;
        
            default:
                break;
        }

        // Arm direction is positive when cmdPos is greater than curPos
        /*
         * armMotor.getClosedLoopController().setReference(armCmdPos,
         * ControlType.kMAXMotionPositionControl, ArmCurrentSlot,
         * Math.abs(Math.sin(armCurPos)) * armDirection);
         */
        // Probably should add some safety logic here
        // recommend storing new target position in a variable and then executing the
        // safety logic here.
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void disablePeriodic() {

        
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    // expose the current position
    public void autonInit() {


    }
    // SYSTEM METHODS GO ABOVE THIS LINE
    // PROCESSING METHODS GO BELOW THIS LINE
    private void getSubsystems() {
        driveTrain = RobotContainer.getInstance().m_driveTrain;

        //coral = RobotContainer.getInstance().m_Coral;

    }

    private void getSubsystemState() {
        
        currRobotPose = driveTrain.getPose();
        updateDrivetrainStatus();
        //currCoralPhase = coral.getCoralPhase();
        
    }

    private void housekeepingTasks() {
        // Any periodic housekeeping tasks can be done here.
        // update key status based on current location / pose robot
        prevTargetPose2d = SelectedtargetPose2d;
        bAtPrevTarget = isPose2dCloseEnough(currRobotPose, prevTargetPose2d);

    }

    private void determineCurrentAction() {
        // Determine what action we need to take based on the current state of the
        // subsystems
        
        // Should pass in the current states of all subsystems to determine the full state.
        // Update search methhod with new subsystems as they are added.
        currFullState = getCurFullState(new String[] {currDriveState.name()});

    }

    private void determinePossibleTargets(){
        // Determine what targets are closest based on the current state of the
        // subsystems. Plus other robot state information.

        if (RobotContainer.getInstance().m_sensors.bPickup) {
            // if bPickup is true, we are picking up
            Targets[] TargetSets = getTargets("PICKUP");
        } else {
            // if bPickup is false, we are deploying
            Targets[] TargetSets = getTargets("DEPLOY");
        }

        
            distances.clear();
        for (Targets target : Targets.values()) {
            Pose2d targetPose = target.getTargetPose();
            distances.put(target, getDistanceToTarget(currRobotPose, targetPose.getTranslation()));
            // Store or process the distance to the target as needed
        }


    }

    private void determineBestTarget() {
        // Determine what target is best based on the current state of the subsystems.
        //also include other robot state information.
        //also include game tactics  <- need to figure out how to do this. haha

        int numofTargets = distances.size();  // number of targets
        
        switch (currentActionState) {
            case EMPTY:
                // logic to select best pick up target based on tactic approach
                break;
            case PICKINGUP:
                // logic to select best pick up target based on tactic approach
                break;
            case STOWED:
                // logic to select best deploy target based on tactic approach
                break;
            case DEPLOYING:
                // logic to select best deploy target based on tactic approach
                break;
            case CLIMB_PREP:

                break;
            case CLIMB:

                break;
            default:
                break;
        }
        //if empty use pick up tactic approach or  if picking up use tactic pick up approach

        //if stowed use deployment tactic approach of if deploying use deployment tactic approach


        /* game tatics notes: identify if we are at previous target location,  if we arriaved at pick up point mark pick up complete
         * then move to deploy point. if we are at deploy point mark deploy complete then move to pick up point.
         */


    }

    private void setDriveTarget() {
        // Set the target for the drivebase based on the current state of the
        // subsystems.

    }

    private void determineLikelyActions() {
        // Determine what actions are possible based on the current state of the
        // subsystems.

    }

    private void executeAction() {
        // Execute the action determined by the current state of the subsystems.
        // This may be a simple command or a complex sequence of commands.
        // Need to set articulation and drivebase targets.
    }

    // PROCESSING METHODS GO ABOVE THIS LINE
    // SUPPORTING METHODS GO BELOW THIS LINE

    //method for storing tactic approaches
    private void reviewTacticApproach() {
        // Review the current tactic approach based on the current state of the
        // subsystems.

        //current tactic approach logic here
        //use enum TACTIC_APPROACH store current tactic approach


    }



    public void setTacticApproach(TACTIC_APPROACH tactic) {
        currentTactic = tactic;
    }

    public TACTIC_APPROACH getTacticApproach() {
        return currentTactic;
    }
    

    public Targets[] getTargets(String type) {
    List<Targets> result = new ArrayList<Targets>();
    for (Targets targets : Targets.values()) {
        if (targets.getTargetType() == type) {
            result.add(targets);
        }
    }
    return result.toArray(new Targets[result.size()]);
}

    // Retreive the current command of the drivetrain. 
    private void updateDrivetrainStatus() {
        // Get the current status of the drivetrain.
        currCmdName = driveTrain.getCurrentCommand().getName();
           
            if(driveTrain.getSwerveDrive().getRobotVelocity().vxMetersPerSecond == 0 && 
                    driveTrain.getSwerveDrive().getRobotVelocity().vyMetersPerSecond == 0){
                        currDriveState = DriveState.STATIONARY;
                    } else {
                        currDriveState = DriveState.MOVING;
                    }
        
    }

    

    public FullState getCurFullState(String[] args) {
        
            CompletableFuture<FullState> fullStateFuture = FullState.getFullRobotStateAsync(currDriveState);

            FullState result = fullStateFuture.join();
            return result;
    }


    //get angle to target in Rotation2d in degrees
public Rotation2d getAngleToTarget(Pose2d currentPose, Translation2d targetPosition) {
    Translation2d toTarget = targetPosition.minus(currentPose.getTranslation());
    return new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()));
}

//get relative angle to target in Rotation2d in degrees
public Rotation2d getRelativeAngleToTarget(Pose2d currentPose, Translation2d targetPosition) {
    Rotation2d angleToTarget = getAngleToTarget(currentPose, targetPosition);
    return angleToTarget.minus(currentPose.getRotation());
}


//get distance to target in meters
public double getDistanceToTarget(Pose2d currentPose, Translation2d targetPosition) {
    return currentPose.getTranslation().getDistance(targetPosition);
}

    // SUPPORTING METHODS GO ABOVE THIS LINE
    // ENUMERATIONS GO BELOW THIS LINE 
    public enum DAStatus {
        INIT, // Initialization is averything up to the point of where we are connected to our subsystems.
        RUNNING, // Cheaking the data and running it.
        STOP; // The end when connection is lost.

       // private final double guideAngle;
        /* 
        GuidePos(double pivotAngle) {
            this.guideAngle = pivotAngle;
        }
        
        public double getGuidePos() {
            return guideAngle;
        }
        */
    }

    public enum Targets {
        PICKUPLEFT(new Pose2d(6.0, 1.5, new Rotation2d(Math.toRadians(0.0))), "PICKUP"),
        PICKUPRIGHT(new Pose2d(6.0, 5.5, new Rotation2d(Math.toRadians(0.0))), "PICKUP"),
        ID8LEFT(new Pose2d(12.0, 1.5, new Rotation2d(Math.toRadians(0.0))), "DEPLOY"),
        ID8RIGHT(new Pose2d(12.0, 5.5, new Rotation2d(Math.toRadians(0.0))), "DEPLOY"),
        CLIMBLEFT(new Pose2d(18.0, 1.5, new Rotation2d(Math.toRadians(0.0))),"END");

        private final Pose2d targetPose;
        private final String targetType;
        Targets(Pose2d targetPose, String targetType) {
            this.targetPose = targetPose;
            this.targetType = targetType;

        }

        public Pose2d getTargetPose() {
            return targetPose;
        }

        public String getTargetType() {
            return targetType;
        }
    }


    //system State based on drive train and other subsystems
    public enum FullState {
        STATIONARY(DriveState.STATIONARY),
        MOVING(DriveState.MOVING),
        UNKOWNSTATE(DriveState.STATIONARY);

        private final DriveState driveTrainState;

        FullState(DriveState driveTrainState) {
            this.driveTrainState = driveTrainState;
        }

        public DriveState getDriveState() {
            return driveTrainState;
        }

        public static CompletableFuture<FullState> getFullRobotStateAsync(DriveState driveTrainState) {
            return CompletableFuture.supplyAsync(() -> {
                for (FullState state : FullState.values()) {
                    if (state.getDriveState() == driveTrainState) {
                        return state;
                    }
                }
                return UNKOWNSTATE;
                
            });
        }
    }

    //state of DriveTrain subsystem
    public enum DriveState {
        STATIONARY,
        MOVING;
    }

    //Tactic Approaches pickup
    public enum PICKUP_TACTIC {
        NEARESTPICKUP,
        RIGHTPICKUP,
        LEFTPICKUP;
        
    }

    public enum DEPLOY_TACTIC {
        NEARESTDEPLOY,
        ID8RIGHTDEPLOY,
        ID8LEFTDEPLOY;
    }
    public enum END_TACTIC {
        NEARESTCLIMB,
        LEFTCLIMB,
        RIGHTCLIMB;
    }

    public enum TACTIC_APPROACH{
        T1(PICKUP_TACTIC.NEARESTPICKUP, DEPLOY_TACTIC.NEARESTDEPLOY, END_TACTIC.LEFTCLIMB),
        T2(PICKUP_TACTIC.NEARESTPICKUP, DEPLOY_TACTIC.ID8RIGHTDEPLOY, END_TACTIC.LEFTCLIMB),
        T3(PICKUP_TACTIC.RIGHTPICKUP, DEPLOY_TACTIC.ID8LEFTDEPLOY , END_TACTIC.LEFTCLIMB);

        private PICKUP_TACTIC pickTactic;
        private DEPLOY_TACTIC deployTactic;
        private END_TACTIC endTactic;

        TACTIC_APPROACH(PICKUP_TACTIC pickup, DEPLOY_TACTIC deploy, END_TACTIC end) {
            this.pickTactic = pickTactic;
            this.deployTactic = deployTactic;
            this.endTactic = endTactic;
        }
        public PICKUP_TACTIC getPickTactic() {
            return pickTactic;
        }
        public DEPLOY_TACTIC getDeployTactic() {
            return deployTactic;
        }
        public END_TACTIC getEndTactic() {
            return endTactic;
        }   
    }

    public enum ActionStates {
        STOWED,
        PICKINGUP,
        DEPLOYING,
        EMPTY,
        CLIMB_PREP,
        CLIMB,
        UNKNOWN;
    }

    // ENUMERATIONS GO ABOVE THIS LINE
    // REFERENCE METHODS GO BELOW THIS LINE

 /**
     * Checks if two poses are within given tolerances.
     *
     * @param currentPose The robot's current pose
     * @param targetPose The desired target pose
     * @param positionToleranceMeters Allowed distance tolerance (meters)
     * @param angleToleranceRadians Allowed angle tolerance (radians)
     * @return true if within tolerance, false otherwise
     */
    public static boolean isPose2dCloseEnough(Pose2d currentPose, Pose2d targetPose ) {
        // Calculate translation distance
        double dx = currentPose.getX() - targetPose.getX();
        double dy = currentPose.getY() - targetPose.getY();
        double distance = Math.hypot(dx, dy);

        // Calculate angular difference
        double angleDiff = Math.abs(
            currentPose.getRotation().minus(targetPose.getRotation()).getRadians()
        );

        return (distance <= positionToleranceMeters) && (angleDiff <= angleToleranceRadians);
    }




public static double CapMotorPower(double MotorPower, double negCapValue, double posCapValue) {
    // logic to cap the motor power between a good range
    double retValue = MotorPower;

    if (MotorPower < negCapValue) {
      retValue = negCapValue;
    }

    if (MotorPower > posCapValue) {
      retValue = posCapValue;
    }

    return retValue;
  }

  public static final double joyDeadBand(double joy, double deadband) {

    double retValue = joy;
    if (Math.abs(retValue) < Math.abs(deadband)) {
      retValue = 0;
    }
    return Math.pow(retValue, 2) * Math.signum(joy);
  }

  public static final boolean isInRange(double curValue, double targetValue, double Tol) {

    double loVal = targetValue - Tol;
    double hiVal = targetValue + Tol;
    boolean retValue = false;

    if (curValue > loVal && curValue < hiVal) {
      retValue = true;
    }
    return retValue;
  }

  public static double getTimeInSeconds() {
    return (System.nanoTime() / Math.pow(10, 9)); // returns time in seconds
  }

  public double deg2Radians(double degrees) {
    return Math.toRadians(degrees);
  }

  public static double calcArcLength(double degrees, double radius) {
    return (Math.toRadians(degrees) * radius);
  }

  public static double gotoPosPIDF(double P, double F_hold, double currentPos, double targetPos) {
    double delta = targetPos - currentPos;

    return (delta * P) + F_hold;

  }

  public static double AutoTurnPIDF(double P, double F_hold, double currentPos, double targetPos) {
    double delta = targetPos - currentPos;
    if(delta > 180){
      delta = -delta;
    }

    return (delta * P) + F_hold;

  }

public static int getIsBlue() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            // is blue = false
            return 0;
        }
        if (ally.get() == Alliance.Blue) {
            // is blue = true
            return 1;
        }
    } else {
        // naptime
        return -1;
    }
        return -1;
}



public static double getTimeInMIlliseconds() {
    return (System.nanoTime() / Math.pow(10, 6));
}

public static double headingDeltaInDegree(double currentHeading, double targetHeading) {
    double headingDelta = 0;
    double invertedHeadingDelta = 0;

    // Positive value
    if (currentHeading >= 0 && targetHeading >= 0) {
        headingDelta = targetHeading - currentHeading;
    }
    // one of each
    else if (currentHeading >= 0 && targetHeading <= 0) {
        // headingDelta = (targetHeading + currentHeading);
        headingDelta = Math.abs(targetHeading) + Math.abs(currentHeading);
        invertedHeadingDelta = Math.abs(360 + targetHeading) - Math.abs(currentHeading);
        headingDelta = Math.min(Math.abs(headingDelta), Math.abs(invertedHeadingDelta));
        if (invertedHeadingDelta != headingDelta) {
            headingDelta = headingDelta * -1;
        }
    }
    // one of each again
    else if (currentHeading <= 0 && targetHeading >= 0) {
        // headingDelta = -1 * (targetHeading + currentHeading);
        headingDelta = Math.abs(targetHeading) + Math.abs(currentHeading);
        invertedHeadingDelta = Math.abs(360 - targetHeading) - Math.abs(currentHeading);
        headingDelta = Math.min(Math.abs(headingDelta), Math.abs(invertedHeadingDelta));
        if (invertedHeadingDelta == headingDelta) {
            headingDelta = headingDelta * -1;
        }
    }
    // both negative
    else if (currentHeading <= 0 && targetHeading <= 0) {
        headingDelta = targetHeading - currentHeading;
    }
    return headingDelta;
}

public static double calcTurnRateAbs(double currentHeading, double targetHeading, double proportion) {

    double headingDelta = headingDeltaInDegree(currentHeading, targetHeading);

    double commandedTurnRate = headingDelta * proportion;
    return commandedTurnRate; // IS ALWAYS POSITIVE!
}

public static double metersToInches(double meter) {
    return meter / 0.0254;
}

public static double inchesToMeters(double inches) {
    return inches * 0.0254;
}

    // REFERENCE METHODS GO ABOVE THIS LINE

}
