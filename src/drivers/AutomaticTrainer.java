package drivers;

import mdp.AccelControl;
import mdp.QLearning;
import mdp.SteerControl;
import torcs.*;

public class AutomaticTrainer extends Controller {

    /* Stuck constants*/
    final int stuckTime = 25;
    final float stuckAngle = (float) 0.523598775; // PI/6

    private final double targetSpeed = 40.0;

    /* Q-learning Steer Control */
    private QLearning steerControlSystem;
    private SteerControl.States previousSteerState;
    private SteerControl.States currentSteerState;
    private SteerControl.Actions actionSteer;
    private double steerReward;

    /* Q-learning AccelControl */
    private QLearning accelControlSystem;
    private AccelControl.States previousAccelState;
    private AccelControl.States currentAccelState;
    private AccelControl.Actions actionAccel;
    private double accelReward;

    /* Time, Laps and Statistics Variables */
    private int tics;
    private int epochs;
    private int laps;
    private double previosDistanceFromStartLine;
    private double currentDistanceFromStartLine;
    private int completeLaps;
    private double distanceRaced;
    private double highSpeed;
    private SensorModel previousSensors;
    private SensorModel currentSensors;

    /* Cache variables */
    private int stuck;
    private double clutch;
    private boolean completeLap;
    private boolean offTrack;
    private boolean timeOut;

    public AutomaticTrainer() {
        steerControlSystem = new QLearning(Constants.ControlSystems.STEERING_CONTROL_SYSTEM);
        previousSteerState = SteerControl.States.CENTER;
        currentSteerState = SteerControl.States.CENTER;
        actionSteer = SteerControl.Actions.TURN_C;
        steerReward = 0;

        accelControlSystem = new QLearning(Constants.ControlSystems.ACCELERATION_CONTROL_SYSTEM);
        previousAccelState = AccelControl.States.STATE_195;
        currentAccelState = AccelControl.States.STATE_195;
        actionAccel = AccelControl.Actions.ACCEL;
        accelReward = 0;

        tics = 0;
        epochs = 0;
        laps = -1;
        completeLaps = 0;
        distanceRaced = 0;
        highSpeed = 0;

        stuck = 0;
        clutch = 0;
        completeLap = false;
        offTrack = false;
        timeOut = false;
    }

    public void reset() {
        System.out.println("Restarting the race!");
    }

    public void shutdown() {
        System.out.println("Bye bye!");
    }

    public Action control(SensorModel sensors) {
        if (this.tics == 0) {
            this.previosDistanceFromStartLine = sensors.getDistanceFromStartLine();
            this.currentDistanceFromStartLine = this.previosDistanceFromStartLine;

            this.previousSensors = sensors;
            this.currentSensors = this.previousSensors;

            this.tics++;
        } else {
            this.previosDistanceFromStartLine = this.currentDistanceFromStartLine;
            this.currentDistanceFromStartLine = sensors.getDistanceFromStartLine();

            this.previousSensors = this.currentSensors;
            this.currentSensors = sensors;

            this.tics++;
        }

        // Check if time-out
        if (this.currentSensors.getLastLapTime() > 240.0) {
            this.timeOut = true;

            Action action = new Action();
            action.restartRace = true;
            return action;
        }

        // Update raced distance
        this.distanceRaced = this.currentSensors.getDistanceRaced();

        // Update high speed
        if (this.currentSensors.getSpeed() > this.highSpeed) {
            this.highSpeed = this.currentSensors.getSpeed();
        }

        // Update complete laps
        if (this.previosDistanceFromStartLine > 1 && this.currentDistanceFromStartLine < 1) {
            this.laps++;

            // Car start back the goal, so ignore first update
            // If the car complete the number of laps, restart the race
            if (this.laps >= 1) {
                this.completeLap = true;

                Action action = new Action();
                action.restartRace = true;
                return action;
            }
        }

        // If the car is off track, restart the race
        if (Math.abs(this.currentSensors.getTrackPosition()) >= 1) {
            this.offTrack = true;

            Action action = new Action();
            action.restartRace = true;
            return action;
        }

        // Check if car is currently stuck
        if (Math.abs(this.currentSensors.getAngleToTrackAxis()) > DrivingInstructor.stuckAngle) {
            this.stuck++;
        } else {
            this.stuck = 0;
        }

        // After car is stuck for a while apply recovering policy
        if (this.stuck > DrivingInstructor.stuckTime) {
            // Set gear and steering command assuming car is pointing in a direction out of track

            // To bring car parallel to track axis
            float steer = (float) (-this.currentSensors.getAngleToTrackAxis() / DrivingInstructor.steerLock);
            int gear = -1; // gear R

            // If car is pointing in the correct direction revert gear and steer
            if (this.currentSensors.getAngleToTrackAxis() * this.currentSensors.getTrackPosition() > 0) {
                gear = 1;
                steer = -steer;
            }

            this.clutch = (double) DrivingInstructor.clutching(this.currentSensors, (float) this.clutch, getStage());

            // Build a CarControl variable and return it
            Action action = new Action();
            action.gear = gear;
            action.steering = steer;
            action.accelerate = 1.0;
            action.brake = 0;
            action.clutch = clutch;

            return action;
        }


        // If the car is not stuck
        Action action = new Action();

        // Calculate gear value ----------------------------------------------------------------------------------------
        action.gear = DrivingInstructor.getGear(this.currentSensors);

        // Calculate steer value ---------------------------------------------------------------------------------------
        double steer;
        this.previousSteerState = this.currentSteerState;
        this.currentSteerState = SteerControl.evaluateSteerState(this.currentSensors);
        this.steerReward = SteerControl.calculateReward(this.currentSensors);
        this.actionSteer = (SteerControl.Actions) this.steerControlSystem.update(
                this.previousSteerState,
                this.currentSteerState,
                this.actionSteer,
                this.steerReward
        );
        steer = SteerControl.steerAction2Double(this.actionSteer);

        // normalize steering
        if (steer < -1) steer = -1;
        if (steer > 1) steer = 1;
        action.steering = steer;

        // Calculate accel/brake ---------------------------------------------------------------------------------------
        float accel_and_brake = DrivingInstructor.getAccel(this.currentSensors);

        // Set accel and brake from the joint accel/brake command
        double accel_brake;
        this.previousAccelState = this.currentAccelState;
        this.currentAccelState = AccelControl.evaluateAccelState(this.currentSensors);
        this.accelReward = AccelControl.calculateReward(this.currentSensors, this.actionAccel);
        this.actionAccel = (AccelControl.Actions) this.accelControlSystem.update(
                this.previousAccelState,
                this.currentAccelState,
                this.actionAccel,
                this.accelReward
        );
        accel_brake = AccelControl.accelAction2Double(this.currentSensors, this.actionAccel);
        if (accel_brake >= 0) {
            action.accelerate = accel_brake;
            action.brake = 0.0;
        } else if (this.currentSensors.getSpeed() >= targetSpeed) {
            action.accelerate = 0.0;
            action.brake = 0.0;
        } else {
            action.accelerate = 0.0;
            action.brake = accel_brake;
        }

        // Calculate clutch --------------------------------------------------------------------------------------------
        this.clutch = DrivingInstructor.clutching(this.currentSensors, (float) this.clutch, getStage());
        action.clutch = this.clutch;

        return action;
    }

}
