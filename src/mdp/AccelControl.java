package mdp;

import torcs.Constants;
import torcs.SensorModel;

/**
 * The AccelControl class handles the control and evaluation of acceleration in a TORCS racing simulation.
 */
public class AccelControl {

    public static States evaluateAccelState(SensorModel current) {
        return States.valueOf("STATE_" + stateIndex(current));
    }

    private static int stateIndex(SensorModel current) {
        int currentFrontDistance = (int) Constants.round(current.getTrackEdgeSensors()[9], 0);
        int index = 0;
        for (int i = 0; i < 200; i++) {
            if (i == currentFrontDistance) return currentFrontDistance;
            else {
                if (i > currentFrontDistance) index = i - 5;
            }
        }
        return index;
    }

    public static double accelAction2Double(SensorModel current, Actions action) {
        return action.getValue();
    }

    public static double calculateReward(SensorModel current, Actions action) {
        double reward = 0.0;
        int frontDistance = stateIndex(current);
        if (frontDistance < 20 && action == Actions.ACCEL) {
            reward -= 10.0;
        } else if (frontDistance >= 20 && action == Actions.BRAKE) {
            reward -= 10.0;
        } else {
            reward += 1.0;
        }
        return reward;
    }


    public enum Actions {
        ACCEL(1.0),
        BRAKE(-1.0);

        private double value;

        Actions(double value) {this.value = value;}

        public double getValue() {return this.value;}

    }

    public enum States {
        STATE_0,
        STATE_5,
        STATE_10,
        STATE_15,
        STATE_20,
        STATE_25,
        STATE_30,
        STATE_35,
        STATE_40,
        STATE_45,
        STATE_50,
        STATE_55,
        STATE_60,
        STATE_65,
        STATE_70,
        STATE_75,
        STATE_80,
        STATE_85,
        STATE_90,
        STATE_95,
        STATE_100,
        STATE_105,
        STATE_110,
        STATE_115,
        STATE_120,
        STATE_125,
        STATE_130,
        STATE_135,
        STATE_140,
        STATE_145,
        STATE_150,
        STATE_155,
        STATE_160,
        STATE_165,
        STATE_170,
        STATE_175,
        STATE_180,
        STATE_185,
        STATE_190,
        STATE_195,
    }
}