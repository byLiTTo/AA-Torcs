package mdp;

import torcs.SensorModel;

public class SteerControl {

    public static States evaluateSteerState(SensorModel current) {
        double distanceToEdge = current.getTrackPosition();
        if (distanceToEdge < 0.5d && distanceToEdge > 0.15d) {
            return States.LEFT_MIDDLE;
        } else if (distanceToEdge >= 0.5d && distanceToEdge < 1.0d) {
            return States.LEFT_BORDER;
        } else if (distanceToEdge > -0.5d && distanceToEdge < -0.15d) {
            return States.RIGHT_MIDDLE;
        } else if (distanceToEdge <= -0.5d && distanceToEdge > -1.0d) {
            return States.RIGHT_BORDER;
        } else {
            return States.CENTER;
        }
    }

    public static double steerAction2Double(Actions action) {
        return action.getAngle();
    }

    public static double calculateReward(SensorModel current) {
        return 1 - Math.abs(current.getTrackPosition());
    }

    public enum Actions {

        TURN_L_HARD(0.4d),
        TURN_L(0.1d),
        TURN_C(0.0d),
        TURN_R(-0.1d),
        TURN_R_HARD(-0.4d);


        private double angle;

        Actions(double value) {this.angle = value;}

        public double getAngle() {return this.angle;}
    }

    public enum States {
        LEFT_BORDER,
        LEFT_MIDDLE,
        CENTER,
        RIGHT_MIDDLE,
        RIGHT_BORDER,
        OFFTRACK
    }
}
