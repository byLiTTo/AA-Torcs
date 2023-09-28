package mdp;

import torcs.Constants;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.*;

import static torcs.Constants.*;


public class QLearning {

    private static final String SEPARATOR = ",";
    private final HashMap<String, HashMap<String, Double>> qTable;
    private List<Object> possibleActions = null;
    private Object lastState;

    private double epsilon = 0.0d;
    private Random random;
    private ControlSystems system;
    private String qTablePath;

    public QLearning(ControlSystems system) {
        this.qTable = new HashMap<>();

        this.random = new Random(System.currentTimeMillis());

        this.system = system;
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                this.possibleActions = Arrays.asList(SteerControl.Actions.values());
                this.qTablePath = STEER_Q_TABLE_PATH;
                break;
            case ACCELERATION_CONTROL_SYSTEM:
                this.possibleActions = Arrays.asList(AccelControl.Actions.values());
                this.qTablePath = ACCEL_Q_TABLE_PATH;
                break;
            case GEAR_CONTROL_SYSTEM:
                this.possibleActions = Arrays.asList(GearControl.Actions.values());
                this.qTablePath = GEAR_Q_TABLE_PATH;
                break;
        }
        File f = new File(this.qTablePath);
        if (!f.exists())
            this.createQTable();
        else
            this.loadQTable();
    }

    private void createQTable() {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                for (SteerControl.States state : SteerControl.States.values()) {
                    HashMap<String, Double> row =

                            new HashMap<>();
                    for (Object tmp : this.possibleActions) {
                        SteerControl.Actions action = (SteerControl.Actions) tmp;
                        row.put(action.name(), 0.0);
                    }
                    qTable.put(state.name(), row);
                }
                break;
            case ACCELERATION_CONTROL_SYSTEM:
                for (AccelControl.States state : AccelControl.States.values()) {
                    HashMap<String, Double> row = new HashMap<>();
                    for (Object tmp : this.possibleActions) {
                        AccelControl.Actions action = (AccelControl.Actions) tmp;
                        row.put(action.name(), 0.0);
                    }
                    qTable.put(state.name(), row);
                }
                break;
            case GEAR_CONTROL_SYSTEM:
                for (GearControl.States state : GearControl.States.values()) {
                    HashMap<String, Double> row = new HashMap<>();
                    for (Object tmp : this.possibleActions) {
                        GearControl.Actions action = (GearControl.Actions) tmp;
                        row.put(action.name(), 0.0);
                    }
                    qTable.put(state.name(), row);
                }
                break;
        }
    }


    private void loadQTable() {
        qTable.clear();
        try (Scanner file = new Scanner(new File(this.qTablePath))) {
            String[] rowLabels = file.nextLine().split(SEPARATOR);

            while (file.hasNextLine()) {
                String[] row = file.nextLine().split(SEPARATOR);
                String state = row[0];
                HashMap<String, Double> rowHash = new HashMap<>();
                for (int i = 1; i < row.length; i++) {
                    rowHash.put(rowLabels[i], Double.parseDouble(row[i]));
                }
                this.qTable.put(state, rowHash);
            }
        } catch (FileNotFoundException e) {
            System.out.println("ERROR!!! -> Could not load tablaQ from .csv file...");
            e.printStackTrace();
        }
    }

    public void saveTable() {
        try (PrintWriter file = new PrintWriter(this.qTablePath)) {
            file.write(" Q-TABLE ");
            file.write(SEPARATOR);
            switch (this.system) {
                case STEERING_CONTROL_SYSTEM:
                    for (Object tmp : this.possibleActions) {
                        SteerControl.Actions action = (SteerControl.Actions) tmp;
                        file.write(action.name());
                        file.write(SEPARATOR);
                    }
                    file.write("\n");
                    for (SteerControl.States state : SteerControl.States.values()) {
                        file.write(state.name());
                        file.write(SEPARATOR);
                        for (Object tmp : possibleActions) {
                            SteerControl.Actions action = (SteerControl.Actions) tmp;
                            String value = String.valueOf(this.qTable.get(state.name()).get(action.name()));
                            file.write(value);
                            file.write(SEPARATOR);
                        }
                        file.write("\n");
                    }
                    break;
                case ACCELERATION_CONTROL_SYSTEM:
                    for (Object tmp : this.possibleActions) {
                        AccelControl.Actions action = (AccelControl.Actions) tmp;
                        file.write(action.name());
                        file.write(SEPARATOR);
                    }
                    file.write("\n");
                    for (AccelControl.States state : AccelControl.States.values()) {
                        file.write(state.name());
                        file.write(SEPARATOR);
                        for (Object tmp : possibleActions) {
                            AccelControl.Actions action = (AccelControl.Actions) tmp;
                            String value = String.valueOf(this.qTable.get(state.name()).get(action.name()));
                            file.write(value);
                            file.write(SEPARATOR);
                        }
                        file.write("\n");
                    }
                    break;
                case GEAR_CONTROL_SYSTEM:
                    for (Object tmp : this.possibleActions) {
                        GearControl.Actions action = (GearControl.Actions) tmp;
                        file.write(action.name());
                        file.write(SEPARATOR);
                    }
                    file.write("\n");
                    for (GearControl.States state : GearControl.States.values()) {
                        file.write(state.name());
                        file.write(SEPARATOR);
                        for (Object tmp : possibleActions) {
                            GearControl.Actions action = (GearControl.Actions) tmp;
                            String value = String.valueOf(this.qTable.get(state.name()).get(action.name()));
                            file.write(value);
                            file.write(SEPARATOR);
                        }
                        file.write("\n");
                    }
                    break;
            }
        } catch (FileNotFoundException e) {
            System.out.println("ERROR!!! -> Could not save tableQ in .csv file...");
            e.printStackTrace();
        }
    }

    public Object update(Object lastState, Object currentState, Object actionPerformed, double reward) {
        this.lastState = lastState;
        if (lastState != null) {
            double oldQValue = this.getQValue(lastState, actionPerformed);
            double newQValue = oldQValue + LEARNING_RATE * (reward + DISCOUNT_FACTOR * this.getMaxQValue(currentState) - oldQValue);
            this.setQValue(lastState, actionPerformed, (Constants.round(newQValue, 8)));
        }
        return nextAction(currentState);
    }

    private double getQValue(Object stateO, Object actionO) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                SteerControl.States state = (SteerControl.States) stateO;
                SteerControl.Actions action = (SteerControl.Actions) actionO;
                return this.qTable.get(state.name()).get(action.name());
            case ACCELERATION_CONTROL_SYSTEM:
                AccelControl.States accelState = (AccelControl.States) stateO;
                AccelControl.Actions accelAction = (AccelControl.Actions) actionO;
                return this.qTable.get(accelState.name()).get(accelAction.name());
            case GEAR_CONTROL_SYSTEM:
                GearControl.States gearState = (GearControl.States) stateO;
                GearControl.Actions gearAction = (GearControl.Actions) actionO;
                return this.qTable.get(gearState.name()).get(gearAction.name());
        }
        return 0.0;
    }

    private void setQValue(Object stateO, Object actionO, double value) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                SteerControl.States state = (SteerControl.States) stateO;
                SteerControl.Actions action = (SteerControl.Actions) actionO;
                HashMap<String, Double> row = this.qTable.get(state.name());
                row.replace(action.name(), value);
                this.qTable.replace(state.name(), row);
                break;
            case ACCELERATION_CONTROL_SYSTEM:
                AccelControl.States accelState = (AccelControl.States) stateO;
                AccelControl.Actions accelAction = (AccelControl.Actions) actionO;
                HashMap<String, Double> accelRow = this.qTable.get(accelState.name());
                accelRow.replace(accelAction.name(), value);
                this.qTable.replace(accelState.name(), accelRow);
                break;
            case GEAR_CONTROL_SYSTEM:
                GearControl.States gearState = (GearControl.States) stateO;
                GearControl.Actions gearAction = (GearControl.Actions) actionO;
                HashMap<String, Double> gearRow = this.qTable.get(gearState.name());
                gearRow.replace(gearAction.name(), value);
                this.qTable.replace(gearState.name(), gearRow);
                break;
        }
    }

    private double getMaxQValue(Object stateO) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                SteerControl.States state = (SteerControl.States) stateO;
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                ArrayList<SteerControl.Actions> candidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    SteerControl.Actions action = (SteerControl.Actions) tmp;
                    double value = values.get(action.name());
                    if (maxValue < value) {
                        maxValue = value;
                        candidates.clear();
                        candidates.add(action);
                    } else if (maxValue == value) {
                        candidates.add(action);
                    }
                }
                int index = random.nextInt(candidates.size());
                SteerControl.Actions selected = candidates.get(index);
                return values.get(selected.name());
            case ACCELERATION_CONTROL_SYSTEM:
                AccelControl.States accelState = (AccelControl.States) stateO;
                double accelMaxValue = -Double.MAX_VALUE;
                HashMap<String, Double> accelValues = qTable.get(accelState.name());
                ArrayList<AccelControl.Actions> accelCandidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    AccelControl.Actions action = (AccelControl.Actions) tmp;
                    double value = accelValues.get(action.name());
                    if (accelMaxValue < value) {
                        accelMaxValue = value;
                        accelCandidates.clear();
                        accelCandidates.add(action);
                    } else if (accelMaxValue == value) {
                        accelCandidates.add(action);
                    }
                }
                int accelIndex = random.nextInt(accelCandidates.size());
                AccelControl.Actions accelSelected = accelCandidates.get(accelIndex);
                return accelValues.get(accelSelected.name());
            case GEAR_CONTROL_SYSTEM:
                GearControl.States gearState = (GearControl.States) stateO;
                double gearMaxValue = -Double.MAX_VALUE;
                HashMap<String, Double> gearValues = qTable.get(gearState.name());
                ArrayList<GearControl.Actions> gearCandidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    GearControl.Actions action = (GearControl.Actions) tmp;
                    double value = gearValues.get(action.name());
                    if (gearMaxValue < value) {
                        gearMaxValue = value;
                        gearCandidates.clear();
                        gearCandidates.add(action);
                    } else if (gearMaxValue == value) {
                        gearCandidates.add(action);
                    }
                }
                int gearIndex = random.nextInt(gearCandidates.size());
                GearControl.Actions gearSelected = gearCandidates.get(gearIndex);
                return gearValues.get(gearSelected.name());
        }
        return 0.0;
    }

    public Object nextAction(Object state) {
        if (random.nextDouble() > this.epsilon) return this.nextOnlyBestAction(state);
        else return this.getRandomAction();
    }

    private Object getRandomAction() {
        return this.possibleActions.get(random.nextInt(this.possibleActions.size()));
    }

    private Object getBestAction(Object stateO) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                SteerControl.States state = (SteerControl.States) stateO;
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                ArrayList<SteerControl.Actions> candidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    SteerControl.Actions action = (SteerControl.Actions) tmp;
                    double value = values.get(action.name());
                    if (maxValue < value) {
                        maxValue = value;
                        candidates.clear();
                        candidates.add(action);
                    } else if (maxValue == value) {
                        candidates.add(action);
                    }
                }
                int index = random.nextInt(candidates.size());
                return candidates.get(index);
            case ACCELERATION_CONTROL_SYSTEM:
                AccelControl.States accelState = (AccelControl.States) stateO;
                double accelMaxValue = -Double.MAX_VALUE;
                HashMap<String, Double> accelValues = qTable.get(accelState.name());
                ArrayList<AccelControl.Actions> accelCandidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    AccelControl.Actions action = (AccelControl.Actions) tmp;
                    double value = accelValues.get(action.name());
                    if (accelMaxValue < value) {
                        accelMaxValue = value;
                        accelCandidates.clear();
                        accelCandidates.add(action);
                    } else if (accelMaxValue == value) {
                        accelCandidates.add(action);
                    }
                }
                int accelIndex = random.nextInt(accelCandidates.size());
                return accelCandidates.get(accelIndex);
            case GEAR_CONTROL_SYSTEM:
                GearControl.States gearState = (GearControl.States) stateO;
                double gearMaxValue = -Double.MAX_VALUE;
                HashMap<String, Double> gearValues = qTable.get(gearState.name());
                ArrayList<GearControl.Actions> gearCandidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    GearControl.Actions action

                            = (GearControl.Actions) tmp;
                    double value = gearValues.get(action.name());
                    if (gearMaxValue < value) {
                        gearMaxValue = value;
                        gearCandidates.clear();
                        gearCandidates.add(action);
                    } else if (gearMaxValue == value) {
                        gearCandidates.add(action);
                    }
                }
                int gearIndex = random.nextInt(gearCandidates.size());
                return gearCandidates.get(gearIndex);
        }
        return null;
    }

    public Object nextOnlyBestAction(Object stateO) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                SteerControl.States state = (SteerControl.States) stateO;
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                Object theBest = this.getRandomAction();
                for (Object tmp : this.possibleActions) {
                    SteerControl.Actions action = (SteerControl.Actions) tmp;
                    double value = values.get(action.name());
                    if (maxValue < value) {
                        maxValue = value;
                        theBest = action;
                    }
                }
                return theBest;
            case ACCELERATION_CONTROL_SYSTEM:
                AccelControl.States accelState = (AccelControl.States) stateO;
                double accelMaxValue = -Double.MAX_VALUE;
                HashMap<String, Double> accelValues = qTable.get(accelState.name());
                Object accelTheBest = this.getRandomAction();
                for (Object tmp : this.possibleActions) {
                    AccelControl.Actions action = (AccelControl.Actions) tmp;
                    double value = accelValues.get(action.name());
                    if (accelMaxValue < value) {
                        accelMaxValue = value;
                        accelTheBest = action;
                    }
                }
                return accelTheBest;
            case GEAR_CONTROL_SYSTEM:
                GearControl.States gearState = (GearControl.States) stateO;
                double gearMaxValue = -Double.MAX_VALUE;
                HashMap<String, Double> gearValues = qTable.get(gearState.name());
                Object gearTheBest = this.getRandomAction();
                for (Object tmp : this.possibleActions) {
                    GearControl.Actions action = (GearControl.Actions) tmp;
                    double value = gearValues.get(action.name());
                    if (gearMaxValue < value) {
                        gearMaxValue = value;
                        gearTheBest = action;
                    }
                }
                return gearTheBest;
        }
        return null;
    }

    public void saveStatistics(String newResults) {
        this.saveStatistics(STATISTICS_TEST_PATH, newResults);
    }

    private void saveStatistics(String filePath, String newResults) {
        List<String> content = new ArrayList<>();
        try (Scanner file = new Scanner(new File(filePath))) {
            while (file.hasNextLine()) {
                content.add

                        (file.nextLine());
            }
        } catch (FileNotFoundException e) {
            System.out.println("ERROR!!! -> Could not load statistics from .csv file...");
            e.printStackTrace();
        }
        try (PrintWriter file = new PrintWriter((filePath))) {
            for (String line : content) {
                file.write(line + "\n");
            }
            file.write(newResults + "\n");
        } catch (FileNotFoundException e) {
            System.out.println("ERROR!!! -> Could not save statistics in .csv file...");
            e.printStackTrace();
        }
    }

    public void decreaseEpsilon() {
        this.epsilon -= EPSILON_DECAY;
    }
}