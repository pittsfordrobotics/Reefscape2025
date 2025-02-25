package frc.robot.subsystems.objectiveTracker;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ObjectiveSelecterIONetworkTables implements ObjectiveSelectorIO {
    private final String NETWORK_TABLE_NAME = "objectiveTracker";
    private final String CURRENT_INDEX_NAME = "currentIndex";
    private final String REEF_SIDE_NAME = "reefSide";
    private final IntegerPublisher currentIndexPublisher;
    private final IntegerPublisher reefSidePublisher;
    
    private int lastIndex = 0;
    private int lastReefSide = 0;

    public ObjectiveSelecterIONetworkTables() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(NETWORK_TABLE_NAME);
        currentIndexPublisher = table.getIntegerTopic(CURRENT_INDEX_NAME).publish();
        reefSidePublisher = table.getIntegerTopic(REEF_SIDE_NAME).publish();
        currentIndexPublisher.set(lastIndex);
    }

    @Override
    public void updateInputs(ObjectiveSelectorInputs inputs) {
        // Nothing really to read back from network tables at this point.
        // Just return the last index that was set.
        inputs.selectedIndex = lastIndex;
        inputs.reefSide = lastReefSide;
    }

    @Override
    public void setIndex(int newIndex) {
        currentIndexPublisher.set(newIndex);
        lastIndex = newIndex;
    }

    @Override
    public void setReefSide(int newReefSide) {
        reefSidePublisher.set(newReefSide);
        lastReefSide = newReefSide;
    }
}
