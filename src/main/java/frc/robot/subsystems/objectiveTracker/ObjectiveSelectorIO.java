package frc.robot.subsystems.objectiveTracker;

public interface ObjectiveSelectorIO {
    class ObjectiveSelectorInputs {
        public int selectedIndex = 0;
    }

    enum MoveDirection {
        UP, DOWN, LEFT, RIGHT
    }

    default void updateInputs(ObjectiveSelectorInputs inputs) {}

    default void setIndex(int newIndex) {}
}
