package frc.lib.ruckig;

public class PositionResult {
    public Profile profile;
    public boolean has_found = false;

    public PositionResult(Profile profile, boolean has_found) {
        this.profile = profile;
        this.has_found = has_found;
    }
}
