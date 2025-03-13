import ntcore


class NetworkingAssistant:
    def __init__(self):
        self.instance = ntcore.NetworkTableInstance.getDefault()
        self.mainTable = self.instance.getTable("VakaTable")
        self.limelightTable = self.instance.getTable("limelight-vaka")