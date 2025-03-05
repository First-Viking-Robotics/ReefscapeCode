import ntcore



class Network:
    def __init__(self):
        self.instance = ntcore.NetworkTableInstance.getDefault()
        self.mainTable = self.instance.getTable("Viking")
        self.topics = []
    
    def setValue(self, topic: str, value: any):
        self.mainTable.putValue(topic, value)
    
    def getValue(self, topic: str) -> any:
        self.mainTable.getValue(topic)
