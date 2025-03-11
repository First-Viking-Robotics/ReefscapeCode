class GameState:
    def __init__(self):
        self.disabled = 0
        self.auto = 1
        self.teleop = 2
        self.test = 3
        self.gameState = 0
    
    def getGameState(self):
        return self.gameState
