from classic_control_envpool import _OlympicsBase

class OlympicsBase(_OlympicsBase):
    def __init__(self,map) -> None:
        super().__init__("/home/quangr/olympics_engine/src/testhelper/scenario.json")
        self.agent_record = []
        self.view_setting = map["view"]
        self.map=map
    def generate_map(self,map) -> None:
        self._generate_map()
