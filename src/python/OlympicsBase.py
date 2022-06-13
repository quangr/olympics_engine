import classic_control_envpool
from classic_control_envpool import _OlympicsBase
IDX_TO_COLOR = {
    0: 'light green',
    1: 'green',
    2: 'sky blue',
    3: 'yellow',
    4: 'grey',
    5: 'purple',
    6: 'black',
    7: 'red',
    8: 'blue'
}

@property
def color(self):
    return IDX_TO_COLOR[int(self._color)]


@property
def type(self):
    if self.is_ball:
        return "ball"
    else:
        return "agent"


classic_control_envpool.agent_t.color = color
classic_control_envpool.agent_t.type = type


class OlympicsBase(_OlympicsBase):
    def __init__(self,map) -> None:
        super().__init__("/home/quangr/olympics_engine/src/python/scenario.json")
        self.agent_record = []
        self.view_setting = map["view"]
        self.map=map
    def generate_map(self,map) -> None:
        self._generate_map()
