from python.curling import curling, IDX_TO_COLOR
from olympics_engine.main import getgame
from python.generator import create_scenario
import timeit

# import ptvsd
# ptvsd.enable_attach(address=('localhost', 5678), redirect_output=True)
# print('Now is a good time to attach your debugger: Run: Python: Attach')
# ptvsd.wait_for_attach()

a = curling(create_scenario("curling"))
b = getgame("curling")
action=[[200, 0], [10, 10]]
def cpp_run()->None:
    a.reset()
    for _ in range(300): 
        a.step(action)
def python_run()->None:
    b.reset()
    for _ in range(300): 
        b.step(action)
print(timeit.Timer(cpp_run).timeit(number=10))
print(timeit.Timer(python_run).timeit(number=10))
