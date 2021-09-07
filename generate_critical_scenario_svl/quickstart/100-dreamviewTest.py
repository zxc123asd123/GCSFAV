import os
import lgsvl

SIMULATOR_HOST = os.environ.get("SIMULATOR_HOST", "127.0.0.1")
SIMULATOR_PORT = int(os.environ.get("SIMULATOR_PORT", 8181))
BRIDGE_HOST = os.environ.get("BRIDGE_HOST", "127.0.0.1")
BRIDGE_PORT = int(os.environ.get("BRIDGE_PORT", 9090))

sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)
if sim.current_scene == "SanFrancisco":
    sim.reset()
else:
    sim.load("SanFrancisco")

spawns = sim.get_spawn()

state = lgsvl.AgentState()
# state.transform = spawns[0]

initEvPos = lgsvl.Vector(766,10,5)
state.transform = sim.map_point_on_lane(initEvPos)
# (773.476,10.2,5.1111)four
# (763,10,5) one
# (769,10,5)three
#(766,10,5) two

print("start::::::::::11111111",state.transform)
print("start::::::::::",state.transform.position.x,state.transform.position.y,state.transform.position.z)


ego = sim.add_agent(lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular, lgsvl.AgentType.EGO, state)
ego.connect_bridge(BRIDGE_HOST, BRIDGE_PORT)

# Dreamview setup
dv = lgsvl.dreamview.Connection(sim, ego, BRIDGE_HOST)
dv.set_hd_map('SanFrancisco')
dv.set_vehicle('Lincoln2017MKZ_LGSVL')
modules = [
    'Localization',
    # 'Perception',
    'Transform',
    'Routing',
    'Prediction',
    'Planning',
    # 'Camera',
    # 'Traffic Light',
    'Control'
]
destination = spawns[0].destinations[0]
# destination = sim.map_point_on_lane(initEvPos).destinations[0]
print("destination",destination.position.x,destination.position.y)
dv.setup_apollo(destination.position.x, destination.position.z, modules)


while True:
    sim.run(3)
    s = ego.state.transform
    print("222222222222222222222222222222222222222", s)

