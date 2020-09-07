from modules.Map import Map_Obj
import modules.AStar as AStar

map_obj = Map_Obj(task=1)

m, _ = map_obj.get_maps()
s = map_obj.get_start_pos()
g = map_obj.get_goal_pos()

AStar.solve(m, s, g)