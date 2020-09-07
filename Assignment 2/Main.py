import sys
from modules.Map import Map_Obj
import modules.AStar as AStar

def main(task):
    map_obj = Map_Obj(task=task)

    intmap, strmap = map_obj.get_maps()
    start = map_obj.get_start_pos()
    goal = map_obj.get_goal_pos()

    path = AStar.solve(intmap, start, goal)

    for p in path:
        strmap[p[0]][p[1]] = ' P '

    map_obj.show_map(map=strmap)

if __name__ == "__main__":
    main(int(sys.argv[1]))