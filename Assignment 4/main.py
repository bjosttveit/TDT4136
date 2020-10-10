import Assignment

#csp = Assignment.create_map_coloring_csp()
csp = Assignment.create_sudoku_csp('hard.txt')

result = csp.backtracking_search()

if result:
    #Assignment.print_sudoku_solution(result)
    print(result)
else:
    print("Failed.")