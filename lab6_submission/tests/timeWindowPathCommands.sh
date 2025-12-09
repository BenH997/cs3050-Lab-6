echo "A feasible path:"
python3 ../src/route_planner.py ../src/data/timeWindowNodes.csv ../src/data/edges.csv 3 1 dijkstraTimeWindows
printf "\n"

echo "An infeasible path:"
python3 ../src/route_planner.py ../src/data/timeWindowNodes.csv ../src/data/edges.csv 3 7 dijkstraTimeWindows
printf "\n"

echo "A case where the shortest distance path violates constraints:"
python3 ../src/route_planner.py ../src/data/timeWindowNodes.csv ../src/data/edges.csv 3 5 dijkstraTimeWindows