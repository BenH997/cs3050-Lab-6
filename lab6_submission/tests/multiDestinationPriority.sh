echo "Normal:"
python3 ../src/route_planner.py ../src/data/timeWindowNodes.csv ../src/data/edges.csv 1 1 multiDestinationPriority ../src/data/priorityTest1.csv 0.20
printf "\n"

echo "Visits lower priority first:"
python3 ../src/route_planner.py ../src/data/timeWindowNodes.csv ../src/data/edges.csv 1 1 multiDestinationPriority ../src/data/priorityTest2.csv 0.20
printf "\n"

echo "Takes longer path based on threshold:"
python3 ../src/route_planner.py ../src/data/timeWindowNodes.csv ../src/data/edges.csv 1 1 multiDestinationPriority ../src/data/priorityTest2.csv 1.0
printf "\n"

echo "Unreachable position:"
python3 ../src/route_planner.py ../src/data/timeWindowNodes.csv ../src/data/edges.csv 1 1 multiDestinationPriority ../src/data/priorityTest3.csv 0.20