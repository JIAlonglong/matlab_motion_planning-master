# matlab_motion_planning
## 路径规划算法
### dijkstra算法：
Dijkstra算法是一种用于图中寻找最短路径的算法，它可以应用于有向图或无向图。该算法通过不断更新起点到各个顶点的最短路径来找到最终的最短路径。Dijkstra算法的时间复杂度为O(V^2)，其中V为顶点数，但可以通过优先队列实现最小堆来优化时间复杂度。
### RRT（Rapidly-exploring Random Tree）算法
是一种适用于高维空间的路径规划算法，它通过随机采样和不断扩展树形结构来搜索路径。RRT算法适用于具有复杂空间结构的环境，并且在机器人导航和运动规划中有着广泛的应用。
### Hybrid-A* 
将A与RRT（快速随机树）、Dijkstra算法或其他启发式方法结合，从而在探索效率和路径质量之间取得平衡，规划效果相较于前两种，有很大进步。
## 路径跟踪算法
### 局部规划器-Pure Pursuit算法
