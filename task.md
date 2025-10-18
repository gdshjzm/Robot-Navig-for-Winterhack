### 3. 编程挑战：未知环境中的前沿探索
未知迷宫环境中的自主导航是本次指定的挑战内容，这是任何智能移动机器人都应具备的一项基本能力。

与已知环境导航不同（在已知环境中，机器人会利用完整的地图信息规划一条通往预定义目标的路径），未知环境探索要求机器人在移动过程中同时发现环境并确定可行路线。

本次挑战的核心在于开发一种决策逻辑，使机器人能够选择有意义的探索目标（即所谓的“前沿”）。前沿是已知自由空间与未探索未知空间之间的边界，这一原理是“基于前沿的探索”（自主机器人领域的关键方法）的基础。

#### 3.1 编程任务
首先，请打开main.py文件，并找到如下所示的函数模板（见图2）：
```python
def determine_frontier_path(state, test=None):
    """
    确定并设置机器人在未知环境中导航的前沿路径。
    
    （涉及）朝向目标的前进进度以及距离。
    参数：
        state (SimulationState)：当前仿真状态，包含机器人位姿、世界信息、目标以及其他导航参数。
    返回：
        None。通过设置frontier_goal和path属性在原地修改状态。
            "frontier_goal"是一个单元格，表示所选的待探索前沿。
            "path"是一个单元格列表，表示到达"frontier_goal"的规划路径。
    """
    # 1. 检查当前前沿是否与总体目标匹配。该函数需执行以下关键步骤：
    # 2. （涉及）
    # - 基于机器人当前朝向的朝向分配
    # - 朝向目标的前进进度
    # - 与机器人的距离
    # 3. 为所选前沿规划路径。
    # 4. 更新：
    # - 与最终目标的接近程度
    # - 将规划的路径更新到state.path（以到达所选前沿）
    # - 将所选前沿更新到state.frontier_goal
    
    if state.frontier_goal == state.goal["cell"]:
        print("找到与目标匹配的前沿")
        return state.frontier_goal
    else:
        frontiers, distances = detect_frontiers(state)
        goal_cell = state.goal["cell"]
        state.frontier_candidates = frontiers
        state.frontier_distances = distances
        
        # -----开始：由候选人完成-----
        # 此处填写你的前沿选择逻辑
        # ------结束：由候选人完成------
        
        state.frontier_goal = best_frontier_cell
        
        return
```

图2. determine_frontier_path()函数模板

你的目标是在提供的Python代码中完成determine_frontier_path()函数。该函数负责利用detect_frontiers()函数生成的信息，选择机器人应导航至的下一个前沿目标。detect_frontiers()程序会分析占据栅格地图（OGM），以识别所有可达的前沿单元格——即那些处于自由空间且与未知空间相邻、同时可从机器人当前位置到达的单元格。

你的任务是实现前沿选择逻辑，以确定应将这些检测到的前沿中的哪一个选为机器人的下一个导航目标。一旦你的算法确定了最佳前沿单元格，请将其赋值给：

state.frontier_goal = best_frontier_cell

其余的系统组件将自动执行以下步骤：
- 利用pose_to_cell(state.world, state.pose)计算机器人当前所在的单元格
- 利用plan_unknown_world(state, start_cell, goal_cell)规划路径（该函数基于当前OGM，采用A*算法在当前单元格与目标前沿单元格之间规划路径）
- 更新state.path，用于导航执行和可视化

你的实现应确保机器人能够高效探索，在逐步揭示迷宫未知区域的同时，向最终目标单元格（state.goal["cell"]）推进。设计良好的前沿选择逻辑需平衡覆盖效率、路径成本和目标导向探索，以此体现你对自主导航和基于决策的控制的理解。

#### 3.2 有用的函数和数据参考
determine_frontier_path()函数运行在一个完全初始化的仿真环境中，所有相关数据和辅助函数均可通过state对象以及util.py中导入的工具直接获取。下表总结了可直接用于完成实现的关键函数、变量和数据结构。

| 类别         | 函数/变量                          | 描述                                                                 |
|--------------|-----------------------------------|----------------------------------------------------------------------|
| 前沿检测     | frontiers, distances = detect_frontiers(state) | 从当前占据栅格地图（OGM）中识别所有可达的前沿单元格（与未知区域相邻的自由单元格）。返回前沿单元格列表以及从机器人出发的广度优先搜索（BFS）距离字典。 |
| 前沿选择输出（由你设置） | state.frontier_candidates = frontiers | 记录检测到的前沿列表，用于可视化（在图表上显示为橙色方块）。 |
|              | state.frontier_distances = distances | 存储BFS距离，用于调试或显示。 |
|              | state.frontier_goal = best_frontier_cell | 所选的前沿目标单元格（显示为金色星标）。你必须在你的逻辑块中设置该值。 |
| 坐标转换     | pose_to_cell(state.world, state.pose) | 将机器人当前位姿（x, y，单位：米）转换为对应的地图单元格坐标（cx, cy）。 |
|              | cell_center(cell, cell_size)    | 返回指定单元格中心的笛卡尔坐标（单位：米），适用于几何计算和距离计算。 |
| 机器人与地图状态 | state.pose["x"], state.pose["y"], state.pose["theta"] | 机器人当前位置（单位：米）和朝向角度（单位：弧度）。 |
|              | state.goal["cell"]              | 迷宫的最终目标单元格，用于引导探索方向。 |

#### 3.3 核心设计原则
在determine_frontier_path()函数中实现选择逻辑时，请考虑以下设计原则：
- 渐进探索——促使机器人持续进入新区域，而非在之前访问过的区域之间往复。
- 目标导向行为——融入对最终目标单元格（state.goal["cell"]）的认知，以便在可行的情况下，使探索向该方向发展。
- 效率与稳定性——通过评估机器人当前朝向与潜在前沿方向之间的关系，减少不必要的转向或过长的路径。
- 鲁棒性——处理未检测到有效前沿的情况（例如，所有区域已探索或机器人陷入困境）。在此类情况下，机器人应保持安全的备用行为。

#### 3.4 测试与验证建议
- 界面视觉反馈（如图1所示）：观察金色星标（所选前沿）和橙色方块（候选前沿）的实时更新情况。若机器人能持续向前推进，则表明其行为表现良好。
- 日志记录与分析——查看CSV日志（diagnostic.csv）中的前沿数量、所选目标坐标以及随时间变化的路径长度，以量化探索性能。
- 迷宫布局——在随机（Random）和蛇形（Snake）两种布局下测试你的解决方案，以确保其在不同几何结构下的鲁棒性。