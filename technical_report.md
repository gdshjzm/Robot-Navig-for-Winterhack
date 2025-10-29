# WinterHack Submission Files

Supplimentary details:
```
- main_hainan.py      # My main submission file.
- test_random_seeds   # A FULLY AI-written quick testing file.
- video_pre.mp4       # A video demonstration of the submission.
- technical_report.md # Technical report of the submission.
```

## AI Usage Assertation
I used Auto-cue completion command and Also the Here's my detail AI-Usage：
- main_hainan.py: The file is constructed by comating AI, used Trae (An IDE) AI auto complete and GPT-5-high to optimize the code, the main function is constructed by me and GPT-5 is used only for parameter finetuning and code debugging.
- test_random_seeds: Fully AI-written to test quickly.
- video_pre.mp4: A video demonstration of the submission.
- Technical_report: 70% AI generated and 30% manually refined.
## Technical report

The core of the navigation logic is a sophisticated frontier-based exploration strategy designed for autonomous operation in unknown environments. The primary goal is to guide the robot to a specified destination while simultaneously mapping its surroundings.

### Frontier Selection Strategy

When the final goal is not immediately accessible or the path is obstructed, the robot switches to an exploration mode. It identifies "frontiers," which are cells at the boundary of known and unknown territory. The selection of the next frontier to explore is not random; it is determined by a multi-faceted scoring function desgned by me that evaluates each candidate frontier cell.

The `composite_score` for each cell is calculated as a weighted sum of several heuristic features:

```
score = (2.0 * alignment) + (2.8 * unknown_gain) + (1.4 * progress_to_goal) - (0.6 * path_cost) - (0.7 * turn_cost) - (0.2 * distance_to_goal)
```

### Heuristic Components Explained:

*   **Alignment (`align`):** This feature encourages the robot to move forward, rewarding frontiers that are directly ahead and penalizing those behind it. This promotes efficient, straight-line movement where possible.
*   **Unknown Gain (`unknown_gain`):** With the highest positive weight (2.8), this is the primary driver for exploration. It measures the amount of unexplored space adjacent to a frontier cell. By maximizing this gain, the robot prioritizes mapping new areas.
*   **Progress to Goal (`progress`):** This encourages the robot to make headway towards the final destination, even while exploring. It measures the reduction in distance to the final goal.
*   **Path Cost (`bfs_cost`):** This penalizes frontiers that are far away or difficult to reach from the robot's current position, based on the path length calculated by a pathfinding algorithm. This ensures the robot takes efficient local paths.
*   **Turn Cost (`turn_cost`):** This penalizes frontiers that would require a large turn, promoting smoother trajectories and reducing time spent rotating in place.
*   **Distance to Goal (`dist_goal`):** A minor penalty for frontiers that are far from the final goal, acting as a tie-breaker and keeping the robot generally oriented towards its ultimate destination.

### Behavioral Refinements

To improve stability and efficiency, the algorithm incorporates several additional rules:

1.  **Dithering Reduction:** Frontiers that are too close to the robot's current position are filtered out to prevent small, inefficient back-and-forth movements.
2.  **Hysteresis:** The previously chosen frontier goal is retained if its score is still reasonably close to the new best candidate's score. This prevents the robot from rapidly oscillating between two equally attractive options, leading to more decisive movement.
3.  **Candidate Filtering:** Frontiers located behind the robot are generally ignored unless they offer a very high `unknown_gain`, ensuring the robot does not waste time backtracking unless it's for significant exploration value.

This hybrid approach, combining goal-seeking with a sophisticated, heuristic-driven exploration strategy, allows the robot to navigate and map unknown environments efficiently and robustly.

## Test Results
I have tested other kind of random seeds, and the results are as follows:

| seed | success | steps | distance_m | avg_step_m | elapsed_s |
|------|---------|-------|------------|------------|-----------|
| 1    | True    | 23    | 2.300      | 0.100      | 1.718     |
| 16   | True    | 27    | 2.700      | 0.100      | 2.568     |
| 9    | True    | 25    | 2.500      | 0.100      | 2.985     |
| 6    | True    | 51    | 5.100      | 0.100      | 6.618     |
| 15   | True    | 27    | 2.700      | 0.100      | 3.634     |
| 87   | True    | 23    | 2.300      | 0.100      | 2.129     |
| 47   | True    | 27    | 2.700      | 0.100      | 2.479     |
| 28   | True    | 27    | 2.700      | 0.100      | 2.600     |
| 463  | True    | 44    | 4.400      | 0.100      | 6.054     |
| 2    | False   |  X    | X          | X          | X         |

I noticed that for some examples, it doesn't work. This is because the parameters in the composite scoring function are finely tuned and represent a delicate balance between exploration and goal-seeking. For certain map layouts generated by specific random seeds, the robot can encounter local minima. In these situations, no available frontiers meet the criteria to be considered a better choice, or the robot might oscillate between a few suboptimal choices, effectively getting stuck. The current weights are optimized for a general case but may not be robust enough for all possible environmental configurations. Further tuning or an adaptive parameter strategy could improve performance on these challenging seeds.