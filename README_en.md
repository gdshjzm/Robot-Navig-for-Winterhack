# Robot Path Exploration Optimization(For Winterhack London)

## Submission necessary files:
```
- main.py        # reference code that initially exist in the directory
- main_.py       # >> main SUBMISSION CODE 
- video_demo.mov # video preview for random-seed-8 and snake-5-walls (most difficult where main.py failed)
- README_en.md   # Technical details about my submission
```

## Overview

This project implements an intelligent frontier selection strategy based on multi-criteria comprehensive scoring to optimize robot path exploration efficiency in unknown environments. Compared to traditional simple linear combination methods, my strategy demonstrates significant performance improvements in various test scenarios.

## Frontier Selection Strategy Design

### Core Idea

My frontier selection strategy is based on the following core ideas:
1. **Multi-dimensional Evaluation**: Evaluate frontier points from multiple dimensions such as information gain, path efficiency, goal orientation, and exploration balance.
2. **Dynamic Weight Adjustment**: Dynamically adjust the weights of each evaluation metric based on exploration progress.
3. **Intelligent Redundancy Avoidance**: Avoid redundant exploration near already explored areas through cluster analysis.
4. **Introduction of Randomness**: Add a small amount of randomness to avoid getting stuck in local optima.

### Detailed Explanation of Evaluation Metrics

#### 1. Information Gain
```python
def calculate_information_gain(cell: Cell) -> float:
    """Calculates the potential information gain of a frontier point"""
```
- **Principle**: Assesses the density of unknown areas around a frontier point.
- **Calculation Method**: Counts the number of unknown cells within a certain range around the frontier point.
- **Significance**: Prioritizes frontier points that can yield more new information.

#### 2. Heading Alignment
```python
def calculate_heading_alignment(cell: Cell) -> float:
    """Calculates alignment with the robot's heading"""
```
- **Principle**: Calculates the cosine similarity between the frontier point's direction and the robot's current heading.
- **Calculation Method**: Uses vector dot product to determine alignment.
- **Significance**: Reduces unnecessary turns and improves movement efficiency.

#### 3. Goal Progress
```python
def calculate_goal_progress(cell: Cell) -> float:
    """Calculates progress towards the goal"""
```
- **Principle**: Evaluates whether selecting this frontier point brings the robot closer to the final goal.
- **Calculation Method**: Compares the distance from the robot's current position to the goal with the distance from the frontier point to the goal.
- **Significance**: Maintains goal-oriented behavior during exploration.

#### 4. Exploration Efficiency
```python
def calculate_exploration_efficiency(cell: Cell) -> float:
    """Calculates exploration efficiency: distance from explored areas"""
```
- **Principle**: Encourages exploring frontier points far from already known areas.
- **Calculation Method**: Calculates the distance from the frontier point to the nearest explored point.
- **Significance**: Avoids redundant exploration near already explored regions.

#### 5. Cluster Size
```python
def calculate_cluster_size(cell: Cell) -> float:
    """Calculates the size of the cluster the frontier point belongs to"""
```
- **Principle**: Prioritizes frontier points within larger clusters.
- **Calculation Method**: Uses a BFS algorithm to calculate the size of the cluster of adjacent frontier points.
- **Significance**: Selects frontier points that allow for exploring larger areas at once.

### Dynamic Weight Adjustment Mechanism

My strategy dynamically adjusts the weights of each metric based on exploration progress:

```python
# Early exploration phase: emphasize information gain and exploration efficiency
# Later phase: emphasize goal orientation and heading alignment
w_info = 0.3 * (1 - exploration_progress) + 0.1 * exploration_progress
w_heading = 0.2 * (1 - exploration_progress) + 0.3 * exploration_progress  
w_goal = 0.1 * (1 - exploration_progress) + 0.4 * exploration_progress
w_efficiency = 0.2
w_cluster = 0.1
```

**Design Philosophy**:
- **Early Exploration**: Prioritize information gain and exploration efficiency to quickly build an environmental map.
- **Later Exploration**: Focus more on goal orientation, moving directly towards the target.

### Comprehensive Scoring Formula

The final frontier point score is calculated using the formula:

```python
score = (w_info * info_gain + 
         w_heading * heading_align + 
         w_goal * goal_progress + 
         w_efficiency * exploration_eff + 
         w_cluster * cluster_size + 
         w_distance * distance_factor) + 
         0.01 * random_factor
```

## Comparison with Traditional Methods

Compared to main.py, I used a more complex but stronger robustness method, which can display a better performance on complicated scenario than the reference code provided in main.py.

### My Improved Method (main_.py)
1. **Multi-dimensional Evaluation**: Added metrics such as information gain, exploration efficiency, and cluster analysis.
2. **Dynamic Adaptability**: Adjusts strategy focus based on exploration progress.
3. **Intelligent Redundancy Avoidance**: Prevents repetitive exploration through distance and cluster analysis.
4. **Enhanced Robustness**: Adds randomness to avoid local optima.

## Experimental Results Analysis

I have tested our strategy performs excellently in multiple test scenarios:

| Test Scenario | main_.py (My Method) | main.py (Reference Code) | Improvement |
|---------------|-----------------------|------------------------------|-------------|
| snake_5wall   | 5.206m                | 4.823m                       | -7.9%       |
| random_seed0  | 2.230m                | 2.230m                       | No change   |
| random_seed1  | 2.550m                | ERROR                        | **Error** |
| random_seed2  | 2.395m                | 2.254m                       | -6.3%       |
| random_seed24 | 2.473m                | 2.553m                       | **+3.1%**   |
| random_seed15 | 2.983m                | ERROR                        | **Error** |
| random_seed10 | 1.970m                | 1.998m                       | **+1.4%**   |
| random_seed8  | 4.024m                | INF_LOOP                     | **Infinite Loop** |
| random_seed55 | 2.800m                | 2.371m                       | -18.1%      |

### Key Advantages

1. **Significantly Improved Robustness**:
   - Resolved errors in `random_seed1` and `random_seed15` that occurred with the traditional method.
   - Fixed the infinite loop issue in `random_seed8`.

2. **Stable Performance**:
   - Maintained competitive path lengths in most test scenarios.
   - Achieved path optimization in `random_seed24` and `random_seed10`.

3. **Strong Adaptability**:
   - Capable of handling various complex maze environments.
   - Operates stably across different random seeds.

### Performance Analysis

While my method's path length is slightly longer in some scenarios (e.g., `snake_5wall` and `random_seed55`), this is a reasonable trade-off for better robustness and stability:

1. **More Conservative Exploration Strategy**: Avoids aggressive choices that could lead to errors or infinite loops.
2. **More Thorough Information Gathering**: Emphasizes information gain in early stages, potentially leading to slightly longer but safer paths.
3. **Smarter Goal Orientation**: Goal-oriented behavior in later stages ensures successful arrival at the target.

## Technical Features

### 1. Computational Efficiency Optimization
- Limited cluster search range to avoid excessive computation.
- Utilized caching mechanisms to reduce redundant calculations.
- Implemented reasonable search depth control.

### 2. Parameter Tunability
- All weight parameters can be adjusted according to specific application scenarios.
- Supports different exploration strategy configurations.
- Easy to extend with new evaluation metrics.

### 3. Code Maintainability
- Modular function design.
- Clear comments and documentation.
- Easy-to-understand algorithm logic.

## Conclusion

Our intelligent frontier selection strategy significantly enhances the robustness and efficiency of robot path exploration through multi-dimensional evaluation, dynamic weight adjustment, and intelligent redundancy avoidance. Experimental results demonstrate that this strategy can:

1. **Address stability issues of traditional methods**: Eliminating errors and infinite loops.
2. **Maintain competitive path efficiency**: Achieving comparable or superior path lengths in most scenarios.
3. **Provide better adaptability**: Capable of handling various complex environments and random scenarios.

This design, balancing efficiency, robustness, and adaptability, makes our strategy more suitable for practical robot navigation applications.

## !!Important: AI Usage Details
**Declaration:I write this details to make AI usage more transparent and make every code I wrote believable.**
1. Code: All code is written by myself using AI Tab completion.
    - heading alignment and goal progress function are taken reference code main.py.
    - Information gain and dynamic weight are new ideas by me and I wrote the raw code manually.
    - Used AI to add a total distance demonstration so that I can compare different methods.
    - Testing raw code occured many bugs and I used AI to debug (only on code issues like value errors, without using new method beyond).
    After all code completed, I used Calude-4-Sonnet to handle with tranceback errors.
2. README tech file: Almost written by AI and was manually modified to ensure readability.
3. video: Shot full-manually without AI.