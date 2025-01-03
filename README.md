# PRM* Path Planning Algorithm

This project implements the Probabilistic Roadmap (PRM*) algorithm for path planning in a 2D environment with obstacles.

## About PRM*
Probabilistic Roadmap (PRM*) is a graph-based path planning algorithm widely used in robotics and motion planning. It involves sampling random points in a configuration space, connecting them based on a distance threshold, and ensuring the connections are collision-free. PRM* is an improved version of PRM that ensures asymptotic optimality, meaning that as the number of samples increases, the solution converges to the optimal path.

Key features of PRM*:
- Efficient for high-dimensional spaces.
- Ensures asymptotic optimality.
- Suitable for static environments with known obstacles.

For more details, you can read the [Wikipedia article on Probabilistic Roadmap Methods](https://en.wikipedia.org/wiki/Probabilistic_roadmap).

## Features
- Generates random obstacles in a 2D space.
- Creates a graph of randomly sampled nodes using PRM*.
- Finds the shortest path between a start and goal point using Dijkstra's algorithm.
- Visualizes the map, graph, and shortest path.

## Requirements
- Python 3.8 or later
- Required Libraries: `numpy`, `matplotlib`, `networkx`


