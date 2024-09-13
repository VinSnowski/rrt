import numpy as np
import matplotlib.pyplot as plt

from typing import Tuple, List, Any

def parse_graph_file(filename: str) -> Tuple[List[Any]]:
    """Read the graph text file containing lines that define vectors in the following format for each line:
    x_head,y_head-x_tail,y_tail\n.
    
    Returns:
        Tuple[List[Any]]: Lists containins the heads and tails of the vectors."""
    with open(filename, 'r') as f:
        content = f.read().strip()
    pairs = (content.split('\n'))[:-1]

    heads = []
    tails = []

    for pair in pairs[:-1]: 
        points = pair.split('-')
        
        mapping = []
        for point in points:
            x, y = map(float, point.split(','))
            mapping.append(np.array([x,y]))
        heads.append(mapping[0])
        tails.append(mapping[1])

    return heads, tails


def parse_path_file(filename: str) -> Tuple[List[Any]]:
    """Read the found path file containing the sequence of lines that form the path. The file is specified
    in the following format:
    x_1,y_1-x_2,y_2-x_3,y_3-...
    So excluding the extremes of the list each point is a head and a tail at the same time and is contained in both lists.
    
    Returns:
        Tuple[List[Any]]: Lists containins the heads and tails of the vectors."""
    with open(filename, 'r') as f:
        content = f.read().strip()

    points = (content.split('-'))[:-1]

    heads = []
    tails = []
    for i in range(len(points)-1):
        xl, yl = map(float, points[i].split(','))
        xr, yr = map(float, points[i+1].split(','))
        heads.append(np.array([xl,yl]))
        tails.append(np.array([xr,yr]))

    return heads, tails


if __name__ == "__main__":
    
    heads_graph, tails_graph = parse_graph_file('results/graph.txt')

    heads_path, tails_path = parse_path_file('results/path.txt')

    plt.figure(figsize=(10, 8))
    for h, t in zip(heads_graph, tails_graph):
        plt.plot([h[0], t[0]], [h[1],t[1]], '-bo', markersize=0.05)
    for h, t in zip(heads_path, tails_path):
        plt.plot([h[0], t[0]], [h[1],t[1]], '-ro', markersize=0.05)

    plt.plot([1,3],[1,1], '-', color="black")
    plt.plot([3,3],[1,3], '-', color="black")
    plt.plot([3,1],[3,3], '-', color="black")
    plt.plot([1,1],[3,1], '-', color="black")

    plt.plot([8,9],[1,1], '-', color="black")
    plt.plot([9,9],[1,9], '-', color="black")
    plt.plot([9,8],[9,9], '-', color="black")
    plt.plot([8,8],[9,1], '-', color="black")

    plt.plot([6,7],[6,6], '-', color="black")
    plt.plot([7,7],[6,7], '-', color="black")
    plt.plot([7,6],[7,7], '-', color="black")
    plt.plot([6,6],[7,6], '-', color="black")

    plt.title('RRT Graph & Solution')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.grid(True)
    plt.axis('equal')