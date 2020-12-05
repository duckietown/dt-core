#!/usr/bin/env python3

import g2o
import numpy as np
from cslam import G2OPoseGraphOptimizer, TFGraph, TF
import matplotlib.pyplot as plt
import networkx as nx
import geometry
from tf import transformations as tf


position = {
    "world": [0, 0, 0],
    "tag": [2, 2, 0],
    "watchtower": None
}

observations = [
    # {
    #     "t": [0, 0, 10],
    #     "R": tf.euler_matrix(0, np.deg2rad(90), 0)
    # },
    # {
    #     "t": [-0.3, 0.3, 0.57],
    #     "R": tf.euler_matrix(-0.18048327699078298, -0.0335326209601194, 0.020760104661116163)
    # }
    # tf.compose_matrix(translate=[0, 0, -1], angles=[0, np.deg2rad(30), 0])
    # tf.compose_matrix(translate=[-0.3, 0.3, 0.57], angles=[np.deg2rad(-0.18), np.deg2rad(0), np.deg2rad(0)])
    # tf.compose_matrix(translate=[-0.3, 0.3, 0.57], angles=[-0.18048, -0.03353, 0.02076])
    tf.compose_matrix(
        translate=[-0.26060939402176597, 0.3477681031655947, 0.571584962124406],
        angles=[-0.18212291541271222, -0.030525071878008164, 0.02107052929723813]
    )
]

Rzero = np.eye(3)
Rtag = geometry.rotation_from_axis_angle([1, 0, 0], np.deg2rad(180))

graph = G2OPoseGraphOptimizer()

graph.add_vertex(0, g2o.Isometry3d(Rzero, position["world"]), fixed=True)
graph.add_vertex(1, g2o.Isometry3d(Rtag, position["tag"]), fixed=True)
graph.add_vertex(2)

for observation in observations:
    # R, t = observation["R"][:3, :3], observation["t"]
    # q = tf.quaternion_from_matrix(observation["R"])
    # print(q)
    # graph.add_edge([1, 2], g2o.Isometry3d(g2o.Quaternion(*list(q)), t))

    print("Adding edge ({} -> {}) w/:\n\t{}".format(2, 1, observation))

    graph.add_edge([2, 1], g2o.Isometry3d(observation))


def gprint(lbl):
    print(lbl)
    for i, n in enumerate(["tag", "watchtower"]):
        p = graph.get_pose(i + 1)
        translation = [np.round(v, decimals=4) for v in p.t]
        angles = [np.round(v, decimals=4) for v in tf.euler_from_matrix(p.R)]
        print(f"\t{n}:")
        print(f"\t\tposition:    {translation}")
        print(f"\t\torientation: {angles}")


gprint("Before Optimization")

graph.optimize(20)

gprint("After Optimization")

G = nx.MultiDiGraph()

for node in position:
    G.add_node(node)
G.add_edge("watchtower", "tag")

pos = {
    "world": graph.get_pose(0).t[:2],
    "tag": graph.get_pose(1).t[:2],
    "watchtower": graph.get_pose(2).t[:2],
}

nx.draw_networkx_nodes(G, pos, nodelist=["tag"], node_shape="s", node_color="red")
nx.draw_networkx_nodes(G, pos, nodelist=["watchtower"], node_shape="h", node_color="orange")
nx.draw_networkx_nodes(G, pos, nodelist=["world"], node_shape="P", node_color="black")

nx.draw_networkx_edges(G, pos)

plt.show()

# graph = TFGraph()
#
# graph.add_node("tag310", pose=TF(t=np.array([1, 1, 1])), fixed=True)
#
# graph.add_node("watchtower2")
#
# for i, position in enumerate(positions):
#     graph.add_measurement("watchtower2", "duckiebot1", measurement=TF(t=np.array(position)))
#
# graph.optimize()
#
# for nname, ndata in graph.nodes.data():
#     print("\nNode `{}`:\n\tposition: {}\n\torientation: {}\n\tlocation:{}".format(
#         nname, ndata["pose"].t, ndata["location"], '(zero)'
#     ))
