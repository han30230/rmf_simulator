"""RECONSTRUCTED navigation graph and VDA5050 order helpers."""

from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math
from pathlib import Path
from typing import Any

import yaml

from vda5050_fleet_adapter.domain.entities.edge import Edge
from vda5050_fleet_adapter.domain.entities.node import Node
from vda5050_fleet_adapter.domain.value_objects.position import NodePosition


@dataclass
class DirectedGraph:
    adjacency: dict[str, list[tuple[str, float]]] = field(default_factory=dict)
    edges: set[tuple[str, str]] = field(default_factory=set)

    def add_edge(self, start: str, end: str, weight: float) -> None:
        self.adjacency.setdefault(start, []).append((end, weight))
        self.adjacency.setdefault(end, [])
        self.edges.add((start, end))

    def has_edge(self, start: str, end: str) -> bool:
        return (start, end) in self.edges


def _distance(a: dict[str, Any], b: dict[str, Any]) -> float:
    return math.hypot(float(b['x']) - float(a['x']), float(b['y']) - float(a['y']))


def build_graph(
    nodes: dict[str, dict[str, Any]],
    edges: dict[str, dict[str, Any]],
) -> DirectedGraph:
    graph = DirectedGraph()
    for edge in edges.values():
        start = edge['start']
        end = edge['end']
        if start not in nodes or end not in nodes:
            continue
        graph.add_edge(start, end, _distance(nodes[start], nodes[end]))
    return graph


def compute_path(
    graph: DirectedGraph, start: str, goal: str
) -> list[str] | None:
    if start not in graph.adjacency or goal not in graph.adjacency:
        return None
    queue: list[tuple[float, str]] = [(0.0, start)]
    costs = {start: 0.0}
    previous: dict[str, str] = {}
    while queue:
        cost, node = heapq.heappop(queue)
        if cost != costs.get(node):
            continue
        if node == goal:
            path = [goal]
            while path[-1] != start:
                path.append(previous[path[-1]])
            return list(reversed(path))
        for next_node, weight in graph.adjacency.get(node, []):
            next_cost = cost + weight
            if next_cost < costs.get(next_node, math.inf):
                costs[next_node] = next_cost
                previous[next_node] = node
                heapq.heappush(queue, (next_cost, next_node))
    return None


def find_nearest_node(
    nodes: dict[str, dict[str, Any]], x: float, y: float
) -> str | None:
    if not nodes:
        return None
    return min(
        nodes,
        key=lambda name: math.hypot(
            float(nodes[name]['x']) - x,
            float(nodes[name]['y']) - y,
        ),
    )


def load_nav_graph(
    nav_graph_path: str | Path,
) -> tuple[dict[str, dict], dict[str, dict], DirectedGraph, str]:
    with Path(nav_graph_path).open(encoding='utf-8') as stream:
        raw = yaml.safe_load(stream)
    levels = raw.get('levels') or {}
    if not levels:
        raise ValueError('navigation graph has no levels')
    map_name, level = next(iter(levels.items()))
    vertices = level.get('vertices') or []
    nodes: dict[str, dict] = {}
    index_names: list[str] = []
    for index, vertex in enumerate(vertices):
        attributes = dict(vertex[2] or {})
        name = str(attributes.get('name') or f'node{index}')
        if name in nodes:
            raise ValueError(f'duplicate navigation node name: {name}')
        nodes[name] = {
            'x': float(vertex[0]), 'y': float(vertex[1]),
            'attributes': attributes,
        }
        index_names.append(name)
    edges: dict[str, dict] = {}
    for index, lane in enumerate(level.get('lanes') or []):
        start_index, end_index = int(lane[0]), int(lane[1])
        edges[f'edge{index}'] = {
            'start': index_names[start_index],
            'end': index_names[end_index],
            'attributes': dict(lane[2] or {}),
        }
    return nodes, edges, build_graph(nodes, edges), str(map_name)


def _edge_for(
    nav_edges: dict[str, dict], start: str, end: str
) -> tuple[str, dict]:
    for name, edge in nav_edges.items():
        if edge.get('start') == start and edge.get('end') == end:
            return name, edge
    return f'{start}-{end}', {'attributes': {}}


def build_vda5050_nodes_edges(
    path: list[str],
    nav_nodes: dict[str, dict],
    map_name: str,
    *,
    base_end_index: int,
    seq_start: int = 0,
    edges: dict[str, dict] | None = None,
    turn_angle_threshold: float | None = None,
    allowed_deviation_theta: float | None = None,
    prev_base_theta: tuple[str, float] | None = None,
) -> tuple[list[Node], list[Edge]]:
    del turn_angle_threshold, prev_base_theta
    if not path:
        raise ValueError('cannot build a VDA5050 order from an empty path')
    if not 0 <= base_end_index < len(path):
        raise ValueError('base_end_index is outside the path')
    nav_edges = edges or {}
    result_nodes: list[Node] = []
    result_edges: list[Edge] = []
    for index, name in enumerate(path):
        if name not in nav_nodes:
            raise KeyError(f'unknown navigation node: {name}')
        node = nav_nodes[name]
        if index + 1 < len(path):
            next_node = nav_nodes[path[index + 1]]
            theta = math.atan2(
                next_node['y'] - node['y'], next_node['x'] - node['x']
            )
        elif index > 0:
            previous = nav_nodes[path[index - 1]]
            theta = math.atan2(
                node['y'] - previous['y'], node['x'] - previous['x']
            )
        else:
            theta = None
        attrs = node.get('attributes') or {}
        result_nodes.append(Node(
            node_id=name,
            sequence_id=seq_start + index * 2,
            released=index <= base_end_index,
            node_description=str(attrs.get('description', '')),
            node_position=NodePosition(
                x=float(node['x']), y=float(node['y']), map_id=map_name,
                theta=theta,
                allowed_deviation_xy=attrs.get('allowed_deviation_xy'),
                allowed_deviation_theta=allowed_deviation_theta,
            ),
        ))
        if index + 1 >= len(path):
            continue
        edge_name, edge_data = _edge_for(
            nav_edges, name, path[index + 1]
        )
        edge_attrs = edge_data.get('attributes') or {}
        result_edges.append(Edge(
            edge_id=edge_name,
            sequence_id=seq_start + index * 2 + 1,
            released=index < base_end_index,
            start_node_id=name,
            end_node_id=path[index + 1],
            max_speed=edge_attrs.get('speed_limit'),
            orientation=theta,
        ))
    return result_nodes, result_edges
