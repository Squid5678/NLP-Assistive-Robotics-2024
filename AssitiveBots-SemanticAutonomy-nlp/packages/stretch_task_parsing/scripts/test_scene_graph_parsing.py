from scene_graph_node import SceneGraphNode
from scene_graph_parsing import problem_pddl_from_scene_graph

def create_example_scene_graph():

    kitchen_node = SceneGraphNode(name="Kitchen", attributes=["CONTAINABLE"])
    apple_node = SceneGraphNode(name="Apple", attributes=["GRASPABLE"])

    return {
        kitchen_node: [apple_node],
        apple_node: [kitchen_node]
    }

def test_pddl_generation():

    scene_graph = create_example_scene_graph()

    print(scene_graph)

    print(problem_pddl_from_scene_graph(scene_graph))

if __name__ == "__main__":
    
    test_pddl_generation()