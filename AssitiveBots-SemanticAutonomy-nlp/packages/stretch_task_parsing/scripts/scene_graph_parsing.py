def problem_pddl_from_scene_graph(scene_graph: object) -> str:
    """
    Assume scene graph looks like the following
    {
        HouseNode: [LivingRoomNode, KitchenNode]
    }
    """

    problem_pddl_template = (
    """
    (define (problem pick_place_scenario)
        (:domain pick_and_place_nav)
        (:objects 
            {scene_graph_entities}
            robo - robot
        )

        (:init 
            {scene_graph_properties}
            (free robo)
        )
        {goals}
    )
    """
    )

    sg_entities = "\n".join(["{sgo} - object".format(sgo=scene_graph_object.name) for scene_graph_object in scene_graph])
    sg_properties = "\n".join(
        ["({property} {sgo})".format(property=sg_property, sgo=scene_graph_object.name) 
        for scene_graph_object in scene_graph for sg_property in scene_graph_object.attributes]
        )

    possible_goal_states = []

    for graspable_scene_graph_object in scene_graph:

        for containable_scene_graph_object in scene_graph:

            if "GRASPABLE" in graspable_scene_graph_object.attributes and "CONTAINABLE" in containable_scene_graph_object.attributes:

                possible_goal_states.append(
                    """
                    (:goal
                        (contain {go} {co})
                    )
                    """.format(go=graspable_scene_graph_object.name, co=containable_scene_graph_object.name)
                )

    return problem_pddl_template.format(
        scene_graph_entities=sg_entities, 
        scene_graph_properties=sg_properties,
        goals="\n".join(possible_goal_states))