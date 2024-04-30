(define (domain pick_and_place_nav)
    (:types object robot)

    (:predicates 
        (GRASPABLE ?x - object)
        (CONTAINABLE ?x - object)
        (carry ?r - robot ?x - object)
        (contain ?x - object ?y - object)
        (free ?r - robot)
        (close ?x - object ?y - object)
        (at-robot ?r ?y - object)
    )

    (:action navigate-to-pickable
        :parameters (?r - robot ?obj - object)
        :precondition 
        (and 
            (free ?r)
            (GRASPABLE ?obj)
            (not (carry ?r ?obj))
        )
        :effect 
        (at-robot ?r ?obj)
    )

    (:action navigate-to-container
        :parameters (?r - robot ?cont - object ?item - object)
        :precondition 
        (and 
            (carry ?r ?item)
            (CONTAINABLE ?cont)
        )
        :effect 
        (at-robot ?r ?cont)
    )

    (:action pickup
        :parameters (?x - object ?r - robot)
        :precondition 
        (and 
            (GRASPABLE ?x)
            (free ?r)
            (at-robot ?r ?x)
        )
        :effect 
        (carry ?r ?x)
    )

    (:action place 
        :parameters (?x - object ?r - robot ?z - object)
        :precondition 
        (and 
            (carry ?r ?x)
            (CONTAINABLE ?z)
            (at-robot ?r ?z)
        )
        :effect 
        (contain ?x ?z)
    )
)