(define (problem pick_place_scenario)
    (:domain pick_and_place_nav)
    (:objects 
        coke - object
        sink - object
        robo - robot
    )

    (:init 
        (GRASPABLE Coke)
        (CONTAINABLE sink)
        (free robo)
    )
    (:goal
       (contain coke sink)
    )
)
