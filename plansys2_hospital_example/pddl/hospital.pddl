(define (domain navegacion_interiores)
(:requirements :strips :equality :typing :durative-actions)
(:types
    corridor room zone - place 
    door
    object 
    elevator 
    robot
)

(:predicates
    (robotAt ?rob - robot ?x - place)
    (robotIdle ?rob - robot)
    (robotGrab ?rob - robot ?o - object)

    (objectAt ?o - object ?x - place)

    (doorAt ?d - door ?x - place)
    (doorOpened ?d - door)
    (doorClosed ?d - door)

    (placesTogether ?x1 - place ?x2 - place)
    (placesDoor ?x1 - place ?d - door ?x2 - place)
    (placesFree ?x1 - place ?x2 - place)

    (elevatorAt ?e - elevator ?x - place)
    
)

(:durative-action take_elevator
    :parameters (?rob - robot ?from - corridor ?e - elevator ?to - corridor)
    :duration (= ?duration 5)
    :condition (and 
        (at start (and 
            (robotAt ?rob ?from)
            (robotIdle ?rob)
        ))
        (over all (and 
            (elevatorAt ?e ?from)
            (elevatorAt ?e ?to)
        ))
    )
    :effect (and 
        (at start (and
            (not (robotAt ?rob ?from)) 
            (not (robotIdle ?rob))
        ))
        (at end (and
            (robotAt ?rob ?to)
            (robotIdle ?rob)
        ))
    )
)

(:durative-action open_door
    :parameters (?rob - robot ?x - place ?d - door)
    :duration (= ?duration 0.5)
    :condition (and 
        (at start (and
            (doorClosed ?d)
            (robotIdle ?rob)
        ))
        (over all (and 
            (robotAt ?rob ?x)
            (doorAt ?d ?x)
        ))
    )
    :effect (and 
        (at start (and 
            (not (doorClosed ?d))
            (not (robotIdle ?rob))
        ))
        (at end (and
            (doorOpened ?d)
            (robotIdle ?rob)
        ))
    )
)

(:durative-action close_door
    :parameters (?rob - robot ?x - place ?d - door)
    :duration (= ?duration 0.5)
    :condition (and 
        (at start (and
            (doorOpened ?d)
            (robotIdle ?rob)
        ))
        (over all (and 
            (robotAt ?rob ?x)
            (doorAt ?d ?x)
        ))
    )
    :effect (and 
        (at start (and 
            (not (doorOpened ?d))
            (not (robotIdle ?rob))
        ))
        (at end (and
            (doorClosed ?d)
            (robotIdle ?rob)
        ))
    )
)

(:durative-action move_across_door
    :parameters (?rob - robot ?from - place ?d - door ?to - place)
    :duration (= ?duration 2)
    :condition (and 
        (at start (and 
            (robotAt ?rob ?from)
            (robotIdle ?rob)
        ))
        (over all (and 
            (placesTogether ?from ?to)
            (placesDoor ?from ?d ?to)
            (doorOpened ?d)
        ))
    )
    :effect (and 
        (at start (and 
            (not (robotAt ?rob ?from))
            (not (robotIdle ?rob))
        ))
        (at end (and 
            (robotAt ?rob ?to)
            (robotIdle ?rob)
        ))
    )
)

(:durative-action move
    :parameters (?rob - robot ?from - place ?to - place)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (robotAt ?rob ?from)
            (robotIdle ?rob)
        ))
        (over all (and 
            (placesTogether ?from ?to)
            (placesFree ?from ?to)
        ))
    )
    :effect (and 
        (at start (and
            (not (robotAt ?rob ?from))
            (not (robotIdle ?rob))
        ))
        (at end (and
            (robotAt ?rob ?to) 
            (robotIdle ?rob)
        ))
    )
)

(:durative-action grab_object
    :parameters (?rob - robot ?o - object ?x - place)
    :duration (= ?duration 4)
    :condition (and
        (at start (and 
            (objectAt ?o ?x)
            (robotIdle ?rob)
        ))
        (over all (and 
            (robotAt ?rob ?x)
        ))
    )
    :effect (and 
        (at start (and
            (not (objectAt ?o ?x))
            (not (robotIdle ?rob))
        ))
        (at end (and 
            (robotGrab ?rob ?o)
            (robotIdle ?rob)
        ))
    )
)

(:durative-action release_object
    :parameters (?rob - robot ?o - object ?x - place)
    :duration (= ?duration 4)
    :condition (and 
        (at start (and 
            (robotGrab ?rob ?o)
            (robotIdle ?rob)
        ))
        (over all (and 
            (robotAt ?rob ?x)
        ))
    )
    :effect (and 
        (at start (and
            (not (robotGrab ?rob ?o))
            (not (robotIdle ?rob))
        ))
        (at end (and 
            (objectAt ?o ?x)
            (robotIdle ?rob)
        ))
    )
)
)