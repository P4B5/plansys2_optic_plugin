(define (domain starwars)
(:requirements :strips :typing :durative-actions :equality)


(:types
    robot planet spaceship
)


(:predicates 
  (robotAt ?r - robot ?p - planet)
  (spaceshipAt ?s - spaceship ?p - planet)
  (robotIn ?r - robot ?s - spaceship)
)


(:durative-action load
    :parameters (?s - spaceship ?r - robot ?p - planet)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
        (robotAt ?r ?p) 
        (spaceshipAt ?s ?p)
        ))
    )
    :effect (and 
        (at end (and 
        (not (robotAt ?r ?p))
        (robotIn ?r ?s)
        ))
    )
)

(:durative-action unload
    :parameters (?s - spaceship ?r - robot ?p - planet)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
        (robotIn ?r ?s) 
        (spaceshipAt ?s ?p) 
        ))
    )
    :effect (and 
        (at start (and 
        (not (robotIn ?r ?s))
        (robotAt ?r ?p)
        ))

    )
)

(:durative-action fly
    :parameters (?s - spaceship ?from ?to - planet)
    :duration (= ?duration 5)
    :condition (and 
        (at start (and 
         (spaceshipAt ?s ?from) 
        ))
    )
    :effect (and 
        (at start (and 
         (not (spaceshipAt ?s ?from))
         (spaceshipAt ?s ?to)
        ))
    )
)
)

