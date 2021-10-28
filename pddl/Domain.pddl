(define (domain bartender)
  (:requirements :strips :typing :equality :disjunctive-preconditions )
  (:types arm stuff table orders recipe - object
    cup - containers
    bottle cocktails - liquids
    containers liquids - stuff 
   )

; Define all of the relations used in the actions
  (:predicates  
        (ontable ?s - stuff)
		(holding ?h - arm ?s - stuff) ; can hold containers
        (handempty ?h - arm)
		(contains ?m - containers ?s - liquids)
		(recipe ?j - liquids ?c - cocktails)
		(completed ?c - cup)
		(order ?c - cocktails ?m - cup)
		(done))

  (:action grab
        :parameters (?a - arm ?b - bottle)
        :precondition (and (ontable ?b) (handempty ?a))
        :effect (and ( not(ontable ?b)   )
                     ( not(handempty ?a) )
                     ( holding ?a ?b     )
                 )
    )   
                      
  (:action release ; release the bottle only when poured everywhere it was needed
        :parameters (?a - arm ?b - bottle)
        :precondition (and (holding ?a ?b) 
                           (forall (?m - cup)
                                (forall (?c - cocktails)
                                    (or (and    (order ?c ?m)
                                                (recipe ?b ?c)
                                                (contains ?m ?b)
                                        )
                                        (and    (order ?c ?m)
                                                (not(recipe ?b ?c))
                                        )
                                        (and    (not(order ?c ?m))
                                        )
                                    )
                                )
                           )
                      )
        :effect (and (  ontable ?b          )
                     (  handempty ?a        )
			         (  not(holding ?a ?b)  )
		         )
    )
    
   (:action pour
        :parameters (?a - arm ?b - bottle ?m - cup ?c - cocktails)
        :precondition (and (holding ?a ?b)
                           (recipe ?b ?c)
                           (order ?c ?m)
                           (not (contains ?m ?b)))
        :effect (contains ?m ?b))
        
   (:action serve
        :parameters (?a - arm ?m - cup ?c - cocktails )
        :precondition (and  (handempty ?a)
                            (order ?c ?m)
                            (forall (?b - bottle) 
                                (or (and (recipe ?b ?c) 
                                         (contains ?m ?b)
                                     )
                                    (and (not (recipe ?b ?c)    ) 
                                         (not (contains ?m ?b)  )
                                    )
                                )
                            )
        			)  
        :effect (completed ?m)
    )

    (:action finished
    	    :parameters ()
    	    :precondition (forall (?c - cup) (completed ?c)) ; can put in conditionon that calls done after 1 completed
    	    :effect (done))
)