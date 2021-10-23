(define (domain bartender)
  (:requirements :strips :typing :equality)
  (:types arm stuff table orders recipe - object
    cup - containers
    alch cocktails - liquids
    containers liquids - stuff 
   )

; Define all of the relations used in the actions
  (:predicates  
        (ontable ?s - stuff)
		(holding ?h - arm ?s - stuff) ; can hold containers
        (handempty ?h - arm)
		(clean ?s - containers) ; liquids are always not clean. doesnt matter
		(contains ?m - containers ?s - liquids)
		(recipe ?j - liquids ?c - cocktails)
		(completed ?o - orders)
		(order ?o - orders ?c - cocktails ?m - cup)
		(done))

  (:action grab
        :parameters (?h - arm ?s - alch)
        :precondition (and (ontable ?s) (handempty ?h))
         :effect (and (not(ontable ?s))
                      (not(handempty ?h))
                      (holding ?h ?s)))
                      
  (:action release ; release the alch only if a hand is needed
        :parameters (?h1 - arm ?s1 ?s2 - alch ?m - cup)
        :precondition (and (ontable ?s2)
                           ;(not (= ?h1 ?h2)) need if multiple hands
                           (not (= ?s1 ?s2))
                           (holding ?h1 ?s1)
                           ; (not(handempty ?h2)) only if have two hands
                           (contains ?m ?s1)) ; works if the alch is poured into 1 mixer
        :effect (and (ontable ?s1)
                     (handempty ?h1)
			         (not(holding ?h1 ?s1))))

; need this action in order to pour more than 1 shot at a time if possible
   (:action pour1
        :parameters (?h - arm ?s - alch ?m - cup ?c - cocktails ?o - orders)
        :precondition (and (holding ?h ?s)
                           (recipe ?s ?c)
                           (order ?o ?c ?m)
                           (clean ?m)
                           (not(contains ?m ?s)))
        :effect (and (contains ?m ?s) 
                     (not (clean ?m))))
   (:action pour2
        :parameters (?h - arm ?s - alch ?m - cup ?c - cocktails ?o - orders)
        :precondition (and (holding ?h ?s)
                           (recipe ?s ?c)
                           (order ?o ?c ?m)
                           (not(contains ?m ?s)))
        :effect (and (contains ?m ?s)))
        
   (:action serve
        :parameters (?h - arm ?m - cup ?c - cocktails ?s1 ?s2 - alch ?o - orders)
        :precondition (and (contains ?m ?s1)
            			   (contains ?m ?s2)
            			   (handempty ?h)
            			   (not (= ?s1 ?s2))
            			   (recipe ?s1 ?c)
            			   (recipe ?s2 ?c)
            			   (order ?o ?c ?m))
        :effect (and (contains ?m ?c)
                     (clean ?m)
                     (not(order ?o ?c ?m))
                     (completed ?o)
                     (not(contains ?m ?s1))
                     (not(contains ?m ?s2)))
			     
	)
    (:action finished
    	    :parameters (?o - orders)
    	    :precondition (and (forall (?o - orders) (completed ?o))) ; can put in conditionon that calls done after 1 completed
    	    :effect (done))
)
