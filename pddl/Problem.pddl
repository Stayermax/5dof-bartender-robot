(define (problem Test1)
    (:domain bartender)
    (:objects
	cup1 cup2 cup3 - cup
	; 1 red - rum
	; 2 blue - vodka
	; 3 green - absinthe
	; 4 orange - juice
	; 5 pink - soda
	; 6 yellow - whiskey
	rum vodka absinthe juice soda whiskey - bottle
	grabber - arm
	RandS VandJ WandJ RVAJ RVAJSW RASW WAJ - cocktails 
	)

(:init
    (handempty grabber)
    
    (ontable cup1)
    (ontable cup2)
    (ontable cup3)
    
    (ontable rum)
    (ontable vodka)
    (ontable absinthe)
    (ontable juice)
    (ontable soda)
    (ontable whiskey)
    
    (recipe rum RandS)
    (recipe soda RandS)
    
    (recipe vodka VandJ)
    (recipe juice VandJ)
    
    (recipe juice WandJ)
    (recipe whiskey WandJ)
    
    (recipe rum RVAJ)
    (recipe vodka RVAJ)
    (recipe absinthe RVAJ)
    (recipe juice RVAJ)
    
    (recipe rum RVAJSW)
    (recipe vodka RVAJSW)
    (recipe absinthe RVAJSW)
    (recipe juice RVAJSW)
    (recipe soda RVAJSW)
    (recipe whiskey RVAJSW)
    
    (recipe rum RASW)
    (recipe absinthe RASW)
    (recipe soda RASW)
    (recipe whiskey RASW)
    
    (recipe whiskey WAJ)
    (recipe absinthe WAJ)
    (recipe juice WAJ)
    
    (order RVAJ cup1)
    (order RASW cup2)
    (order WAJ cup3)
    )

 (:goal
  (done)))
