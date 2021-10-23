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
	rum vodka absinthe juice soda whiskey - alch
	grabber - arm
	RandS VandJ WandJ RVAJ - cocktails ; same thing as orders
	one two three - orders
	)

(:init
    (ontable cup1)
    (ontable cup2)
    (ontable cup3)
    (clean cup1)
    (clean cup2)
    (clean cup3)
    (ontable vodka)
    (ontable whiskey)
    (ontable rum)
    (ontable soda)
    (ontable juice)
    (handempty grabber)
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
    (order one RandS cup1)
    (order two RVAJ cup2)
    (order three WandJ cup3)
    )

 (:goal
  (done)))