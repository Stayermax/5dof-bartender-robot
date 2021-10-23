BZP = 0.18 # Bottle z position
BUO = 0.8 # Bottle upper offset
CUO = 0.8 # Cup upper offset
DSDC = 0.93 # Double Safe Dictance Coefficient
SDC = 0.6 # Safe Dictance Coefficient

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

	# 1 red - rum
	# 2 blue - vodka
	# 3 green - absinthe
	# 4 orange - juice
	# 5 pink - soda
	# 6 yellow - whiskey

BottleToDrink = {
    "bottle_1" : "Rum",
    "bottle_2" : "Vodka",
    "bottle_3" : "Absinthe",
    "bottle_4" : "Juice",
    "bottle_5" : "Soda",
    "bottle_6" : "Whiskey",

}

Poses = {
    "bottle_1" : Pose(position=Point(x=0.5, y=-1, z=0), orientation=Quaternion(x=0, y=0, z=0.3826834, w=0.9238795 )),    # red
    "bottle_2" : Pose(position=Point(x=0, y=-1, z=0),   orientation=Quaternion(x=0, y=0, z=0.3826834, w=0.9238795 )),      # blue
    "bottle_3" : Pose(position=Point(x=-0.5, y=-1, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1 )),   # green
    "bottle_4" : Pose(position=Point(x=0.5, y=1, z=0),  orientation=Quaternion(x=0, y=0, z=0, w=1 )),     # orange
    "bottle_5" : Pose(position=Point(x=0, y=1, z=0),    orientation=Quaternion(x=0, y=0, z=0.3826834, w=0.9238795 )),       # pink
    "bottle_6" : Pose(position=Point(x=-0.5, y=1, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1 )),    # yellow

    "cup_1" : Pose(position=Point(x=1, y=0.5, z=0)),        # red
    "cup_2" : Pose(position=Point(x=1, y=0, z=0)),          # green
    "cup_3" : Pose(position=Point(x=1, y=-0.5, z=0))        # blue
}

Colors = {
    "bottle_1" : "red",
    "bottle_2" : "blue",
    "bottle_3" : "green",
    "bottle_4" : "orange",
    "bottle_5" : "pink",
    "bottle_6" : "yellow",

    "cup_1" : "red",
    "cup_2" : "green",
    "cup_3" : "blue"
}

# Jack bottles
Names = {
    "bottle_1" : "Jack_bottle_red",
    "bottle_2" : "Jack_bottle_blue",
    "bottle_3" : "Jack_bottle_green",
    "bottle_4" : "Jack_bottle_orange",
    "bottle_5" : "Jack_bottle_pink",
    "bottle_6" : "Jack_bottle_yellow",
    
    "cup_1" : "Cocktail_red_glass",
    "cup_2" : "Cocktail_green_glass",
    "cup_3" : "Cocktail_blue_glass"
}

PourPos = {
    "cup_1": [[1, 0, 0], -1],
    "cup_2": [[1, 0.5, 0], 1],
    "cup_3": [[1, 0, 0], 1]
}

# Round bottles
# Names = {
#     "bottle_1" : "bottle_red",
#     "bottle_2" : "bottle_blue",
#     "bottle_3" : "bottle_green",
#     "bottle_4" : "bottle_orange",
#     "bottle_5" : "bottle_pink",
#     "bottle_6" : "bottle_yellow",
    
#     "cup_1" : "Cocktail_red_glass",
#     "cup_2" : "Cocktail_green_glass",
#     "cup_3" : "Cocktail_blue_glass"
# }