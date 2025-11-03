import opensim as osim
import math
pi = math.pi

"""       Parameters of the model                """
####################################################
# rower
rowerw     = 80
rowerl     = 1.86

# Boat parameters
boatw      = 10
# rest rigging

# heigth of seat
seatHeight = 0.1
# length of boat
boatLength = 6.0
# width of boat
boatWidth = 0.6
# height of boat
boatHeight = 0.2
# distance between locks
span     = 1.60
# height lock above the seat
lockHeight   = 0.22
# horizontal distance between stretcher and lock
#   tov bankje bij inzetstand van maken?
place    = 0.45
# inboard length total
inboard = 0.88
# inboard part of the oar minus half of the hand
inbhand  = inboard-0.05
# outboard part of the oar
outboard = 1.98
# begin blad vanaf einde
bladepoint = 0.45   # scull


# difference between port and starboard lock
heightdiff = 0.02

"""   Derived parameters    """
llegw      = rowerw * 0.1
ulegw      = rowerw * 0.1
lbackw     = rowerw * 0.2
ubackw     = rowerw * 0.2
shoulderw  = rowerw * 0.1
uarmw      = rowerw * 0.05
larmw      = rowerw * 0.05
headw      = rowerw * 0.1

llegl      = rowerl * 0.27
ulegl      = rowerl * 0.23
lbackl     = rowerl * 0.10
ubackl     = rowerl * 0.23
shoulderth = rowerl * 0.05
headl      = rowerl * 0.12

shoulderl  = rowerl * 0.25
uarml      = rowerl * 0.15
larml      = rowerl * 0.14

total = llegl+ulegl+lbackl+ubackl+shoulderth+headl
if total != rowerl:
    print("Error in rower lenghts, please correct!")
    exit(1)

# angle in back, will become a variable
angleinb = 0.3

# Markers
markers = ['mBoat', 'mSeat', 'mShoulder', 'mpElbow', 'msElbow', 'mpHandle', 'msHandle', 'mpBlade', 'msBlade']
nmbrmarkers = len(markers)


def normalize(v):
    """ normalize vector v, should be non-zero!  """
    return v / np.sqrt(np.sum(v**2))


"""      Inertia             """
###################################################
"""
Solid cylinder of radius r, height h and mass m
I_xx =  1/12m(3r^2+h^2)
I_yy =  1/12m(3r^2+h^2)
I_zz =  1/2mr^2

All bodies this for simplicity?
theBoat:

I_xx = 1/12*boatw*(math.pow(boatHeight/2, 2)+math.pow(boatLength/2, 2))
I_yy = I_xx
I_zz = 1/2*boatw*math.pow(boatLength/2, 2)

Intertia because of the slow speeds probably not very relevant
"""


