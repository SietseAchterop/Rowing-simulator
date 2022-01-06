import opensim as osim
import math, csv
import numpy as np
from pathlib import Path, PurePath

from bb_common import *

# time steps
global gltime

"""     Create the rowing motion and write to a trc (track row column) file
          we pretend the boat is not moving.

        All heigths are wrt the boat frame.
"""

# timing parameters
Hz = 50
strokerate = 1
strokepart = 1/3  # part of time that is a stroke. 1/3 means a stroke/recover ratio of  1/2

# steps of 1/Hz seconds. We calculate in steps
# we leave 1 step(s) for both exit and entry.
#      maybe 2 at 100 Hz
# time steps for each complete cycle
totalsteps = int(60/strokerate*Hz)
strokestart = 0
stroketime = int(60/strokerate*strokepart*Hz)
strokeend = stroketime - 1
recoverstart = stroketime
recoverend = totalsteps - 1

#
nmbrstrokes = 3
#  height of (middle of) blade above the water
blheight = 0.1

""" position of the rower is determined by 4 values:
      1. height of th blade above the water
            determines the hand height
      2. frontangle/kneeangle
            start until stretched
                  first calculate start, see what is possible/wanted
      3. backangle
             (upperleg angle-x) to 45 degrees back
      4. elbowangle
            arm stretched to max (from backpos)
                  first calculate backpos to determine max
            when elbow is bend:
              plane created with upper en lower arm is at 45 degrees
              but if elbow should be not lower than the handle heigth

     We kiezen frontangle en elbowangle voorlopig op de gok.
"""
frontangle_start = pi/2 - 0.2
elbowangle_end = 3/4*pi

"""
   Fases in de haal

    1. Alleen benen
        hand op bol rond schouder snijd handle cirkel

    2. benen rug
        als 1

    3. benen rug armen
        elleboog kan richting handle hoogte gaan. maar ook naar buiten. kies voorlopig 45 graden naar buiten.
        hand op bol rond elleboog snijf handle cirkel

    4. rug armen
        als 3
    5. armen
        als 3


    Dus: benen rug legt een hoop vast
         arm/handle:
            gestrekte arm: botl vanuit schouder
            gebogen arm: bepaal plaats elleboog en dan bol vanuit elleboog
    """

# when are legs, back and arms used in the stroke as percentage of the stroke
l_st  = 0   # always zero
l_end = 70
b_st  = 25
b_end = 95
a_st  = 50
a_end = 98
# for recover we go backwards with (at least) a different b_end.

# the current position
position = np.zeros((nmbrmarkers, 3))


def normalize(v):
    """ normalize vector v, should be non-zero!  """
    return v / np.sqrt(np.sum(v**2))


def setPosition(name, place):
    """ Set trajectory points for front position"""
    position[markers.index(name)] = place


# correction functions to determine the actual angles at a point time
#   linear functions for now
def l_curve(t):
    # linear for now
    return t

def b_curve(t):
    # linear for now
    return t

def a_curve(t):
    # linear for now
    return t

# alleen nog voor de haal!
def pos_in_parts(t):
    """ Determine the position of frontangle, backangle, elbowangle
        using functions to tune the trajectory in the stroke

    Args:
        t:  (normalized) time index in the stroke: range(100)

    Returns:
        phase: part of the stroke
        fa: frontangle, angle of lower leg wrt horizontal.
        ba: backangle, lower back angle
        ea: elbowangle
    """
    # legs (always start at zero)
    # 
    if t < l_end:
        fa = l_curve(t*100/l_end)
    else:
        fa = l_curve(100)

    # back
    if t < b_st:
        ba = b_curve(0)
    elif t > b_end:
        ba = b_curve(100)
    else:
        ba = b_curve((t-b_st)*100/(b_end-b_st))
        
    # arms (always end at 97)
    if t < a_st:
        ea = a_curve(0)
    elif t > a_end:
        ea = a_curve(100)
    else:
        ea = a_curve((t-a_st)*100/(a_end-a_st))
        
    # determine stroke phase
    if fa < 100 and ba == 0 and ea == 0:
        phase = 1
    elif fa < 100 and 0 < ba < 100 and ea == 0:
        phase = 2
    elif fa < 100 and 0 < ba < 100 and ea > 0:
        phase = 3
    elif fa == 100 and ba <100:
        phase = 4
    else:
        phase = 5

    return (phase, fa, ba, ea)


def calc_pos(pos, blheight, bbaan):
    global gltime
    """ Determine all marker positions   """

    """ First determine handle circles   """

    """ determine positions   """
    (phase, fa, ba, ea) = pos

    # from percentage to angle
    # frontangle start
    frontangle_b = frontangle_start
    # frontangle with stretched legs
    frontangle_e = math.asin(seatHeight/(ulegl+llegl))
    frontangle = frontangle_b+fa*(frontangle_e-frontangle_b)/100

    ''' Seat position   '''
    # knee position
    kneeh = llegl * math.sin(frontangle)
    if kneeh >= ulegl + seatHeight:
        print("Cannot reach front position, exiting")
        exit()
    # intersection circle with seatHeight with highest x
    kneeposvert = boatHeight/2 + boatHeight/2 + seatHeight/2 + kneeh
    kneeposhor  = llegl * math.cos(frontangle)
    mkneepos = (kneeposhor, kneeposvert, 0)
   
    # seat position
    hpos = kneeposhor + math.sqrt(ulegl**2 - (kneeh - seatHeight)**2)
    setPosition('mSeat', (hpos, boatHeight+seatHeight, 0))

def main():
    global gltime, b_end
    trfile = Path.cwd() / 'trajectory.trc'
    with open(trfile, mode='w') as report_file:
        report_writer = csv.writer(report_file, dialect='excel-tab')

        # start of header
        report_writer.writerow(["PathFileType", 4, "(X/Y/Z)", "trajectory.trc"])
        report_writer.writerow(["DataRate", "CameraRate", "NumFrames", "NumMarkers", "Units", "OrigDataRate", "OrigDataStartFrame", "OrigNumFrames"])
        report_writer.writerow([Hz, Hz, totalsteps*nmbrstrokes, nmbrmarkers, "m", Hz, 1, 10])
        line = ["Frame#", "Time"]
        for f in range(nmbrmarkers):
            line.append(markers[f])
            line.append("")
            line.append("")
        report_writer.writerow(line)
        line = ["", ""]
        for f in range(nmbrmarkers):
            line.append("X"+str(f+1))
            line.append("Y"+str(f+1))
            line.append("Z"+str(f+1))
        report_writer.writerow(line)
        report_writer.writerow("")

        # if we want to create markers to test
        bbaan = osim.Model("BootBaan.osim")
        bbaan.setName('Trajectory')


        # time in steps!
        time = 0
        for n in range(2):  #nmbrstrokes):
            # stroke
            for t in range(strokeend):
                # normalized time
                pos = pos_in_parts(t*100/strokeend)
                # set position
                gltime = time+t
                calc_pos(pos, blheight, bbaan)
                nextstring = [gltime, gltime/Hz]
                for x in position:
                    nextstring.append(f'{x[0]:.3f}')
                    nextstring.append(f'{x[1]:.3f}')
                    nextstring.append(f'{x[2]:.3f}')
                report_writer.writerow(nextstring)
                # t/m 97, dus arm nog niet af!

            #  Exit steps
            time = time + strokeend
            pos = pos_in_parts(strokeend)
            # set position
            calc_pos(pos, blheight+0.05, bbaan)
            nextstring = [time, (time)/Hz]
            for x in position:
                nextstring.append(f'{x[0]:.3f}')
                nextstring.append(f'{x[1]:.3f}')
                nextstring.append(f'{x[2]:.3f}')
            report_writer.writerow(nextstring)
            time += 1

            # recover  (voorlopig de haal achterste voren)
            b_end -= 20
            for t in range(strokeend, 0, -1):
                pos = pos_in_parts(t*100/strokeend)
                # set position
                calc_pos(pos, blheight+0.10, bbaan)
                nextstring = [time+(strokeend-t), (time+(strokeend-t))/Hz]
                for x in position:
                    nextstring.append(f'{x[0]:.3f}')
                    nextstring.append(f'{x[1]:.3f}')
                    nextstring.append(f'{x[2]:.3f}')
                if t == 0:
                    print("The position:")
                    print(nextstring, "\n")
                report_writer.writerow(nextstring)
            b_end += 20

            #  Entry steps
            time = time + strokeend

            pos = pos_in_parts(0)
            # set position
            calc_pos(pos, blheight+0.08, bbaan)
            nextstring = [time, (time)/Hz]
            for x in position:
                nextstring.append(f'{x[0]:.3f}')
                nextstring.append(f'{x[1]:.3f}')
                nextstring.append(f'{x[2]:.3f}')
            report_writer.writerow(nextstring)
            time += 1
        # done

        # Create model for testing
        bbaan.finalizeConnections()
        bbaan.printToXML("Trajectory.osim")
    
if __name__ == "__main__":
    main()

