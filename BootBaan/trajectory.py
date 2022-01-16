import opensim as osim
import math, csv
import numpy as np
from pathlib import Path, PurePath

from bb_common import *

"""     Create the rowing motion and write to a trc (track row column) file
          we pretend the boat is not moving.

        All heigths are wrt the boat frame.
"""

# timing parameters
Hz = 50
strokerate = 30
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
      1. height of the blade above the water
            determines the hand height
      2. frontangle/kneeangle
            start until stretched
                  first calculate start, see what is possible/wanted
      3. backangle
             (upperleg angle-x) to 45 degrees back
      4. elbowangle
            arm stretched to max (from frontpos to backpos)
                  first calculate backpos to determine max
            when elbow is bend:
              plane created with upper en lower arm is at 45 degrees
              but if elbow should be not lower than the handle heigth

     For the moment we use the following:
"""
frontangle_start = pi/2 - 0.2
elbowangle_end = 3/4*pi

"""
   Fases in de haal

    0. Inzet

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

    6. Uitzet

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
a_end = 100
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
    if fa < 2 and ba == 0 and ea == 0:
        phase = 0
    elif fa < 100 and ba == 0 and ea == 0:
        phase = 1
    elif fa < 100 and 0 < ba < 100 and ea == 0:
        phase = 2
    elif fa < 100 and 0 < ba < 100 and ea > 0:
        phase = 3
    elif fa == 100 and ba <100:
        phase = 4
    else:
        # also exit phase
        phase = 5

    return (phase, fa, ba, ea)


def calc_pos(pos, blheight, bbaan, time):
    """ Determine all marker positions   """

    """ First determine handle circles   """
    # Starboard side
    #  place of (frame of) lock (0.02 higher than port side)
    slockheight = boatHeight+seatHeight+lockHeight + 0.025 + 0.02 #   hoogteverschil geeft iets raars bij de elleboog in de uitzet 
    slockplace = np.array((place, slockheight, span/2))
    # handle height with blade  blheight above the water
    soarangle = math.asin((slockheight-blheight)/(outboard-bladepoint))
    shandleheight = slockheight + inboard*math.asin(soarangle)

    # center and radius of handle circle
    scirc_c = np.array([place, shandleheight, span/2])
    scircrad = inboard*math.cos(soarangle)

    # Port side
    plockheight = boatHeight+seatHeight+lockHeight + 0.025
    plockplace = np.array((place, plockheight, -span/2))
    # handle height with blade  blheight above the water
    poarangle = math.asin((plockheight-blheight)/(outboard-bladepoint))
    phandleheight = plockheight + inboard*math.asin(poarangle)

    # center and radius of circle
    pcirc_c = np.array([place, phandleheight, -span/2])
    pcircrad = inboard*math.cos(poarangle)
    
    # normal of both circle planes
    handle_normal = np.array([0, 1, 0])

    """ determine positions   """
    (phase, fa, ba, ea) = pos

    # from percentage to angle
    # frontangle start
    frontangle_b = frontangle_start
    # frontangle with stretched legs
    frontangle_e = math.asin(seatHeight/(ulegl+llegl))
    frontangle = frontangle_b+fa*(frontangle_e-frontangle_b)/100

    # backangle (lowbackangle, note the angleinb!) (loopt af!)
    backangle_b = 0.2   # voorlopig, nog af laten hangen van hoek bovenbeen.
    backangle_e = -0.7
    backangle = backangle_b+ba*(backangle_e-backangle_b)/100

    # elbowangle start
    elbowangle_b = 0  # gestrekte arm
    # elbowangle end
    elbowangle_e = elbowangle_end
    elbowangle = elbowangle_b+ea*(elbowangle_e-elbowangle_b)/100

    """  Markers   """
    setPosition('mBoat', (0, 0, 0))

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

    # shoulder position (lower back vertical, upper back bend angleinb forward)
    shposvert = lbackl*math.cos(backangle) + (ubackl+shoulderth/2) * math.cos(backangle+angleinb)
    shposhor  = hpos - lbackl*math.sin(backangle) - (ubackl+shoulderth/2) * math.sin(backangle+angleinb)
    setPosition('mShoulder', (shposhor, boatHeight+3*seatHeight/2+shposvert, 0))
    # print("back:  ", math.degrees(backangle))
    # shoulder left
    shleftpos = (shposhor, boatHeight+3*seatHeight/2+shposvert, shoulderl/2)
    shoulderleft = np.array(shleftpos)

    # shoulder right
    shrightpos = (shposhor, boatHeight+3*seatHeight/2+shposvert, -shoulderl/2)
    shoulderright = np.array(shrightpos)
    
    """ Arm positions,
          find the intersection of sphere and circle
          https://gamedev.stackexchange.com/questions/75756/sphere-sphere-intersection-and-circle-sphere-intersection

        If the arm is bend, we take the "effective" length of the arm and later position the elbow.
    """
    ax = uarml + larml*math.cos(elbowangle)
    ay = larml*math.sin(elbowangle)
    sphererad = math.sqrt(ax**2 + ay**2)
    # we ignore the little difference in arm lenght due to height difference in the locks

    """  Determine circle cut by sphere in handle plane   """
    # Starboard
    # plane of the circle cuts the sphere d = dot(n, c_c - c_s) units from the sphere's center
    dcut = np.dot(handle_normal, scirc_c-shoulderleft)
    if abs(dcut) >= sphererad:
        print("sphere does not intersect circle, exiting.")
    # circle cut by plane of original circle
    c_p = shoulderleft + dcut*handle_normal
    r_p = math.sqrt(sphererad*sphererad - dcut*dcut)

    # Intersections of the two circles
    d = np.linalg.norm(scirc_c - c_p)
    h = 1/2 + (scircrad * scircrad - r_p * r_p)/(2 * d*d)
    c_i = scirc_c + h * (c_p - scirc_c)

    t = normalize(np.cross(c_p - scirc_c, handle_normal))

    p_0 = c_i - t * r_p
    p_1 = c_i + t * r_p

    # always p_0?
    setPosition('msHandle', p_0)

    # Place elbow
    """ 
       first place elbow a 45 degrees. If above handle plane, then OK.
       else set to handle plane


       find angle at handle in (shoulder, handle, elbow)_ triangle using cosine rule:
          c^2 = a^2 + b^2 -2ab.cos(C)
          cangle = math.acos( (a^2 + b^2 - c^2)/(2ab) )
       then deterime projection of elbow on handle-shoulder line
    """
    # normal of circle of elbow
    elnormal = normalize(p_0 - shoulderleft)

    cangle = math.acos((larml*larml + sphererad*sphererad - uarml*uarml)/(2*larml*sphererad))
    # distance to elcenter (we could remove cos and acos)
    p = larml * math.cos(cangle)
    elcenter = p_0 + p*elnormal
    re = larml * math.sin(cangle)

    # distance shoulder to center of circle
    tmp = uarml*uarml - re*re
    if tmp > 0:
        a = math.sqrt( uarml*uarml - re*re)
    else:
        # a on other side of elbow plane)
        a = - math.sqrt(abs(uarml*uarml - re*re))
    #print("a ", a)
    
    # center of circle
    elcenter = shoulderleft + a*elnormal

    # create the 2 other normals
    #  first pick a random (unit) vector in the elbow plane
    n1 = np.cross(elnormal, handle_normal)
    n2 = np.cross(elnormal, n1)

    # equation of circle

    # about 45 degrees
    theta = 0.75*pi
    deze = elcenter + re*(n1*math.cos(theta) + n2*math.sin(theta))
    if deze[1] >= p_0[1]:
        setPosition('msElbow', deze)
    else:
        # set elbow to handle height
        h = elcenter[1] - p_0[1]
        b = math.sqrt(re*re - h*h)
        deze = (deze[0], p_0[1], elcenter[2]+b)
        setPosition('msElbow', deze)

    """
    (x, y, z) = deze
    mkmarker = osim.Marker("PORTCIRCLE", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = elcenter
    mkmarker = osim.Marker("ELCENTER", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    """
    
    """ blades horizontal
          y = blheight, x and z from: handle, lock, inboard, (outboard-bladepoint)

    """
    outer = (outboard-bladepoint)*math.cos(soarangle)
    horoarangle = math.atan((place - p_0[0])/(span/2 - p_0[2]))
    # connection of blade to the oar
    x = place + outer*math.sin(horoarangle)
    z = span/2 + outer*math.cos(horoarangle)
    msbladepos = (x, blheight, z)
    setPosition('msBlade', msbladepos)

    # Port
    # plane of the circle cuts the sphere d = dot(n, c_c - c_s) units from the sphere's center
    dcut = np.dot(handle_normal, pcirc_c-shoulderright)
    if abs(dcut) >= sphererad:
        print("sphere does not intersect circle, exiting.")
    # circle cut by plane of original circle
    c_p = shoulderright + dcut*handle_normal
    r_p = math.sqrt(sphererad*sphererad - dcut*dcut)

    # Intersecions of the two circles
    d = np.linalg.norm(pcirc_c - c_p)
    h = 1/2 + (pcircrad * pcircrad - r_p * r_p)/(2 * d*d)
    c_i = pcirc_c + h * (c_p - pcirc_c)

    t = normalize(np.cross(c_p - pcirc_c, handle_normal))

    p_0 = c_i - t * r_p
    p_1 = c_i + t * r_p

    # always p_1?
    setPosition('mpHandle', p_1)

    # Place elbow
    """ 
       first place elbow a 45 degrees. If above handle plane, then OK.
       else set to handle plane
    """
    # normal of circle of elbow
    elnormal = normalize(shoulderleft - p_1)

    cangle = math.acos((larml*larml + sphererad*sphererad - uarml*uarml)/(2*larml*sphererad))
    # distance to elcenter (we could remove cos and acos)
    p = larml * math.cos(cangle)
    elcenter = p_1 + p*elnormal
    re = larml * math.sin(cangle)

    # distance shoulder to center of circle
    tmp =  uarml*uarml - re*re
    if tmp > 0:
        a = math.sqrt( uarml*uarml - re*re )
    else:
        # a on other side of elbow plane)
        a = - math.sqrt(abs(uarml*uarml - re*re))
    #print("a ", a)
    
    # normal of circle of elbow
    elnormal = normalize(p_1 - shoulderright)
    # center of circle
    elcenter = shoulderright + a*elnormal

    # create the 2 other normals
    #  first pick a random (unit) vector in the elbow plane
    n1 = np.cross(elnormal, handle_normal)
    n2 = np.cross(elnormal, n1)

    # equation of circle

    # about 45 degrees
    theta = 0.25*pi
    deze = elcenter + re*(n1*math.cos(theta) + n2*math.sin(theta))
    if deze[1] >= p_1[1]:
        setPosition('mpElbow', deze)
    else:
        # set elbow to handle height
        h = elcenter[1] - p_1[1]
        b = math.sqrt(re*re - h*h)
        deze = (deze[0], p_1[1], elcenter[2]-b)
        setPosition('mpElbow', deze)

    outer = (outboard-bladepoint)*math.cos(poarangle)
    horoarangle = math.atan((place - p_1[0])/(p_1[2] + span/2))
    x = place + outer*math.sin(horoarangle)
    z = -span/2 - outer*math.cos(horoarangle)
    mpbladepos = (x, blheight, z)
    setPosition('mpBlade', mpbladepos)


def main():
    global b_end
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

        # Start rowing!
        time = 0
        
        #  Entry step 1
        pos = pos_in_parts(0)
        # set position
        calc_pos(pos, blheight+0.05, bbaan, time)
        nextstring = [time, (time)/Hz]
        for x in position:
            nextstring.append(f'{x[0]:.3f}')
            nextstring.append(f'{x[1]:.3f}')
            nextstring.append(f'{x[2]:.3f}')
        report_writer.writerow(nextstring)
        time += 1

        # Entry step 2
        pos = pos_in_parts(1)  # tempo?
        # set position
        calc_pos(pos, blheight, bbaan, time)
        nextstring = [time, (time)/Hz]
        for x in position:
            nextstring.append(f'{x[0]:.3f}')
            nextstring.append(f'{x[1]:.3f}')
            nextstring.append(f'{x[2]:.3f}')
        report_writer.writerow(nextstring)
        time += 1

        for n in range(nmbrstrokes):
            # stroke
            for t in range(2, strokeend):
                # t is normalized time from start of this stroke
                pos = pos_in_parts(t*100/strokeend)
                # set position
                calc_pos(pos, blheight, bbaan, time)
                nextstring = [time, time/Hz]
                for x in position:
                    nextstring.append(f'{x[0]:.3f}')
                    nextstring.append(f'{x[1]:.3f}')
                    nextstring.append(f'{x[2]:.3f}')
                report_writer.writerow(nextstring)
                time += 1

            #  Exit step 1
            # Only vertical
            pos = pos_in_parts(100)
            # set position
            calc_pos(pos, blheight+0.05, bbaan, time)
            nextstring = [time, (time)/Hz]
            for x in position:
                nextstring.append(f'{x[0]:.3f}')
                nextstring.append(f'{x[1]:.3f}')
                nextstring.append(f'{x[2]:.3f}')
            report_writer.writerow(nextstring)
            time += 1

            # Exit step 2
            pos = pos_in_parts(99)
            # set position
            calc_pos(pos, blheight+0.10, bbaan, time)
            nextstring = [time, (time)/Hz]
            for x in position:
                nextstring.append(f'{x[0]:.3f}')
                nextstring.append(f'{x[1]:.3f}')
                nextstring.append(f'{x[2]:.3f}')
            report_writer.writerow(nextstring)
            time += 1

            # recover  (stroke in reverse for now)
            b_end -= 20  # delay start of back in recover wrt stroke
            aantal = recoverend - recoverstart
            for t in range(aantal, 2, -1):
                pos = pos_in_parts(t*100/aantal)
                # set position
                calc_pos(pos, blheight+0.10, bbaan, time)
                nextstring = [time, (time/Hz)]
                for x in position:
                    nextstring.append(f'{x[0]:.3f}')
                    nextstring.append(f'{x[1]:.3f}')
                    nextstring.append(f'{x[2]:.3f}')
                if t == 0:
                    print("The position:")
                    print(nextstring, "\n")
                report_writer.writerow(nextstring)
                time += 1
            b_end += 20

            #  Entry step 1
            pos = pos_in_parts(0)
            # set position
            calc_pos(pos, blheight+0.05, bbaan, time)
            nextstring = [time, (time)/Hz]
            for x in position:
                nextstring.append(f'{x[0]:.3f}')
                nextstring.append(f'{x[1]:.3f}')
                nextstring.append(f'{x[2]:.3f}')
            report_writer.writerow(nextstring)
            time += 1

            # Entry step 2
            pos = pos_in_parts(1)  # tempo?
            # set position
            calc_pos(pos, blheight, bbaan, time)
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

