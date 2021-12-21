import opensim as osim
import math, csv
import numpy as np
from pathlib import Path, PurePath

from bb_common import *

"""     Create the rowing motion and write to a trc (track row column) file
          we pretend the boat is not moving.

        All heigths are wrt the boat frame.

"""

# parameters
nmbrstrokes = 3
strokerate = 10

Hz = 50
# time steps for each complete stroke
nmbrframes = int(60/strokerate*Hz)

frontPos = np.zeros((len(markers), 3))
backPos = np.zeros((len(markers), 3))


def normalize(v):
    """ normalize vector v, should be non-zero!  """
    return v / np.sqrt(np.sum(v**2))


def setfrontPos(name, place):
    """ Set trajectory points for front position"""
    frontPos[markers.index(name)] = place


def setbackPos(name, place):
    """ Set trajectory points for front position"""
    backPos[markers.index(name)] = place


def frontposition(bbaan, blheight=0, frontangle=pi/2 - 0.3):
    """ Determine the position of the rower a the start of the stroke 

    Args:
        blheight: height of the blade above the water
        frontangle: angle of lower leg wrt horizontal
    """
    
    # markers are described wrt to ground!
    #  we assume that the boat is laying on the water
    setfrontPos('mBoat', (0, 0, 0))

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
    setfrontPos('mSeat', (hpos, boatHeight+seatHeight, 0))
    print("f hpos", hpos)
    
    # shoulder position (lower back vertical, upper back bend angleinb forward)
    shposvert = lbackl + (ubackl+shoulderth/2) * math.cos(angleinb)
    shposhor  = hpos - (ubackl+shoulderth/2) * math.sin(angleinb)
    setfrontPos('mShoulder', (shposhor, boatHeight+3*seatHeight/2+shposvert, 0))

    """ Arm positions,
          arm stretched, blade on the water: hand is on a sphere, and handle on a horizontal circle.
          find the intersection of sphere and circle
          https://gamedev.stackexchange.com/questions/75756/sphere-sphere-intersection-and-circle-sphere-intersection
    """

    # Starboard side
    # center and radius of sphere
    shleftpos = (shposhor, boatHeight+3*seatHeight/2+shposvert, shoulderl/2)
    shoulderleft = np.array(shleftpos)
    sphererad = uarml + larml

    #  place of (frame of) lock (0.02 higher than port side)
    lheight = boatHeight+seatHeight+lockHeight + 0.025 + 0.02
    # lockplace = np.array((place, lheight, span/2))
    # handle height with blade  blheight above the water
    oarangle = math.sin((lheight-blheight)/(outboard-bladepoint))
    handleheight = lheight + inboard*math.asin(oarangle)

    # center and radius of handle circle
    circ_c = np.array([place, handleheight, span/2])
    circrad = inboard*math.cos(oarangle)
    # normal of circle plane
    handle_normal = np.array([0, 1, 0])

    # plane of the circle cuts the sphere d = dot(n, c_c - c_s) units from the sphere's center
    dcut = np.dot(handle_normal, circ_c-shoulderleft)
    if abs(dcut) >= sphererad:
        print("sphere does not intersect circle, exiting.")
    # circle cut by plane of original circle
    c_p = shoulderleft + dcut*handle_normal
    r_p = math.sqrt(sphererad*sphererad - dcut*dcut)

    # a different d than dcut.
    d = np.linalg.norm(circ_c - c_p)
    h = 1/2 + (circrad * circrad - r_p * r_p)/(2 * d*d)

    c_i = circ_c + h * (c_p - circ_c)

    t = normalize(np.cross(c_p - circ_c, handle_normal))

    # intersects
    p_0 = c_i - t * r_p
    p_1 = c_i + t * r_p

    # p_0 intersection is the one
    markerconnect = (p_0[0], p_0[1], p_0[2])
    markerlockcirc = (circ_c[0], circ_c[1], circ_c[2])
    markerbolcirc = (c_p[0], c_p[1], c_p[2])

    #
    setfrontPos('msHandle', p_0)
    # the arm is straight
    setfrontPos('msElbow', ((p_0+shoulderleft)/2))

    """ blades horizontal
          y = blheight, x and z from: handle, lock, inboard, (outboard-bladepoint)

    """
    ratio = (outboard-bladepoint)/inboard
    horoarangle = math.atan((place - p_0[0])/(span/2 - p_0[2]))
    # connection of blade to the oar
    x = place + (place - p_0[0])*ratio
    z = span/2 + (span/2 - p_0[2])*ratio
    msbladepos = (x+bladepoint*math.cos(horoarangle), blheight, z+bladepoint*math.sin(horoarangle))
    setfrontPos('msBlade', msbladepos)


    # Port side  (right)
    # center and radius of sphere
    shrightpos = (shposhor, boatHeight+3*seatHeight/2+shposvert, -shoulderl/2)
    shoulderright = np.array(shrightpos)
    sphererad = uarml + larml

    #  place of (frame of) lock
    rheight = boatHeight+seatHeight+lockHeight + 0.025
    rlockplace = np.array((place, rheight, -span/2))
    # handle height with blade  blheight above the water
    oarangle = math.sin((rheight-blheight)/(outboard-bladepoint))
    handleheight = rheight + inboard*math.asin(oarangle)

    # center and radius of circle
    circ_c = np.array([place, handleheight, -span/2])
    circrad = inboard*math.cos(oarangle)
    # normal of circle plane
    handle_normal = np.array([0, 1, 0])

    # plane of the circle cuts the sphere d = dot(n, c_c - c_s) units from the sphere's center
    dcut = np.dot(handle_normal, circ_c-shoulderright)
    if abs(dcut) >= sphererad:
        print("sphere does not intersect circle, exiting.")
    # circle cut by plane of original circle
    c_p = shoulderright + dcut*handle_normal
    r_p = math.sqrt(sphererad*sphererad - dcut*dcut)

    # a different d than dcut.
    d = np.linalg.norm(circ_c - c_p)
    h = 1/2 + (circrad * circrad - r_p * r_p)/(2 * d*d)
    c_i = circ_c + h * (c_p - circ_c)

    t = normalize(np.cross(c_p - circ_c, handle_normal))

    p_0 = c_i - t * r_p
    p_1 = c_i + t * r_p

    # p_1 intersection is the one
    pmarkerconnect = (p_1[0], p_1[1], p_1[2])
    pmarkerlockcirc = (circ_c[0], circ_c[1], circ_c[2])
    pmarkerbolcirc = (c_p[0], c_p[1], c_p[2])

    #
    setfrontPos('mpHandle', p_1)
    # the arm is straight
    setfrontPos('mpElbow', ((p_1+shoulderright)/2))

    ratio = (outboard-bladepoint)/inboard
    horoarangle = math.atan((place - p_1[0])/(-span/2 - p_1[2]))
    x = place + (place - p_1[0])*ratio
    z = -span/2 + (-span/2 - p_1[2])*ratio
    mpbladepos = (x+bladepoint*math.cos(horoarangle), blheight, z+bladepoint*math.sin(horoarangle))
    setfrontPos('mpBlade', mpbladepos)

    # create markers for testing
    (x, y, z) = markershoulder
    mkmarker = osim.Marker("FR_MSHPOS", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = shleftpos
    mkmarker = osim.Marker("FR_SHLEFTPOS", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = shrightpos
    mkmarker = osim.Marker("FR_SHRIGHTPOS", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = markerconnect  # hand to handle
    mkmarker = osim.Marker("FR_MARKERCONNECT", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = pmarkerconnect
    mkmarker = osim.Marker("FR_PMARKERCONNECT", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = msbladepos
    mkmarker = osim.Marker("MSBLADEPOS", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = mpbladepos
    mkmarker = osim.Marker("MPBLADEPOS", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)

    (x, y, z) = markerlockcirc
    mkmarker = osim.Marker("MARKERLOCKCIRC", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = pmarkerlockcirc
    mkmarker = osim.Marker("PMARKERLOCKCIRC", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = markerbolcirc
    mkmarker = osim.Marker("MARKERBOLCIRC", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = pmarkerbolcirc
    mkmarker = osim.Marker("PMARKERBOLCIRC", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)



    

def backposition(bbaan, blheight = 0):
    """ Determine the position of the rower a the finish of the stroke 
       eigenlijk 2 posities, boven en in het water
    """

    setbackPos('mBoat', (0, 0, 0))

    #   seat position (legs stretched)
    hpos = math.sqrt((ulegl + llegl)**2 - seatHeight**2)
    vpos = boatHeight+3*seatHeight/2
    setbackPos('mSeat', (hpos, vpos, 0))

    # knee position, halfway between stretcher and seat  (klopt niet helemaal, onder en bovenbeen niet evenlang)
    mkneepos = (hpos/2, boatHeight+seatHeight, 0)
    
    hipangle = 0.7
    # shoulder position (lower back angle at hipangle backwards)
    shposvert = lbackl*math.cos(hipangle) + (ubackl+shoulderth/2) * math.cos(hipangle-angleinb)
    shposhor  = hpos + lbackl*math.sin(hipangle) + (ubackl+shoulderth/2) * math.sin(hipangle-angleinb)
    markershoulder =  (shposhor, boatHeight+3*seatHeight/2+shposvert, 0)
    print("SDF" , markershoulder)
    setbackPos('mShoulder', markershoulder)

    """  Arm positions
           lower arm horizontal
           upper arm(s) alongside upper back (on plane with upper back)
         elbow in plane of handle and on circle of upper back
         handle on handle-circle
    """

    # Starboard side
    shleftpos = (shposhor, boatHeight+3*seatHeight/2+shposvert, shoulderl/2)
    shoulderleft = np.array(shleftpos)
    
    #  place of (frame of) lock (0.02 higher than port side)
    lheight = boatHeight+seatHeight+lockHeight + 0.025 + 0.02
    # lockplace = np.array((place, lheight, span/2))
    # handle height with blade  blheight above the water
    oarangle = math.sin((lheight-blheight)/(outboard-bladepoint))
    handleheight = lheight + inboard*math.asin(oarangle)
    ##### center and radius of handle circle
    circ_c = np.array([place, handleheight, span/2])
    circrad = inboard*math.cos(oarangle)
    # normal of circle plane
    handle_normal = np.array([0, 1, 0])
    markerlockcirc = (circ_c[0], circ_c[1], circ_c[2])

    # Elbow on circle in plane of upper back
    #  is elleboog langs arm lager dan hangleheight? zo niet dan elleboog helemaal op zijn laagste laten?
    if uarml*math.cos(hipangle) < shoulderleft[1] - handleheight:
        print("Cannot get elbow into plane of handle! Exiting")
        exit()
    #   uarml.cos(0.4-angleinb)*cos(x) = shoulderleft[1] - handleheight
    x = math.acos((shoulderleft[1] - handleheight)/(uarml*math.cos(hipangle-angleinb)))
    # x = outside angle of upper arm.

    ###### elbow center and radius for the hand
    selbowpos = (shposhor-uarml*math.sin(hipangle-angleinb), handleheight, shoulderl/2+(shoulderleft[1]-handleheight)*math.tan(x))
    setbackPos('msElbow', selbowpos)
    selbowrad = larml
    eb = np.array(selbowpos)
    markerconnect = selbowpos

    d = np.linalg.norm(circ_c - eb)
    h = 1/2 + (circrad * circrad - selbowrad * selbowrad)/(2 * d*d)

    c_i = circ_c + h * (eb - circ_c)

    # hand and handle both on a (horizontal) circle, find intersection.
    t = normalize(np.cross(eb - circ_c, handle_normal))

    p_0 = c_i - t * selbowrad
    p_1 = c_i + t * selbowrad
    setbackPos('msHandle', p_0)
    # altijd deze?
    
    ratio = (outboard-bladepoint)/inboard
    horoarangle = math.atan((place - p_0[0])/-(-span/2 - p_0[2]))
    print("oar 1", horoarangle)
    # connection of blade to the oar
    x = place + (place - p_0[0])*ratio
    z = span/2 + (span/2 - p_0[2])*ratio
    msbladepos = (x+bladepoint*math.sin(horoarangle), blheight, z+bladepoint*math.cos(horoarangle))
    setbackPos('msBlade', msbladepos)

    # Port side
    shrightpos = (shposhor, boatHeight+3*seatHeight/2+shposvert, -shoulderl/2)
    shoulderright = np.array(shrightpos)
    
    #  place of (frame of) lock
    lheight = boatHeight+seatHeight+lockHeight + 0.025
    # plockplace = np.array((place, lheight, -span/2))
    # handle height with blade  blheight above the water
    oarangle = math.sin((lheight-blheight)/(outboard-bladepoint))
    handleheight = lheight + inboard*math.asin(oarangle)
    ##### center and radius of handle circle
    circ_c = np.array([place, handleheight, -span/2])
    circrad = inboard*math.cos(oarangle)
    # normal of circle plane
    handle_normal = np.array([0, 1, 0])
    pmarkerlockcirc = (circ_c[0], circ_c[1], circ_c[2])

    # Elbow on circle in plane of upper back
    #  is elleboog langs arm lager dan hangleheight? zo niet dan elleboog helemaal op zijn laagste laten?
    if uarml*math.cos(hipangle) < shoulderright[1] - handleheight:
        print("Cannot get elbow low enough? Exiting")
        exit()
    #   uarml.cos(0.4-angleinb)*cos(x) = shoulderright[1] - handleheight
    x = math.acos((shoulderright[1] - handleheight)/(uarml*math.cos(hipangle-angleinb)))
    # x = outside angle of upper arm.

    # elbow center and radius for the hand
    pelbowpos = (shposhor-uarml*math.sin(hipangle-angleinb), handleheight, -shoulderl/2-(shoulderleft[1]-handleheight)*math.tan(x))
    setbackPos('mpElbow', pelbowpos)
    pelbowrad = larml
    eb = np.array(pelbowpos)
    pmarkerconnect = pelbowpos

    d = np.linalg.norm(circ_c - eb)
    h = 1/2 + (circrad * circrad - pelbowrad * pelbowrad)/(2 * d*d)

    c_i = circ_c + h * (eb - circ_c)

    # hand and handle both on a (horizontal) circle, find intersection.
    t = normalize(np.cross(eb - circ_c, handle_normal))

    p_0 = c_i - t * selbowrad
    p_1 = c_i + t * selbowrad
    setbackPos('mpHandle', p_1)
    
    # angle wrt the normal of the boat
    horoarangle = math.atan((place - p_1[0])/(span/2 + p_1[2]))
    print("oar 2", horoarangle)
    x = place + (place - p_0[0])*ratio
    z = -span/2 + (-span/2 - p_0[2])*ratio
    mpbladepos = (x-bladepoint*math.cos(horoarangle), blheight, z+bladepoint*math.sin(horoarangle))
    setbackPos('mpBlade', mpbladepos)

    # create markers for testing
    (x, y, z) = markershoulder
    mkmarker = osim.Marker("MSHPOS", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = shleftpos
    mkmarker = osim.Marker("SHLEFTPOS", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = shrightpos
    mkmarker = osim.Marker("SHRIGHTPOS", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = markerconnect  # hand to handle
    mkmarker = osim.Marker("MARKERCONNECT", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = pmarkerconnect
    mkmarker = osim.Marker("PMARKERCONNECT", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = msbladepos
    mkmarker = osim.Marker("MSBLADEPOS", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)
    (x, y, z) = mpbladepos
    mkmarker = osim.Marker("MPBLADEPOS", bbaan.getGround(), osim.Vec3(x, y, z))
    bbaan.addMarker(mkmarker)


def main():
    trfile = Path.cwd() / 'trajectory.trc'
    with open(trfile, mode='w') as report_file:
        report_writer = csv.writer(report_file, dialect='excel-tab')

        # start of header
        report_writer.writerow(["PathFileType", 4, "(X/Y/Z)", "trajectory.trc"])
        report_writer.writerow(["DataRate", "CameraRate", "NumFrames", "NumMarkers", "Units", "OrigDataRate", "OrigDataStartFrame", "OrigNumFrames"])
        report_writer.writerow([Hz, Hz, nmbrframes*nmbrstrokes, nmbrmarkers, "m", Hz, 1, 10])
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

        bbaan = osim.Model("BootBaan.osim")
        bbaan.setName('Trajectory')

        #frontposition(bbaan, blheight=0.1, frontangle=pi/2 - 0.6)
        backposition(bbaan, blheight=0.1)
        #print(frontPos)
        #print(backPos)


        # Create model for testing
        bbaan.finalizeConnections()
        bbaan.printToXML("Trajectory.osim")


        time = 0
        for xx in range(1):
            # start of data
            for index in range(20):
                nextstring = [time, time/50]
                for x in backPos:
                    nextstring.append(f'{x[0]:.3f}')
                    nextstring.append(f'{x[1]:.3f}')
                    nextstring.append(f'{x[2]:.3f}')
                if index == 0:
                    print("The position:")
                    print(nextstring, "\n")
                report_writer.writerow(nextstring)
                time += 1
        """
            for index in range(10):
                nextstring = [time, time/50]
                for x in frontPos:
                    nextstring.append(f'{x[0]:.3f}')
                    nextstring.append(f'{x[1]:.3f}')
                    nextstring.append(f'{x[2]:.2f}')
                if index == 0:
                    print("The position:")
                    print(nextstring, "\n")
                report_writer.writerow(nextstring)
                time += 1



        for i in range(nmbrstrokes):
            for j in range(nmbrframes):
                index = i*nmbrframes + j + startindex
                nextstring = [index, index/50]
                for x in frontPos:
                    nextstring.append(f'{x[0]:.3f}')
                    nextstring.append(f'{x[1]:.3f}')
                    nextstring.append(f'{x[2]:.2f}')
                if i==0 and j==0:
                    print("The position:")
                    print(nextstring, "\n")
                report_writer.writerow(nextstring)
        """
    
if __name__ == "__main__":
    main()

