import opensim as osim
import math, csv
import numpy as np
from pathlib import Path, PurePath

pi = math.pi

"""     Create the pusher motion and write to a trc (track row column) file

        push beweging
        =============

          nu tov ground. beter tov boot. Dan kunnen we die makkelijk verplaatsen

          vind (kneeangle en) positie bij baseangle = 0: dat is begin van de push:   3.8
          dan bij het eind van de haal:   6.0
          dan de markerposities genereren en klaar
          alleen de tijdstippen erbij zijn niet wat we willen, er moet nog iets van een snelheidscurve bepaald worden?

        Been u(pper) en l(ower) leg
        Blad horizontaal op hoogte h en positie p
        kneex = 5 + u*sin(-baseangle)
        kneez = 0.3 + u*cos(-baseangle)
        p = kneex + l*cos(-baseangle-kneeangle)
        h = kneez - l*sin(-baseangle-kneeangle)
        gegeven p en h, reken uit baseangle en kneeangle

        wat vereenvoudigd:
        kneex = 5 - u*sin(baseangle)
        kneez = 0.3 + u*cos(baseangle)
        p = kneex + l*cos(baseangle+kneeangle)
        h = kneez + l*sin(baseangle+kneeangle)

        symbolisch lukt niet? Mathematica op de rpi of maxima.

        u = 1
        l = 1.5
        for x in range(20):
            for y in range(45):
                baseangle = -x/10; kneeangle = 1.5-y/15;
                kneex = 5+u*math.sin(-baseangle); kneez = 0.3+u*math.cos(-baseangle)
                print("baseangle: ", math.degrees(baseangle), ", kneeangle: ", math.degrees(kneeangle), ", kneex: ", kneex, ", kneez: ", kneez)
                print("p: ", kneex+l*math.cos(-baseangle-kneeangle), ", h: ", kneez-l*math.sin(-baseangle-kneeangle))
                print()
        """

# parameters
nmbrstrokes = 6
strokerate = 30
recover_to_stroke = 2.0

Hz = 50
# time steps for each complete stroke
nmbrframes = int(60/strokerate*Hz)

# global variables
time = 0
report_writer = None

"""
   take 6 timesteps for exit and entry of the blade each
   h+r+12 = nmbrframes
   recover_to_stroke*(h+6) = r+6

   h = nmbrframes - 12 - r
   r = 2*(h+6) - 6 = 2h + 6
   h = nmbrframes - 12 - (2h + 6)
   h = (nmbrframes - 12 - 6)/3

"""
strsteps = int((nmbrframes - 18)/3)
# distance per timestep
str_inc = (6.0-3.8)/strsteps
recsteps = 2*strsteps + 6
# distance per timestep
rec_inc = (6.0-3.8)/recsteps

# height of blade above ground
height = -0.00
# in recover
recheight = 0.3

markers = ['mbladej', 'mend']
nmbrmarkers = len(markers)

frontPos = np.zeros((len(markers), 3))
backPos = np.zeros((len(markers), 3))

mkneepos = (0, 0, 0)

def setfrontPos(name, place):
    """ Set trajectory points for front position"""
    frontPos[markers.index(name)] = place

def stroke():
    global time
    bladepos = 3.8
    myheight = height
    while True:
        # fill frontPos
        setfrontPos('mbladej', (bladepos, myheight, 0))
        setfrontPos('mend', (bladepos+0.225, myheight, 0))
        myheight -= 0 #0.1/20
        # 
        nextstring = [time, time/Hz]
        for x in frontPos:
            nextstring.append(f'{x[0]:.4f}')
            nextstring.append(f'{x[1]:.4f}')
            nextstring.append(f'{x[2]:.4f}')
        # print(nextstring)
        report_writer.writerow(nextstring)
        time += 1
        bladepos += str_inc
        if bladepos > 6.0:
            break


def exitblade():
    global time

    def tst(x, y):
        global time
        setfrontPos('mbladej', (x, y, 0))
        setfrontPos('mend', (x+0.225, y, 0))
        nextstring = [time, time/Hz]
        for x in frontPos:
            nextstring.append(f'{x[0]:.4f}')
            nextstring.append(f'{x[1]:.4f}')
            nextstring.append(f'{x[2]:.4f}')
        # print(nextstring)
        report_writer.writerow(nextstring)
        time += 1

    bladepos = 6.0
    # in 6 stapjes omkeren, ad hoc
    inc = (recheight-height)/6

    tst(bladepos+0.01, height + inc)
    tst(bladepos+0.02, height + 2*inc)
    tst(bladepos+0.03, height + 3*inc)
    tst(bladepos+0.03, height + 4*inc)
    tst(bladepos+0.02, height + 5*inc)
    tst(bladepos+0.01, height + 6*inc)


def recover():
    global time
    bladepos = 6.0
    while True:
        # fill frontPos
        setfrontPos('mbladej', (bladepos, recheight, 0))
        setfrontPos('mend', (bladepos+0.225, recheight, 0))
        # 
        nextstring = [time, time/Hz]
        for x in frontPos:
            nextstring.append(f'{x[0]:.4f}')
            nextstring.append(f'{x[1]:.4f}')
            nextstring.append(f'{x[2]:.4f}')
        # print(nextstring)
        report_writer.writerow(nextstring)
        time += 1
        bladepos -= rec_inc
        if bladepos < 3.8:
            break


def entryblade():
    global time

    def tst(x, y):
        global time
        setfrontPos('mbladej', (x, y, 0))
        setfrontPos('mend', (x+0.225, y, 0))
        nextstring = [time, time/Hz]
        for x in frontPos:
            nextstring.append(f'{x[0]:.4f}')
            nextstring.append(f'{x[1]:.4f}')
            nextstring.append(f'{x[2]:.4f}')
        # print(nextstring)
        report_writer.writerow(nextstring)
        time += 1

    bladepos = 3.8
    # in 6 stapjes omkeren, ad hoc
    inc = (recheight-height)/6

    tst(bladepos-0.01, recheight - inc)
    tst(bladepos-0.02, recheight - 2*inc)
    tst(bladepos-0.03, recheight - 3*inc)
    tst(bladepos-0.03, recheight - 4*inc)
    tst(bladepos-0.02, recheight - 5*inc)
    tst(bladepos-0.01, recheight - 6*inc)


def main():
    global time, report_writer
    trfile = Path.cwd() / 'trajectory.trc'
    with open(trfile, mode='w') as report_file:
        report_writer = csv.writer(report_file, dialect='excel-tab')

        # Create header of trc file
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

        bbaan = osim.Model("Pusher.osim")
        bbaan.setName('Trajectory')

        # generate marker positions from 3.8 to 6.0 meter (wrt boat origin)
        # stapjes van 1cm
        #  Pas op wat is de tijd die er bij hoort?
        #  en de hele cyclus een aantal keren doen

        time = 0
        for h in range(nmbrstrokes):
            stroke()
            exitblade()
            recover()
            entryblade()
        print("End time: ", time/50, " seconds.")

        # create markers for testing
        (x, y, z) = mkneepos
        mkmarker = osim.Marker("mkneepos", bbaan.getGround(), osim.Vec3(x, y, z))
        bbaan.addMarker(mkmarker)

        # Create model for testing
        # bbaan.finalizeConnections()
        # bbaan.printToXML("Trajectory.osim")


if __name__ == "__main__":
    main()

