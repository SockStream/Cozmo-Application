import cozmo
from cozmo.util import degrees, radians, distance_mm, speed_mmps
import queue
import time
import math
import cv2
from datetime import datetime
import random
import signal
import sys
from PIL import ImageDraw, ImageFont

class Battery(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        batt = self.world.robot.battery_voltage
        text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='green')
        text.render(d, bounds)

class MCP():
    def play_animation(self,anim_trig, body = False, lift = False, parallel = False):
    # anim_trig = cozmo.anim.Triggers."Name of trigger" (this is an object)
    # Refer to "http://cozmosdk.anki.com/docs/generated/cozmo.anim.html#cozmo.anim.Triggers" for animations' triggers
    # Refer to "http://cozmosdk.anki.com/docs/generated/cozmo.robot.html#cozmo.robot.Robot.play_anim_trigger" for playing the animation

        self.robot.play_anim_trigger(anim_trig,loop_count = 1, in_parallel = parallel,
                                num_retries = 0, use_lift_safe = False,
                                ignore_body_track = body, ignore_head_track=False,
                                ignore_lift_track = lift).wait_for_completed()

    def frustrated(self):
        trigger = cozmo.anim.Triggers.FrustratedByFailureMajor  
        self.play_animation(trigger)

    def celebrate(self):
        trigger = cozmo.anim.Triggers.CodeLabCelebrate  
        self.play_animation(trigger,body=True,lift=True,parallel=True)

    def happy(self):
        trigger = cozmo.anim.Triggers.CodeLabHappy
        self.play_animation(trigger,body=True,lift=True,parallel=True)
    def shutdown(self):
        trigger = cozmo.anim.Triggers.GoToSleepSleeping
        self.play_animation(trigger,parallel=False)
    def wake_up(self):
        trigger = cozmo.anim.Triggers.GoToSleepGetOut
        self.play_animation(trigger,parallel=False)

    def sleepy(self):
        trigger = cozmo.anim.Triggers.Sleeping 
        self.play_animation(trigger,parallel=False)

    def shutdown_robot(self,sig, frame):
        self.interrupted = True
        self.robot.abort_all_actions(True)
        self.shutdown()

    def ParlerDecompose(self,texte):
        liste = texte.split();
        for str in liste:
            self.Parler(str)

    def Parler(self,texte):
        self.robot.say_text(texte,use_cozmo_voice=True,voice_pitch=0.0).wait_for_completed()

    def gestion_occupation(self):
        maintenant = datetime.now()
        delta = maintenant - self.derniere_occupation_time
        nb_secondes = delta.total_seconds()
        liste_actions_possibles = []
        if (nb_secondes > self.delai_occupation) :
            if (not self.robot.is_charging):
                liste_actions_possibles.append("ROCKY")
                liste_actions_possibles.append("ROULER_CUBE")
            liste_actions_possibles.append("CHANTER")
            numero_action = random.randint(0,len(liste_actions_possibles) - 1)
            self.ListeNormale.append(liste_actions_possibles[numero_action])

    def gestion_visages(self):
        if (not self.robot.is_charging):
            self.robot.set_head_angle(cozmo.robot.MAX_HEAD_ANGLE).wait_for_completed()
        #print(self.robot.world.visible_face_count())
        if ( self.robot.world.visible_face_count() > 0 and not 'VISAGE' in self.ListeNormale):
            self.ListeNormale.append("VISAGE")

    def gestion_volume(self):
        maintenant = datetime.now()
        if (self.be_quiet):
            self.robot.set_robot_volume(0.1)
        elif (maintenant.hour >= 12 and maintenant.hour <14):
            self.robot.set_robot_volume(0.5)
        else:
            self.robot.set_robot_volume(1)

    def gestion_batterie(self):
        #print(self.robot.battery_voltage)
        if (self.robot.battery_voltage < self.threshold_min and not self.robot.is_charging):
            self.QueuePrioritaire.put("CHARGER")
        if (self.robot.battery_voltage > self.threshold_max and self.robot.is_charging) :
            self.QueuePrioritaire.put("SORTIR_CHARGEUR")

    def find_charger(self):

        while(True):
            behavior = self.robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
            try: 
                seen_charger = self.robot.world.wait_for_observed_charger(timeout=10,include_existing=True)
            except:
                seen_charger = None
            behavior.stop()
            if(seen_charger != None):
                #print(seen_charger)
                return seen_charger
            self.frustrated()
            self.robot.say_text('Charge?',duration_scalar=0.5).wait_for_completed()
            time.sleep(2)
        return None

    def check_tol(self,charger: cozmo.objects.Charger,dist_charger=40):
        # Check if the position tolerance in front of the charger is respected
        global PI

        distance_tol = 5 # mm, tolerance for placement error
        angle_tol = 5*PI/180 # rad, tolerance for orientation error

        try: 
            charger = self.robot.world.wait_for_observed_charger(timeout=2,include_existing=True)
        except:
            print('WARNING: Cannot see the charger to verify the position.')

        # Calculate positions
        r_coord = [0,0,0]
        c_coord = [0,0,0]
        # Coordonates of robot and charger
        r_coord[0] = self.robot.pose.position.x #.x .y .z, .rotation otherwise
        r_coord[1] = self.robot.pose.position.y
        r_coord[2] = self.robot.pose.position.z
        r_zRot = self.robot.pose_angle.radians # .degrees or .radians
        c_coord[0] = charger.pose.position.x
        c_coord[1] = charger.pose.position.y
        c_coord[2] = charger.pose.position.z
        c_zRot = charger.pose.rotation.angle_z.radians

        # Create target position 
        # dist_charger in mm, distance if front of charger
        c_coord[0] -=  dist_charger*math.cos(c_zRot)
        c_coord[1] -=  dist_charger*math.sin(c_zRot)

        # Direction and distance to target position (in front of charger)
        distance = math.sqrt((c_coord[0]-r_coord[0])**2 + (c_coord[1]-r_coord[1])**2 + (c_coord[2]-r_coord[2])**2)

        if(distance < distance_tol and math.fabs(r_zRot-c_zRot) < angle_tol):
    	    return 1
        else: 
    	    return 0
    def clip_angle(self,angle=3.1415):
        # Allow Cozmo to turn the least possible. Without it, Cozmo could
        # spin on itself several times or turn for instance -350 degrees
        # instead of 10 degrees. 
        global PI

        # Retreive supplementary turns (in radians)
        while(angle >= 2*PI):
            angle -= 2*PI
        while(angle <= -2*PI):
            angle += 2*PI
        # Select shortest rotation to reach the target
        if(angle > PI):
            angle -= 2*PI
        elif(angle < -PI):
            angle += 2*PI
        return angle

    def final_adjust(self,charger: cozmo.objects.Charger,dist_charger=40,speed=40,critical=False):
        # Final adjustement to properly face the charger.
        # The position can be adjusted several times if 
        # the precision is critical, i.e. when climbing
        # back onto the charger.  
        global PI

        while(True):
            # Calculate positions
            r_coord = [0,0,0]
            c_coord = [0,0,0]
            # Coordonates of robot and charger
            r_coord[0] = self.robot.pose.position.x #.x .y .z, .rotation otherwise
            r_coord[1] = self.robot.pose.position.y
            r_coord[2] = self.robot.pose.position.z
            r_zRot = self.robot.pose_angle.radians # .degrees or .radians
            c_coord[0] = charger.pose.position.x
            c_coord[1] = charger.pose.position.y
            c_coord[2] = charger.pose.position.z
            c_zRot = charger.pose.rotation.angle_z.radians

            # Create target position 
            # dist_charger in mm, distance if front of charger
            c_coord[0] -=  dist_charger*math.cos(c_zRot)
            c_coord[1] -=  dist_charger*math.sin(c_zRot)

            # Direction and distance to target position (in front of charger)
            distance = math.sqrt((c_coord[0]-r_coord[0])**2 + (c_coord[1]-r_coord[1])**2 + (c_coord[2]-r_coord[2])**2)
            vect = [c_coord[0]-r_coord[0],c_coord[1]-r_coord[1],c_coord[2]-r_coord[2]]
            # Angle of vector going from robot's origin to target's position
            theta_t = math.atan2(vect[1],vect[0])

            print('CHECK: Adjusting position')
            # Face the target position
            angle = self.clip_angle((theta_t-r_zRot))
            self.robot.turn_in_place(radians(angle)).wait_for_completed()
            # Drive toward the target position
            self.robot.drive_straight(distance_mm(distance),speed_mmps(speed)).wait_for_completed()
            # Face the charger
            angle = self.clip_angle((c_zRot-theta_t))
            self.robot.turn_in_place(radians(angle)).wait_for_completed()

            # In case the robot does not need to climb onto the charger
            if (not critical):
                break
            if self.check_tol(charger,dist_charger):
                print("CHECK: Robot aligned relativ to the charger.")
                break
        return

    def go_to_charger(self):
        # Driving towards charger without much precision

        charger = None
        ''' cf. 08_drive_to_charger_test.py '''
        # see if Cozmo already knows where the charger is
        if self.robot.world.charger:
            # make sure Cozmo was not delocalised after observing the charger
            if self.robot.world.charger.pose.is_comparable(self.robot.pose):
                print("Cozmo already knows where the charger is!")
                charger = self.robot.world.charger
            else:
                # Cozmo knows about the charger, but the pose is not based on the
                # same origin as the robot (e.g. the robot was moved since seeing
                # the charger) so try to look for the charger first
                pass
        if not charger:
            charger = self.find_charger()
    
        action = self.robot.go_to_object(charger,distance_from_object=distance_mm(80), in_parallel=False, num_retries=5)
        #action = self.robot.go_to_pose(charger.pose)
        action.wait_for_completed()
        return charger

    def chercher_cube(self):
        cube = None
        self.robot.set_lift_height(0).wait_for_completed()
        self.robot.set_head_angle(cozmo.util.radians(0)).wait_for_completed()
        look_around = self.robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
        print("Cozmo is waiting until he sees a cube.")
        try:
            cube = self.robot.world.wait_for_observed_light_cube(timeout=10)
        except :
            cube = None
        look_around.stop()
        return cube

    def gestion_actions(self):
        action = None
        if (not self.QueuePrioritaire.empty()):
            self.ListeNormale = []
            action = self.QueuePrioritaire.get()
        elif (len(self.ListeNormale) > 0):
            action = self.ListeNormale[0]
            del self.ListeNormale[0]

        if (action == "CHARGER"):
            self.robot.move_lift(-3)
            self.robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE).wait_for_completed()
            charger = self.go_to_charger()
            self.final_adjust(charger,critical=True)
            self.robot.turn_in_place(degrees(180)).wait_for_completed()
            self.robot.set_head_angle(cozmo.robot.MAX_HEAD_ANGLE).wait_for_completed()
            while (not self.robot.is_charging):
                self.robot.backup_onto_charger()
            #self.Parler("System Rebouting")
            self.sleepy();
        elif (action == "VISAGE"):
            latest_image = self.robot.world.latest_image

            #gestion de l'oubli des visages inconnus
            maintenant = datetime.now()
            delta = maintenant - self.date_dernier_bonjour_inconnu
            nb_secondes = delta.total_seconds()
            if (nb_secondes >= self.temps_oubli_inconnus and "" in self.liste_bonjours_dits):
                self.liste_bonjours_dits.remove("")

            if (latest_image is not None):
                faces_as_generator = self.robot.world.visible_faces
                faces_as_list = list(faces_as_generator)
                for visage in faces_as_list:
                    nom = visage.name
                    if (not nom and self.timide):
                        break

                    if (nom == self.master):
                        self.last_seen_pose = self.robot.pose

                    if (not nom in self.liste_bonjours_dits):
                        self.happy()
                        self.Parler("Bonjour " + visage.name)
                        self.liste_bonjours_dits.append(nom)
                        if (not nom):
                            self.date_dernier_bonjour_inconnu = datetime.now()

        elif (action == "SORTIR_CHARGEUR"):
            if (self.robot.is_charging):
                self.robot.drive_off_charger_contacts().wait_for_completed()
                time.sleep(2)
            self.robot.drive_straight(distance_mm(200), speed_mmps(50)).wait_for_completed()
            time.sleep(2)
            look_around = self.robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
            time.sleep(10)
            look_around.stop()
        elif (action == "CHANTER"):
            notes = [
                cozmo.song.SongNote(cozmo.song.NoteTypes.G2, cozmo.song.NoteDurations.Half),
                cozmo.song.SongNote(cozmo.song.NoteTypes.G2, cozmo.song.NoteDurations.Half),
                cozmo.song.SongNote(cozmo.song.NoteTypes.G2, cozmo.song.NoteDurations.Half),
                cozmo.song.SongNote(cozmo.song.NoteTypes.D2_Sharp, cozmo.song.NoteDurations.Quarter),
                cozmo.song.SongNote(cozmo.song.NoteTypes.A2_Sharp, cozmo.song.NoteDurations.Quarter),
                cozmo.song.SongNote(cozmo.song.NoteTypes.G2, cozmo.song.NoteDurations.Half),
                cozmo.song.SongNote(cozmo.song.NoteTypes.D2_Sharp, cozmo.song.NoteDurations.Quarter),
                cozmo.song.SongNote(cozmo.song.NoteTypes.A2_Sharp, cozmo.song.NoteDurations.Quarter),
                cozmo.song.SongNote(cozmo.song.NoteTypes.G2, cozmo.song.NoteDurations.Whole)
                ]
            # Play the ascending notes
            self.robot.play_song(notes, loop_count=1).wait_for_completed()

        elif (action == "ROCKY"):
            cube = self.chercher_cube()            
            if (cube is not None):
                print("Cozmo found a cube, and will now attempt to dock with it:")
                # Cozmo will approach the cube he has seen
                # using a 180 approach angle will cause him to drive past the cube and approach from the opposite side
                # num_retries allows us to specify how many times Cozmo will retry the action in the event of it failing
                action = self.robot.dock_with_cube(cube, approach_angle=None, distance_from_marker=distance_mm(0), in_parallel=False, num_retries=5)
                action.wait_for_completed()
                self.robot.drive_straight(distance_mm(20), speed_mmps(10)).wait_for_completed()
                self.robot.set_lift_height(1).wait_for_completed()
                time.sleep(1)
                self.robot.set_lift_height(0).wait_for_completed()
                time.sleep(1)
                self.robot.drive_straight(distance_mm(-20), speed_mmps(10)).wait_for_completed()
                self.robot.set_head_angle(cozmo.robot.MAX_HEAD_ANGLE).wait_for_completed()
        elif (action == "ROULER_CUBE"):
            cube = self.chercher_cube()
            if (cube is not None):
                self.robot.set_lift_height(1).wait_for_completed()
                action = self.robot.dock_with_cube(cube, approach_angle=None, distance_from_marker=distance_mm(10), in_parallel=False, num_retries=5)
                action.wait_for_completed()
                self.robot.drive_straight(distance_mm(10), speed_mmps(10)).wait_for_completed()
                self.robot.set_lift_height(0).wait_for_completed()
                self.robot.drive_straight(distance_mm(-80), speed_mmps(40)).wait_for_completed()


        if (action is not None):
            self.derniere_occupation_time = datetime.now()
                
    def __init__(self):
        self.QueuePrioritaire = queue.Queue()
        self.ListeNormale = []
        self.liste_bonjours_dits = []
        self.derniere_occupation_time = datetime.now()
        self.date_dernier_bonjour_inconnu = datetime.now()
        self.last_seen_pose = None
        self.interrupted = False
        ###CONFIGURATION#####
        self.temps_oubli_inconnus = 30 #temps (s) avant qu'il redise bonjour à un inconnu
        self.delai_occupation = 50 #temps (s) entre deux activités de Cozmo
        self.timide = False
        self.be_quiet = True
        self.master="clement"
        self.threshold_min = 3.6
        self.threshold_max = 4.58

    def Manage(self,robot : cozmo.robot.Robot):
        self.robot = robot
        self.robot.world.image_annotator.annotation_enabled = True
        self.robot.world.image_annotator.add_annotator('battery', Battery)
        self.wake_up()
        print(robot.battery_voltage)
        self.robot.set_lift_height(0).wait_for_completed()
        self.robot.drive_off_charger_contacts().wait_for_completed()
        self.robot.set_head_angle(cozmo.robot.MAX_HEAD_ANGLE).wait_for_completed()
        self.robot.backup_onto_charger()
        self.QueuePrioritaire.put("SORTIR_CHARGEUR")

        while(not self.interrupted):
            #gestion des actions
            self.gestion_volume()
            self.gestion_batterie()
            self.gestion_visages()
            self.gestion_occupation()
            
            self.gestion_actions()
            
PI = 3.14159265359
mcp = MCP()
signal.signal(signal.SIGINT, mcp.shutdown_robot)

cozmo.run_program(mcp.Manage,True,force_viewer_on_top=True)
