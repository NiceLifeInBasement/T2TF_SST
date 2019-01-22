#!/usr/bin/env python
from __future__ import print_function
"""
Plays a bagfile from a selected list of files that were manually selected.
Additional CAM messages for all visible vehicles are simulated. Using these and the original CAM messages, T2TA between
C2X messags and lidar scan object tracking is performed.

This is based on historic_assoc.py.

List of all bag files that can be used: (enter the number before it as the command line parameter to select it)
0. cut130_142.bag
    Is a cut-down of mavenNew_large.bag to only the time between seconds 130 to 142 of that bag.
    Scenario: Car passing the (real) c2x vehicle in front of the ego car
"""

from tracking_visuals import *
import matplotlib.pyplot as plt
import subprocess
from tf2_msgs.msg import TFMessage
import threading as thr
import tf
from geometry_msgs.msg import PointStamped, Point
from simulation.sim_classes import SimulatedVehicle
import general.t2ta_algorithms as t2ta
import os
from general.t2t_history import *
import copy
import tf_conversions as tf_c
import pickle


# --- method that converts a lidar scan to a simulated c2x measurement and returns this
def to_c2x_measurement(original_data):
    """
    Converts a lidar scan to a simulated c2x measurement and returns this.
    :param original_data: TrackedOrientedBox that should be used as a basis for this
    :return: A TrackedOrientedBox that is a simulated c2x message based on the input.
    """
    global pos_stddev  # The stddev that should be used for changing the positional data of the box
    scan = copy.deepcopy(original_data)
    scan.object_id = scan.object_id * 100  # Add 2 0s to the object id to make distinguishable from the original data
    x_new = np.random.normal(loc=scan.box.center_x, scale=pos_stddev)
    y_new = np.random.normal(loc=scan.box.center_y, scale=pos_stddev)

    scan.box.center_x = x_new
    scan.box.center_y = y_new

    return scan


# --- callback functions
def callback_tracking(data):
    """
    Plot data from the original laser scans and from the c2x scan to the visuals object and additionally perform
    T2TA on this data, printing the results to the command line
    """
    # insert the basic code like in general_plot.py here to display everything etc
    # additionally, add the t2t_history code that updates the t2th object and calls the t2ta_historic code
    global visuals, lock, transforms, steps, c2x, inc_c2x, history, t2ta_thresh, state_space, use_identity, c2x_offset_x, c2x_offset_y, c2x_offset_test, no_bag
    lock.acquire()
    c2x_selection = closest_match(c2x, data.header.stamp)
    if c2x_selection is not None:
        # Remove the selected c2x entry from the list of all c2x entries so that it doesn't get used twice
        c2x.remove(c2x_selection)

    global plot_bounding_boxes
    if plot_bounding_boxes:
        visuals.plot_box_array_rectangles(data.boxes, color='b', append=False)
    else:
        visuals.plot_box_array(data.boxes, append=False)

    # Append the current information to the history object
    history.add("lidar_0", data.boxes)

    # ---
    # SIMULATION OF ADDITIONAL C2X entry
    if c2x_selection is not None:
        # here, the incoming data is used to simulate additional c2x messages.
        # For that, it's necessary to know which IDs belong to cars in the current bag. Since this can not be extracted
        # from the bag data, a hard coded list is used instead
        vehicle_ids = []  # List of all object_ids of objects that were identified as vehicles
        if no_bag == 0:
            vehicle_ids = [4784]
        elif no_bag == 1:
            vehicle_ids = [6077, 7736, 7739, 7755, 7756, 7770, 7773, 7772, 7785]  # [7781, 7886] maybe?

        # selected all ids, proceed to add additional lidar scans for this data
        for tracked_box in data.boxes:
            if tracked_box.object_id in vehicle_ids:
                extra_c2x = to_c2x_measurement(tracked_box)
                # Now, append this extra_c2x TrackedOrientedBox to the history object and also use it for the current
                # step of assoc etc
                c2x_selection.tracks.append(extra_c2x)
        global lidar_steps
        try:
            lidar_steps += 1
        except NameError:
            lidar_steps = 1
    # ---

    if c2x_selection is not None:
        # Also include c2x data in the plot

        # ---
        # The following is the c2x data transformation, where the c2x pos/vel etc are transformed into the same coord.
        # frame as the tracking data that was received in this time step.
        # For this, all tracks in the c2x data are transformed using the transformer object.
        # Afterwards, T2TA is attempted on the resulting data. Results of this are printed to console, but currently
        # not passed on in any way. A future implementation should probably include a publisher, that publishes the
        # resulting data to a new topic, that a t2tf client can subscribe to.
        # ---

        tracks = c2x_selection.tracks
        # transformer: tf.TransformerROS
        for track in tracks:
            time_obj = rospy.Time(0)
            x_pos = track.box.center_x
            y_pos = track.box.center_y
            # Experimental transform of box l/w, velocity
            x_vel = track.box.velocity_x
            y_vel = track.box.velocity_y
            length = track.box.length
            width = track.box.width
            try:
                next_point = (x_pos, y_pos, track.object_id, "y")
                if plot_bounding_boxes:
                    visuals.plot_box_array_rectangles([track], color="y", append=True)
                else:
                    visuals.plot_points_tuple([next_point], append=True)
            except tf.ExtrapolationException as e:
                # Extrapolation error, print but keep going (possible just because only one frame was received so far)
                print(e)
            except tf.ConnectivityException as e:
                # print("Connectivity Exception during transform")
                print(e)
        # End of for going over tracks
        steps += 1  # c2x was used in another step, increase the counter

        # Now to the T2TA with history:
        try:
            # generate object ids and sensor names
            lidar_ids = []
            c2x_ids = []
            for obj in data.boxes:
                lidar_ids.append(obj.object_id)
            for obj in c2x_selection.tracks:
                c2x_ids.append(obj.object_id)
            obj_ids = [lidar_ids, c2x_ids]

            sensor_names = ["lidar_0", "c2x_0"]

            # Data acquisition from the history object should be based on the most recently received objects stamp
            timing = data.boxes[0].box.header.stamp
            assoc = t2ta.t2ta_historic(obj_ids, sensor_names, t2ta_thresh, hist_size, history, time=timing,
                                       state_space=state_space, use_identity=use_identity)
            ids = []  # this list will hold lists where each entry is an object id in a cluster
            for a in assoc:  # get a list of all associations
                temp = []  # stores ids for one association
                for box in a:  # all tracking boxes in the current association
                    temp.append(box.object_id)
                ids.append(temp)  # ids is a list made up of lists of object ids
                if len(a) > 1:
                    # If a non-singleton cluster was found, print all ids that belong to it
                    global print_clusters  # Boolean that specifies whether to print clustering information
                    if print_clusters:
                        print("<<  Non-singleton Cluster: " + str(temp) + "  >>")
                    # --- ADD_CAM specific
                    # additionally, maintain a counter of how many associations were correct
                    global assoc_correct
                    try:  # init global list if it hasn't happened before
                        assoc_correct
                    except NameError:
                        assoc_correct = []  # Array that stores 1 for a correct assoc and 0 for a bad one
                    if 100 in temp:
                        pass  # "Real" CAM message, do not track since correct association
                    else:
                        if temp[0]*100 == temp[1]:
                            # follows the rule of id changed from the simulation: correct assoc
                            assoc_correct.append(1)
                        else:
                            assoc_correct.append(0)

                        # Boolean to determine whether information about the association should be printed:
                        show_assoc_information = len(assoc_correct) % 10 == 0  # Show assoc info every 10 steps
                        show_assoc_information = True  # Show assoc info in every step
                        if show_assoc_information:
                            percent_correct = float(sum(assoc_correct)) / len(assoc_correct) * 100
                            print("No. assoc: "+str(len(assoc_correct))+"\t"+"Correct: "+str(sum(assoc_correct))+"\t%CORRECT: "+str(percent_correct))
                            percent_of_total = float(len(assoc_correct)) / lidar_steps * 100
                            print("No. assoc: "+str(len(assoc_correct))+"\t"+"Possible: "+str(lidar_steps)+"\t%TOTAL: "+str(percent_of_total))
                            print("")
                # DEBUG
                # The following block checks for association clusters of size 2
                # and then proceeds to store the distance between the boxes in these clusters
                # Additionally, a print of the distance with the number of assoc. made is printed
                if len(a) == 2:
                    global avg_dist
                    dist = t2t_distance_box(box_a=a[0], box_b=a[1], state_space=state_space, use_identity=use_identity)
                    try:
                        avg_dist
                    except NameError:  # not yet initialized
                        avg_dist = []
                    avg_dist.append(dist)

                    # DEBUG
                    # print("No. assoc:"+str(len(avg_dist))+"\t Avg Distance: "+str(sum(avg_dist)/len(avg_dist)))

                # DYNAMIC OFFSET CHANGING
                # dynamically modify the offset of the c2x vehicle
                # in every step, check if reducing or increasing the offset by c2x_offset_test will improve the
                # distance between the 2 associated objects
                # if yes, modify the offset parameter by that value
                if c2x_offset_test != 0.0:
                    if len(a) == 2:  # TESTING INCREMENTAL OFFSET
                        # found an association between the two tracks
                        dist = t2t_distance_box(box_a=a[0], box_b=a[1], state_space=state_space,
                                                use_identity=use_identity)
                        # id == 100 in the c2x data for the ego-vehicle --> compare to that
                        # test_box_a will gets its value changed
                        if a[0].object_id == 100:
                            test_box_a = copy.deepcopy(a[0])
                            test_box_b = copy.deepcopy(a[1])
                        else:
                            test_box_b = copy.deepcopy(a[0])
                            test_box_a = copy.deepcopy(a[1])
                        # calc dist_up
                        test_box_a.box.center_x += c2x_offset_test
                        dist_up = t2t_distance_box(box_a=test_box_a, box_b=test_box_b,
                                                   state_space=state_space, use_identity=use_identity)
                        # calc dist_down (subtract c2x_offset_test twice, because you added it once previously)
                        test_box_a.box.center_x -= 2 * c2x_offset_test
                        dist_down = t2t_distance_box(box_a=test_box_a, box_b=test_box_b,
                                                     state_space=state_space, use_identity=use_identity)
                        if dist_up < dist:
                            # found an improvement by increasing the distance
                            c2x_offset_x += c2x_offset_test
                            print("Offset++ ->" + str(c2x_offset_x) + "\t distance -:" + str(dist - dist_up))
                        if dist_down < dist:
                            # found an improvement by increasing the distance
                            c2x_offset_x -= c2x_offset_test
                            print("Offset-- ->" + str(c2x_offset_x) + "\t distance -:" + str(dist - dist_down))
                # END OF DYNAMIC CHANGING
        except ValueError as e:
            print("ValueError during association")
            # print(e)
        except IndexError as e:
            #print(e)
            print("IndexError during association, likely because not enough data was received yet.")


    lock.release()


def callback_tf_static(data):
    """
    Acquires static tf data and adds it to the transformer object.
    """
    global transformer

    for tf_obj in data.transforms:
        # Force time == 0 or you will run into issues TODO time==0 force here
        tf_obj.header.stamp = rospy.Time(0)
        transformer.setTransform(tf_obj)


def callback_c2x_tf(data):
    """
    Stores the last c2x message, but transforms it beforehand

    currently does not include rotation of the bounding box length/width
    """
    global history, steps, constant_velo, c2x_offset_x, c2x_offset_y
    tracks = data.tracks
    head = SimulatedVehicle.create_def_header(frame_id=src_id)
    # transformer: tf.TransformerROS
    for track in tracks:
        time_obj = rospy.Time(0)
        x_pos = track.box.center_x
        y_pos = track.box.center_y
        x_vel = track.box.velocity_x
        y_vel = track.box.velocity_y
        try:
            # Try to transform the point
            # Create a point with z=0 for the source frame (out of c2x tracks x/y) coordinate
            point = PointStamped(header=head,
                                 point=Point(x=x_pos, y=y_pos, z=0))
            # Don't use the current timestamp, use 0 to use the latest available tf data
            point.header.stamp = rospy.Time(0)
            # Now transform the point using the data
            tf_point = transformer.transformPoint(target_frame=dest_id, ps=point)

            # Acquire the transformation matrix for this
            tf_mat = tf_c.toMatrix(tf_c.fromTf(transformer.lookupTransform(target_frame=dest_id, source_frame=src_id, time=rospy.Time(0))))
            # print(tf_mat)

            # tf_vel stores the transformed velocity
            tf_vel = np.dot(tf_mat[0:2, 0:2], [x_vel, y_vel])
            # Update the track with the transformed data
            track.box.center_x = tf_point.point.x + c2x_offset_x
            track.box.center_y = tf_point.point.y + c2x_offset_y
            track.box.velocity_x = tf_vel[0]
            track.box.velocity_y = tf_vel[1]
            # DEBUG
            # print(str(track.box.center_x)+"\t"+str(track.box.center_y))
            # print("steps: "+str(steps)+"\tvelx: "+str(point_vel.point.x)+" vely: "+str(point_vel.point.y))
        except tf.ExtrapolationException as e:
            # Extrapolation error, print but keep going (possible just because only one frame was received so far)
            print(e)
        except tf.ConnectivityException as e:
            # print("Connectivity Exception during transform")
            print(e)
        except tf.LookupException as e:
            print(e)
            pass
    # Now change the frame_id to dest_id, since the tf was already performed
    data.header.frame_id = dest_id

    c2x.append(data)
    history.add("c2x_0", data.tracks)  # Add the data to the history object under the name c2x_0

    # The following block adds several fake measurements to the c2x tracking that are time delayed to the original
    # measurement from the current timestep
    # They also have changed x/y data based on velocity, so that the track doesn't need to be reused later on with the
    # outdated coordinates and instead can work with "updated" coordinates for time steps that are not included in the
    # real c2x messages.
    add_points = 4  # how many "fake" points should be added between this and the next measurement
    # TODO consider changing the timing/number of fakes depending on the time diff between this data and the last data
    if len(data.tracks) > 0:  # only do the following if the track contained something
        for i in range(add_points):
            # add 4 more measurements, each based on the data, but with manipulated time and position based on velocity
            fake_data = copy.deepcopy(data)  # Create a copy of the data
            # now change the timestamp of this new data
            # c2x data arrives every 0.2 seconds, so to fit an additional 4 measurements in there
            time_shift = rospy.Duration(0, 40000000)  # increase by 1/20 of a sec
            fake_data.header.stamp = fake_data.header.stamp + time_shift
            for track in fake_data.tracks:
                track.box.header.stamp = track.box.header.stamp + time_shift
                track.box.center_x += constant_velo * track.box.velocity_x * (i+1)
                track.box.center_y += constant_velo * track.box.velocity_y * (i+1)
            c2x.append(fake_data)
            history.add("c2x_0", fake_data.tracks)

    steps = 0


def listener(fname, recreate_tf_static=True):
    """
    Prepare the subscribers and setup the plot etc
    :param fname: The name of the file to start
    :param recreate_tf_static: True if the tf_static message saved in data/tf_static_dump.pkl should be loaded.
    This needs to be set to True if the bag that will be played does not contain the single tf_static msg that is
    usually right at the beginning of the bag file.
    """
    global visuals, transformer
    rospy.init_node('listener_laser_scan', anonymous=True)

    transformer = tf.TransformerROS(True)
    # print("FrameList:\t" + transformer.allFramesAsString())
    # Start the subscriber(s)
    rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback_tracking)  # General subscriber (tracking data)

    # Currently using _static here because plotting should happen upon receiving lidar data (incl c2x plotting)
    rospy.Subscriber("tf", TFMessage, callback_tf_static)  # Acquire transform messages

    rospy.Subscriber("/FASCarE_ROS_Interface/car2x_objects", TrackedOrientedBoxArray, callback_c2x_tf)
    rospy.Subscriber("tf_static", TFMessage, callback_tf_static)  # Acquire static transform message for "ibeo" frames
    # rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback_org_data)

    # now start a rosbag play for that filename
    FNULL = open(os.devnull, 'w')  # redirect rosbag play output to devnull to suppress it

    play_rate = 0.15  # set the number to whatever you want your play-rate to be
    # play_rate = 1
    rate = '-r' + str(play_rate)
    # using '-r 1' is the usual playback speed - this works, but since the code lags behind (cant process everything
    # in realtime), you will then get results after the bag finished playing (cached results)
    # using '-r 0.25' is still too fast for maven-1.bag
    # using '-r 0.2' works (bag finishes and no more associations are made on buffered data afterwards)

    start_time = 0  # time at which the bag should start playing
    time = '-s ' + str(start_time)

    if recreate_tf_static:
        pkl_filename = "./src/T2TF_SST/data/"  # folder
        pkl_filename += "tf_static_dump.pkl"  # filename
        with open(pkl_filename, 'rb') as pklinput:
            tf_static_data = pickle.load(pklinput)
            callback_tf_static(tf_static_data)

    player_proc = subprocess.Popen(['rosbag', 'play', rate, time, fname], cwd="./", stdout=FNULL)

    plt.show()  # DON'T NEED TO SPIN IF YOU HAVE A BLOCKING plt.show

    # Kill the process (if it was started)
    player_proc.terminate()


def setup():
    """
    Setup all global variables and start the listener for all necessary topics
    """
    global steps, src_id, dest_id, c2x, lock, history, hist_size, state_space, use_identity, transforms, t2ta_thresh, constant_velo, visuals, c2x_offset_x, c2x_offset_y, c2x_offset_test, no_bag
    steps = 0  # how many steps passed since the last time the c2x message was used
    src_id = "odom"  # transformations are performed FROM this frame
    dest_id = "ibeo_front_center"  # transformations are performed TO this frame
    c2x = []  # Array that is used to store all incoming c2x messages
    lock = thr.Lock()
    history = TrackingHistory()
    # hist_size = rospy.Duration(0) => history will be only 1 track (i.e. no history)
    # hist size Duration(4) causes significant lag already!
    hist_size = rospy.Duration(0, 500000000)  # .5 secs
    # hist_size = rospy.Duration(0)

    state_space = (True, False, False, False)  # usual state space: (TFFT), only pos: (TFFF)
    # The threshold NEEDS TO BE ADJUSTED if you use something other than TFFF!

    use_identity = True

    transforms = []
    # Create a new Visualization object with the axis limits and "blue" as default plotting color
    visuals = TrackVisuals(limit=65, neg_limit=-40, limit_y=50, neg_limit_y=-40, color='b')

    # define a similarity function for t2ta
    # Init the similarity checker that provides the similarity function
    # 20 works safe, but leads to some wrong associations between c2x data and road boundaries in very few scenarios
    # reduced it to 13 for now, but without doing extensive testing
    t2ta_thresh = 13  # Threshold for using only position data
    # t2ta_thresh = 25  # Threshold for using position + velocity data

    # value that is multiplied with velocity to acquire the position of the c2x objects in "untracked" time steps
    # i.e. between messages (since the frequency of the messages is lacking)
    # factor that needs to be used for velo when looking at change between 2 consecutive steps
    constant_velo = 0.05  # 0.05 performs best in my tests using maven-1.bag (if using c2x_offset_x=4)
    # The new transformation of velocity using the extracted matrix works, so you can now use a "normal" factor.
    # Constant offsets for all c2x data
    # TODO possibly "overfitting", but works incredibly well on both bag files
    # I assume the reason that this works/is necessary could be because the trackers have different "centers of tracking
    # boxes, and therefore this offset is necessary to "skip" the distance between the points
    c2x_offset_x = 4
    c2x_offset_y = 0

    c2x_offset_test = 0.0  # value that will be added+subtracted in every step to improve the above offset_x

    global plot_bounding_boxes, print_clusters
    plot_bounding_boxes = False
    print_clusters = False

    # Select the name of the bagfile from the list of allowed files
    # The list is intentionally hard-coded
    list_bagnames = ["data/cut130_142.bag", "data/cut200_205.bag"]
    no_bag = 0
    global pos_stddev
    if len(sys.argv) < 3:
        print("Arguments need to be \"<file_id> <measurement stddev>\"")
        print("Defaulting to \"0 0.05\"")
        no_bag = 0
        pos_stddev = 0.1
    else:
        no_bag = int(sys.argv[1])
        pos_stddev = float(sys.argv[2])

    fname = list_bagnames[no_bag]  # Select the bag from the list of files

    # Now setup the start
    listener(fname)


if __name__ == '__main__':
    setup()
