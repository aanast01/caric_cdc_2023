import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Duration
import csv
import time

score = [['detected,num_of_interest,score,time,mission_time']]
remaining = 10000000
TAG = '[REPORT SCRIPT]: '
count = 0


def set_tag(tag):
    global TAG
    TAG = tag

def log_info(info):
    global TAG, debug
    if debug:
        rospy.loginfo(TAG + info)
    #print(TAG)

def callback(msg):
    global score, count

    if count%1000==0:
        score.append([msg.text])
        count = 0
    count += 1

def timeCallback(msg):
    global remaining
    remaining = int(msg.data.secs)

def main():
    global remaining, score, debug

    try:
        namespace = "scorekeeper" # node_name/argsname
        debug = rospy.get_param('/jurong/debug')
        set_tag("[" + namespace.upper() + " SCRIPT]: ")
        #rospy.loginfo(TAG + namespace)
    except Exception as e:
        print(e)
        scenario = 'mbs'
        debug = True
        set_tag("[" + namespace.upper() + " SCRIPT]: ")

    rospy.init_node(namespace, anonymous=True)
    log_info(namespace)

    rate = rospy.Rate(5)
    rospy.Subscriber("/viz_score_totalled_time", Marker, callback)
    rospy.Subscriber("/gcs/mission_duration_remained", Duration, timeCallback)

    log_info("waiting")
    t = time.localtime()
    time_str = time.strftime("%d-%M_%H:%M:%S", t)
    rospy.wait_for_message("/viz_score_totalled_time", Marker)
    log_info("starting record")
    while remaining > 0:
        rate.sleep()

    try:
        scenario = rospy.get_param('/jurong/scenario')
    except Exception as e:
        print(e)
        scenario = 'unknown'

    with open('/home/dronesteam/ws_caric/custom_logs/score_'+scenario+'_'+time_str+'.csv', 'w') as f:
        
        score = [[sub[0].replace('Detected:    ', '')] for sub in score]
        score = [[sub[0].replace('Detected:   ', '')] for sub in score]
        score = [[sub[0].replace('Detected:  ', '')] for sub in score]
        score = [[sub[0].replace('Detected: ', '')] for sub in score]
        score = [[sub[0].replace(' / ', ',')] for sub in score]
        score = [[sub[0].replace('. Score: ', ',')] for sub in score]
        score = [[sub[0].replace('. Time: ', ',')] for sub in score]
        score = [[sub[0].replace(' s / ', ',')] for sub in score]
        score = [[sub[0].replace(' s', '')] for sub in score]
        
        # using csv.writer method from CSV package
        write = csv.writer(f)
        write.writerows(score)

    log_info("Score saved")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("terminating...")
    except Exception as e:
        print(e)
    finally:
        exit()